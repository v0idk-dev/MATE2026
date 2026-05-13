# task1_2.py — Flask blueprint for the overhauled single-flow Task 1.2.
#
# Replaces the multi-mode `task1_2_modes.py` with a single endpoint
# (/api/task1_2/analyze) that accepts N stereo pairs, runs the C++
# pipeline once, and returns the Custom JSON model + (optionally) a GLB
# blob URL.
#
# Other blueprints/files are NOT touched. This module is intentionally
# self-contained so it can be wired into the existing Flask app with one
# line:
#     from task1_2 import task1_2_bp
#     app.register_blueprint(task1_2_bp)
#
# Endpoints
# ---------
#   GET  /api/task1_2/health                — pipeline binary present?
#   POST /api/task1_2/analyze               — multipart upload of pairs
#                                              + JSON config; returns model
#   POST /api/task1_2/import                — accept user JSON model
#   POST /api/task1_2/manual_width          — apply step-9 manual override
#   GET  /api/task1_2/export/<job_id>.<ext> — download model in JSON/GLB/OBJ
#
# All heavy lifting is delegated to the compiled `task1_2` binary via
# subprocess — no Python triangulation here.
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import time
import uuid
from pathlib import Path
from flask import Blueprint, jsonify, request, send_file

task1_2_bp = Blueprint("task1_2", __name__, url_prefix="/api/task1_2")

# Resolved at import; can also be overridden via env vars.
_HERE         = Path(__file__).resolve().parent
_BIN_CANDS    = [
    Path(os.environ["TASK1_2_BIN"]) if os.environ.get("TASK1_2_BIN") else None,
    _HERE.parent / "scripts" / "task1_2" / "task1_2",
    _HERE.parent.parent / "scripts" / "task1_2" / "task1_2",
]
_AI_SCRIPT    = (_HERE.parent / "scripts" / "task1_2" / "python" / "ai_caller.py")
_PKL_TO_YAML  = (_HERE.parent / "scripts" / "task1_2" / "python" / "pkl_to_yaml.py")
_PYTHON       = os.environ.get("TASK1_2_PYTHON", sys.executable)


def _materialize_intrinsics_yaml(file_storage, dest_yaml: Path,
                                 image_w: int = 0, image_h: int = 0) -> bool:
    """Save an uploaded per-camera calibration to ``dest_yaml``.

    Accepts either a YAML the C++ side reads directly, or a ``.pkl``
    produced by ``camera_calibration.py`` — in which case we shell out to
    ``pkl_to_yaml.py`` to convert. Returns True on success, False if no
    file was provided. Raises RuntimeError on conversion failure so the
    caller can return a structured 400.
    """
    if file_storage is None:
        return False
    name = (file_storage.filename or "").lower()
    if name.endswith(".pkl"):
        tmp_pkl = dest_yaml.with_suffix(".pkl")
        file_storage.save(tmp_pkl)
        if not _PKL_TO_YAML.exists():
            raise RuntimeError(f"pkl_to_yaml.py not found at {_PKL_TO_YAML}")
        cmd = [_PYTHON, str(_PKL_TO_YAML), str(tmp_pkl), str(dest_yaml)]
        if image_w: cmd += ["--width",  str(int(image_w))]
        if image_h: cmd += ["--height", str(int(image_h))]
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
        try: tmp_pkl.unlink()
        except OSError: pass
        if proc.returncode != 0 or not dest_yaml.exists():
            raise RuntimeError(
                f"pkl_to_yaml.py failed (rc={proc.returncode}): "
                f"{(proc.stderr or proc.stdout).strip()[-400:]}")
        return True
    # Treat anything else as raw YAML; let cv::FileStorage on the C++
    # side reject it if malformed.
    file_storage.save(dest_yaml)
    return True


def _peek_image_size(path: Path):
    """Return (w, h) of an image file without depending on cv2/PIL.
    Falls back to (0, 0) if the format isn't recognized.
    """
    try:
        with open(path, "rb") as f:
            head = f.read(64)
        # JPEG: scan for SOF0/SOF2 marker.
        if head[:2] == b"\xff\xd8":
            with open(path, "rb") as f:
                data = f.read()
            i = 2
            while i < len(data) - 9:
                if data[i] != 0xFF:
                    i += 1; continue
                marker = data[i+1]
                if marker in (0xC0, 0xC2):
                    h = (data[i+5] << 8) | data[i+6]
                    w = (data[i+7] << 8) | data[i+8]
                    return w, h
                # Skip segment.
                seg_len = (data[i+2] << 8) | data[i+3]
                i += 2 + seg_len
        # PNG.
        if head[:8] == b"\x89PNG\r\n\x1a\n":
            w = int.from_bytes(head[16:20], "big")
            h = int.from_bytes(head[20:24], "big")
            return w, h
    except Exception:
        pass
    return 0, 0

# Per-job working dirs (cleaned up on app exit by the OS tmp).
_JOB_ROOT     = Path(tempfile.gettempdir()) / "mate_task1_2_jobs"
_JOB_ROOT.mkdir(exist_ok=True)


def _bin_path():
    for c in _BIN_CANDS:
        if c and c.is_file() and os.access(c, os.X_OK):
            return c
    return None


# ─── DEMO FALLBACK ─────────────────────────────────────────────────────
# When the native binary isn't built (e.g. you're running this in Replit
# on Linux instead of on your Mac), we synthesize a plausible 3-section
# Model3D so the UI is fully clickable end-to-end. The shape matches the
# spec sketch: 3 cuboid sections of varying heights on a shared base
# axis, with 8 plates distributed across their faces.
def _synthetic_model(n_pairs: int, cfg: dict) -> dict:
    sections = [
        {"id": 0, "origin": [0.30, 0.18, 0.10], "size": [0.60, 0.36, 0.20],
         "yaw_deg": 0.0, "confidence": 0.82},
        {"id": 1, "origin": [0.95, 0.18, 0.18], "size": [0.55, 0.36, 0.36],
         "yaw_deg": 0.0, "confidence": 0.78},
        {"id": 2, "origin": [1.55, 0.18, 0.13], "size": [0.65, 0.36, 0.26],
         "yaw_deg": 0.0, "confidence": 0.74},
    ]
    plate_sites = [
        (0, "+z", 0.30, 0.50), (0, "+z", 0.70, 0.50),
        (1, "+x", 0.50, 0.55), (1, "+z", 0.40, 0.40),
        (1, "-y", 0.50, 0.60), (2, "+z", 0.30, 0.50),
        (2, "+z", 0.70, 0.50), (2, "+x", 0.50, 0.50),
    ]
    plates = []
    for i, (sid, face, u, v) in enumerate(plate_sites):
        plates.append({
            "id": i, "section_id": sid, "face": face,
            "u": u, "v": v, "side_m": 0.10,
            "confidence": 0.7 + 0.03 * (i % 4),
        })
    length = max(s["origin"][0] + s["size"][0] * 0.5 for s in sections) - \
             min(s["origin"][0] - s["size"][0] * 0.5 for s in sections)
    width  = max(s["size"][1] for s in sections)
    height = max(s["origin"][2] + s["size"][2] * 0.5 for s in sections)

    manual_w = float(cfg.get("manual_width_m") or 0)
    if manual_w > 0:
        k = manual_w / max(width, 1e-6)
        for s in sections:
            s["origin"] = [s["origin"][0] * k, s["origin"][1] * k, s["origin"][2] * k]
            s["size"]   = [s["size"][0]   * k, s["size"][1]   * k, s["size"][2]   * k]
        for p in plates:
            p["side_m"] *= k
        length *= k; width *= k; height *= k

    return {
        "version": 1,
        "unit": "m",
        "sections": sections,
        "plates": plates,
        "totals": {"length": length, "width": width, "height": height},
        "scale": {
            "k": 1.0,
            "source": "manual" if manual_w > 0 else "stereo",
            "confidence": 1.0 if manual_w > 0 else 0.65,
            "reason": "demo synthetic model (native binary not built)",
        },
        "calibration": {"present": False, "rms_px": -1.0, "baseline_m": 0.0},
        "n_pairs_used": max(1, n_pairs),
        "warning": "DEMO MODE: native binary not built; serving synthetic model.",
    }


def _save_uploads(job_dir, files, prefix):
    """Save FileStorage list to job_dir; return saved paths in upload order."""
    out = []
    for i, f in enumerate(files):
        ext = (Path(f.filename or "img.jpg").suffix or ".jpg").lower()
        p = job_dir / f"{prefix}_{i:02d}{ext}"
        f.save(p)
        out.append(str(p))
    return out


@task1_2_bp.get("/health")
def health():
    bin_p = _bin_path()
    return jsonify({
        "binary_present": bin_p is not None,
        "binary_path":    str(bin_p) if bin_p else None,
        "ai_script":      str(_AI_SCRIPT) if _AI_SCRIPT.exists() else None,
        "python":         _PYTHON,
    })


@task1_2_bp.post("/analyze")
def analyze():
    """Multipart fields:
        lefts[]              : N left images
        rights[]             : N right images (same count as lefts)
        left_calib           : YAML
        right_calib          : YAML
        stereo_calib         : YAML  (optional)
        config               : JSON  (knobs; see below)
    config keys: target_hue (int), hue_tol (int), expected_plates (int),
                 underwater (bool), water_n (float),
                 ai_enhance (bool), ai_provider (str), ai_model (str),
                 use_apple_intelligence (bool),
                 manual_width_m (float), use_metal (bool).
    """
    bin_p = _bin_path()
    lefts  = request.files.getlist("lefts[]")  or request.files.getlist("lefts")
    rights = request.files.getlist("rights[]") or request.files.getlist("rights")
    if not lefts or len(lefts) != len(rights):
        return jsonify({"error": f"need same count of lefts/rights (got {len(lefts)}/{len(rights)})"}), 400

    try:
        cfg = json.loads(request.form.get("config") or "{}")
    except Exception as e:
        return jsonify({"error": f"bad config JSON: {e}"}), 400

    # ── Demo fallback (no native binary) ───────────────────────────
    if bin_p is None:
        job_id  = uuid.uuid4().hex[:12]
        job_dir = _JOB_ROOT / job_id
        job_dir.mkdir(parents=True, exist_ok=True)
        model = _synthetic_model(len(lefts), cfg)
        (job_dir / "model.json").write_text(json.dumps(model, indent=2))
        return jsonify({
            "job_id": job_id, "model": model,
            "stderr": "[demo mode] native task1_2 binary not present; "
                      "synthesized Model3D. Build via `make -C scripts/task1_2` on macOS.",
            "elapsed_ms": 50, "debug_files": [],
            "exports": {
                "json": f"/api/task1_2/export/{job_id}.json",
                "glb":  f"/api/task1_2/export/{job_id}.glb",
                "obj":  f"/api/task1_2/export/{job_id}.obj",
            },
        })

    job_id  = uuid.uuid4().hex[:12]
    job_dir = _JOB_ROOT / job_id
    job_dir.mkdir(parents=True, exist_ok=True)
    debug_dir = job_dir / "debug"; debug_dir.mkdir(exist_ok=True)

    L = _save_uploads(job_dir, lefts,  "L")
    R = _save_uploads(job_dir, rights, "R")

    # Per-camera calibrations may be uploaded as either YAML (read directly
    # by the C++ side) or PKL (the format produced by camera_calibration.py;
    # we convert to YAML via pkl_to_yaml.py). Image dims come from the first
    # left frame so the rectifier picks the right principal-point hint.
    img_w, img_h = (_peek_image_size(Path(L[0])) if L else (0, 0))
    lc = job_dir / "left.yaml"
    rc = job_dir / "right.yaml"
    try:
        if not _materialize_intrinsics_yaml(
                request.files.get("left_calib"),  lc, img_w, img_h):
            lc.write_text("")
        if not _materialize_intrinsics_yaml(
                request.files.get("right_calib"), rc, img_w, img_h):
            rc.write_text("")
    except RuntimeError as e:
        return jsonify({"error": f"calibration conversion failed: {e}"}), 400

    # Fall back to settings-assigned calibrations when nothing was uploaded.
    try:
        from app import _resolve_calibration_paths, _pkl_to_yaml_cached
        settings_calib = _resolve_calibration_paths()
    except Exception:
        settings_calib = {}

    if not lc.stat().st_size and settings_calib.get("left_id"):
        try:
            yaml_src = _pkl_to_yaml_cached(settings_calib["left_id"], img_w, img_h)
            shutil.copy(yaml_src, lc)
        except Exception:
            pass

    if not rc.stat().st_size and settings_calib.get("right_id"):
        try:
            yaml_src = _pkl_to_yaml_cached(settings_calib["right_id"], img_w, img_h)
            shutil.copy(yaml_src, rc)
        except Exception:
            pass

    # Stereo extrinsics is always YAML — saved as-is.
    sc = job_dir / "stereo.yaml"
    if "stereo_calib" in request.files:
        request.files["stereo_calib"].save(sc)
    elif settings_calib.get("stereo_yaml"):
        shutil.copy(settings_calib["stereo_yaml"], sc)

    out_json = job_dir / "model.json"
    out_glb  = job_dir / "model.glb"
    out_obj  = job_dir / "model.obj"

    cmd = [str(bin_p)]
    for li, ri in zip(L, R): cmd += ["--pair", li, ri]
    if lc.stat().st_size: cmd += ["--left-calib", str(lc)]
    if rc.stat().st_size: cmd += ["--right-calib", str(rc)]
    if sc.exists() and sc.stat().st_size: cmd += ["--stereo", str(sc)]
    cmd += ["--out", str(out_json), "--glb", str(out_glb), "--obj", str(out_obj),
            "--debug-dir", str(debug_dir)]
    if "target_hue" in cfg:        cmd += ["--target-hue", str(int(cfg["target_hue"]))]
    if "hue_tol"    in cfg:        cmd += ["--hue-tol",    str(int(cfg["hue_tol"]))]
    if "expected_plates" in cfg:   cmd += ["--expected-plates", str(int(cfg["expected_plates"]))]
    if "plate_side_m"   in cfg:   cmd += ["--plate-side-m",    str(float(cfg["plate_side_m"]))]
    if cfg.get("underwater"):      cmd += ["--underwater"]
    if "water_n" in cfg:           cmd += ["--water-n", str(float(cfg["water_n"]))]
    if cfg.get("use_apple_intelligence"): cmd += ["--apple-intelligence"]
    if cfg.get("ai_enhance"):
        cmd += ["--ai-enhance"]
        if cfg.get("ai_provider"): cmd += ["--ai-provider", str(cfg["ai_provider"])]
        if cfg.get("ai_model"):    cmd += ["--ai-model",    str(cfg["ai_model"])]
        cmd += ["--python", _PYTHON, "--ai-script", str(_AI_SCRIPT)]
    if "manual_width_m" in cfg and float(cfg["manual_width_m"]) > 0:
        cmd += ["--manual-width-m", str(float(cfg["manual_width_m"]))]
    if cfg.get("use_metal") is False: cmd += ["--no-metal"]
    # Manhattan-world auto-calibration fallback (kicks in only when no
    # stereo/intrinsics YAMLs were supplied). The user always knows the
    # physical L↔R camera separation on their rig — pass it through so
    # absolute scale is still correct in fallback mode. Default 0.10 m
    # mirrors the C++ default; pass 0 to disable the fallback entirely
    # and force the legacy pixel-units behavior.
    if "rig_baseline_m" in cfg:
        b = float(cfg["rig_baseline_m"])
        if b > 0: cmd += ["--rig-baseline-m", str(b)]
        else:     cmd += ["--no-auto-calib"]
    if cfg.get("engine") in ("pipe", "plate"):
        cmd += ["--engine", cfg["engine"]]

    t0 = time.time()
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
    t = int((time.time() - t0) * 1000)

    if proc.returncode != 0 or not out_json.exists():
        return jsonify({
            "error":     "pipeline failed",
            "stderr":    proc.stderr[-4000:],
            "stdout":    proc.stdout[-1000:],
            "command":   cmd,
        }), 500

    try:
        model = json.loads(out_json.read_text())
    except Exception as e:
        return jsonify({"error": f"bad model JSON: {e}"}), 500

    # List debug images for the front-end to render.
    debug_files = sorted([p.name for p in debug_dir.glob("*.jpg")])

    # Per-pair detection counts (written by main.cpp). Lets the front-end
    # explain *why* the fused model is empty — e.g. all-zero plate counts
    # almost always means the Plate Color picker doesn't match the real
    # plate hue. Missing file is non-fatal (older binary).
    per_pair = []
    pp_path = job_dir / "per_pair.json"
    if pp_path.exists():
        try: per_pair = json.loads(pp_path.read_text())
        except Exception: per_pair = []

    return jsonify({
        "job_id":   job_id,
        "model":    model,
        "stderr":   proc.stderr[-4000:],
        "elapsed_ms": t,
        "debug_files": debug_files,
        "per_pair":    per_pair,
        "exports": {
            "json": f"/api/task1_2/export/{job_id}.json",
            "glb":  f"/api/task1_2/export/{job_id}.glb",
            "obj":  f"/api/task1_2/export/{job_id}.obj",
        },
    })


@task1_2_bp.post("/import")
def import_model():
    """Validate user-provided custom JSON; pass-through if valid."""
    try:
        data = request.get_json(force=True)
        if not isinstance(data, dict) or "sections" not in data or "plates" not in data:
            return jsonify({"error": "missing sections/plates"}), 400
        return jsonify({"model": data})
    except Exception as e:
        return jsonify({"error": str(e)}), 400


@task1_2_bp.post("/manual_width")
def manual_width():
    """Apply step-9 width override to an already-loaded model and return it.
    Body: { model: {...}, width_m: float }"""
    body = request.get_json(force=True) or {}
    m = body.get("model") or {}
    w = float(body.get("width_m") or 0)
    if w <= 0 or "totals" not in m:
        return jsonify({"error": "need model + positive width_m"}), 400
    cur_w = float(m["totals"].get("width") or 0)
    if cur_w <= 1e-6:
        return jsonify({"error": "model width is degenerate"}), 400
    k = w / cur_w
    # Apply uniform scale in-Python (mirrors C++ applyScale).
    def sv(v): return [v[0]*k, v[1]*k, v[2]*k]
    for s in m.get("sections", []):
        s["origin"] = sv(s["origin"]); s["size"] = sv(s["size"])
    for p in m.get("plates", []):
        p["side_m"] = float(p.get("side_m", 0.10)) * k
        if "corners" in p: p["corners"] = [sv(c) for c in p["corners"]]
    for key in ("length", "width", "height"):
        m["totals"][key] = float(m["totals"].get(key, 0)) * k
    m.setdefault("scale", {})
    m["scale"]["k"]      = float(m["scale"].get("k", 1.0)) * k
    m["scale"]["source"] = "manual"
    m["scale"]["confidence"] = 1.0
    m["scale"]["reason"] = "user-supplied total width override"
    return jsonify({"model": m})


@task1_2_bp.get("/export/<job_id>.<ext>")
def export_artifact(job_id, ext):
    """Return JSON/GLB/OBJ produced by an earlier /analyze call."""
    if ext not in ("json", "glb", "obj", "mtl"):
        return jsonify({"error": "bad ext"}), 400
    p = _JOB_ROOT / job_id / f"model.{ext}"
    if not p.exists():
        # Demo mode: synthesize a minimal placeholder for non-JSON exports
        # so the UI download buttons don't 404 when the binary is absent.
        if ext == "json" or not (_JOB_ROOT / job_id / "model.json").exists():
            return jsonify({"error": "expired or unknown job"}), 404
        model_path = _JOB_ROOT / job_id / "model.json"
        if ext == "obj":
            p.write_text("# demo OBJ — native binary not built\n")
        elif ext == "glb":
            p.write_bytes(b"glTF\x02\x00\x00\x00\x00\x00\x00\x00")
        else:
            return jsonify({"error": "expired or unknown job"}), 404
    mt = {"json": "application/json",
          "glb":  "model/gltf-binary",
          "obj":  "text/plain",
          "mtl":  "text/plain"}[ext]
    return send_file(p, mimetype=mt, as_attachment=True, download_name=f"coral_garden.{ext}")


@task1_2_bp.get("/debug/<job_id>/<name>")
def debug_image(job_id, name):
    """Serve per-pair debug images written by the pipeline."""
    if "/" in name or ".." in name:
        return jsonify({"error": "bad name"}), 400
    p = _JOB_ROOT / job_id / "debug" / name
    if not p.exists():
        return jsonify({"error": "not found"}), 404
    return send_file(p, mimetype="image/jpeg")
