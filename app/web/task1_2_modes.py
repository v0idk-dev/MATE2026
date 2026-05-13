"""Task 1.2 — Video / Hybrid AI / AI-Only mode handlers.

Registered as a Flask blueprint. The Stereo mode lives in app.py; this file
adds the other three.

Modes:
  • video      — stack of N stereo pairs; runs the C++ binary on each pair,
                 averages plate 3D positions across pairs (weighted by per-
                 plate confidence), unions pipe segments, fuses scale.
  • hybrid_ai  — runs an AI to localize plates in 2D in both images; passes
                 those 2D coordinates into the C++ binary's "stereo" path
                 (skipping HSV detection) for classical triangulation.
  • ai_only    — sends images to AI and asks for full 3D scene; no
                 triangulation. Lower accuracy; flagged in the JSON.

All AI dispatching goes through the Electron main process on :5002 which
holds the Keychain-backed API keys. No keys ever touch Python.
"""
import json, os, subprocess, tempfile, base64, statistics
from typing import Any, Dict, List, Optional, Tuple

from flask import Blueprint, jsonify, request

bp = Blueprint('task1_2_modes', __name__)

# Shared with app.py — these globals are imported lazily to avoid a cycle.
def _shared():
    import app as _app
    return _app

# ────────────────────────────────────────────────────────────────────────
# Helpers
# ────────────────────────────────────────────────────────────────────────

def _save_uploaded(file_storage, suffix: str) -> str:
    f = tempfile.NamedTemporaryFile(delete=False, suffix=suffix)
    file_storage.save(f.name); f.close()
    return f.name

def _file_to_data_url(path: str) -> str:
    ext = os.path.splitext(path)[1].lstrip('.').lower()
    mime = 'image/png' if ext == 'png' else 'image/jpeg'
    with open(path, 'rb') as f:
        return f'data:{mime};base64,{base64.b64encode(f.read()).decode()}'

def _invoke_binary_stereo(left_img: str, right_img: str,
                          left_yaml: str, right_yaml: str,
                          stereo_yaml: str, opts: Dict[str, Any],
                          dbg_dir: Optional[str] = None) -> Dict[str, Any]:
    """Run the stereo binary on one pair; return parsed scene JSON."""
    a = _shared()
    cmd = [a.BINARY_PATH, '--mode', 'stereo',
           '--left', left_img, '--right', right_img,
           '--left-calib', left_yaml, '--right-calib', right_yaml,
           '--stereo-extrinsics', stereo_yaml,
           '--target-h', str(opts.get('target_h', 135)),
           '--hue-tolerance', str(opts.get('hue_tolerance', 25)),
           '--expected-plates', str(opts.get('expected_plates', 8)),
           '--plate-side-m', str(opts.get('plate_side_m', 0.10))]
    if opts.get('underwater'):
        cmd += ['--underwater', '--water-n', str(opts.get('water_n', 1.333))]
    if opts.get('plates_json'):
        cmd += ['--plates-json', opts['plates_json']]
    if opts.get('dense'):
        cmd += ['--dense']
        if opts.get('dense_voxel_m'):
            cmd += ['--dense-voxel-m', str(opts['dense_voxel_m'])]
    if dbg_dir:
        cmd += ['--debug-dir', dbg_dir]
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    if proc.returncode != 0:
        raise RuntimeError(f'binary failed (rc={proc.returncode}): {proc.stderr.strip()}')
    return json.loads(proc.stdout)

def _resolve_calibs():
    """Validate and return the configured calibration paths, or raise."""
    a = _shared()
    paths = a._resolve_calibration_paths()
    if not paths['stereo_yaml']:
        raise RuntimeError('No active stereo extrinsics. Configure in Settings → Photogrammetry.')
    if not paths['left_pkl'] or not paths['right_pkl']:
        raise RuntimeError('Left and Right per-camera calibrations required.')
    return paths

def _yaml_pair(paths: Dict[str, Any], w: int, h: int) -> Tuple[str, str]:
    a = _shared()
    return (a._pkl_to_yaml_cached(paths['left_id'],  w, h),
            a._pkl_to_yaml_cached(paths['right_id'], w, h))

# ────────────────────────────────────────────────────────────────────────
# Mode 1: VIDEO — stacked pairs, fused result
# ────────────────────────────────────────────────────────────────────────

@bp.route('/api/task1_2/analyze_video', methods=['POST'])
def analyze_video():
    """Accept N stereo pairs as left_image_0/right_image_0, left_image_1/...,
    run the binary on each, fuse results.

    Fusion:
      • plates: matched by detection-rank order (same as binary). Position
        averaged across pairs weighted by confidence.
      • pipes: simple union — each pair's pipes append. Future: cluster.
      • length/height: median across pairs.
      • scale: median of per-pair scale.k.
    """
    a = _shared()
    try:
        paths = _resolve_calibs()
    except RuntimeError as e:
        return jsonify({'error': str(e)}), 400

    # Collect pair indices.
    pair_idxs = sorted({int(k.rsplit('_', 1)[-1])
                        for k in request.files
                        if k.startswith('left_image_')})
    if not pair_idxs:
        return jsonify({'error': 'No pairs uploaded (expected left_image_0, right_image_0, …)'}), 400

    opts = {k: request.form.get(k) for k in
            ('target_h', 'hue_tolerance', 'expected_plates',
             'plate_side_m', 'underwater', 'water_n')}
    opts['underwater'] = request.form.get('underwater', '0') in ('1','true','True')

    tmp_files: List[str] = []
    tmp_dirs:  List[str] = []
    pair_scenes: List[Dict[str, Any]] = []
    try:
        import cv2
        for idx in pair_idxs:
            lf = request.files.get(f'left_image_{idx}')
            rf = request.files.get(f'right_image_{idx}')
            if not lf or not rf: continue
            l = _save_uploaded(lf, '.jpg'); r = _save_uploaded(rf, '.jpg')
            tmp_files += [l, r]
            img = cv2.imread(l)
            if img is None: continue
            h, w = img.shape[:2]
            ly, ry = _yaml_pair(paths, w, h)
            dbg = tempfile.mkdtemp(prefix=f'task1_2_v{idx}_'); tmp_dirs.append(dbg)
            try:
                scene = _invoke_binary_stereo(l, r, ly, ry, paths['stereo_yaml'], opts, dbg)
            except Exception as e:
                pair_scenes.append({'run':{'error':str(e),'pair_index':idx}})
                continue
            scene['_pair_index'] = idx
            scene['_dbg_dir']    = dbg
            scene['_left_path']  = l
            scene['_right_path'] = r
            pair_scenes.append(scene)

        if not pair_scenes:
            return jsonify({'error': 'no pair could be processed'}), 500

        fused = _fuse_video_pairs(pair_scenes)

        # Attach uploaded images per pair, debug images for the first pair only.
        fused['pairs'] = [{
            'pair_index': s.get('_pair_index'),
            'uploaded_images': {
                'left':  _file_to_data_url(s['_left_path'])  if s.get('_left_path')  else None,
                'right': _file_to_data_url(s['_right_path']) if s.get('_right_path') else None,
            },
            'plate_count': len(s.get('plates', [])),
            'pipe_count':  len(s.get('pipes', [])),
            'error':       (s.get('run') or {}).get('error', ''),
        } for s in pair_scenes]
        first = pair_scenes[0]
        if first.get('_dbg_dir') and os.path.isdir(first['_dbg_dir']):
            dbg_imgs = {}
            for f in sorted(os.listdir(first['_dbg_dir'])):
                if f.endswith(('.png','.jpg','.jpeg')):
                    dbg_imgs[f] = _file_to_data_url(os.path.join(first['_dbg_dir'], f))
            fused['debug_images'] = dbg_imgs
        fused['run_token'] = a._store_recent_run(fused)
        return jsonify(fused)
    finally:
        for p in tmp_files:
            try: os.unlink(p)
            except Exception: pass
        for d in tmp_dirs:
            try: import shutil; shutil.rmtree(d, ignore_errors=True)
            except Exception: pass

def _fuse_video_pairs(scenes: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Combine N per-pair scene dicts into one fused scene."""
    valid = [s for s in scenes if not (s.get('run') or {}).get('error')]
    if not valid:
        return {'run':{'mode':'video','error':'all pairs failed'},
                'plates':[],'pipes':[],'junctions':[]}

    out = {
        'run': {'mode': 'video', 'version': '0.4', 'warning': '',
                'pairs_processed': len(valid)},
        'calibration': valid[0].get('calibration', {}),
        'unit': valid[0].get('unit', 'm'),
        'underwater': valid[0].get('underwater', False),
        'water_n': valid[0].get('water_n', 0.0),
    }
    # Plates: match by index across pairs; average where has_3d.
    max_p = max(len(s.get('plates', [])) for s in valid)
    fused_plates = []
    for i in range(max_p):
        accs = []   # (conf, position_3d, corners_3d)
        center_l = center_r = None
        for s in valid:
            ps = s.get('plates', [])
            if i >= len(ps): continue
            p = ps[i]
            center_l = center_l or p.get('center_left')
            center_r = center_r or p.get('center_right')
            if p.get('has_3d') and p.get('position_3d'):
                accs.append((float(p.get('confidence', 50.0)),
                             p['position_3d'], p.get('corners_3d', [])))
        if accs:
            wsum = sum(w for w, _, _ in accs)
            avg_pos = {ax: sum(w * pos[ax] for w, pos, _ in accs) / wsum
                       for ax in ('x','y','z')}
            corners3d = []
            for ci in range(4):
                cs = [c[ci] for _, _, c in accs if c and len(c) == 4]
                if cs:
                    corners3d.append({ax: sum(c[ax] for c in cs)/len(cs)
                                      for ax in ('x','y','z')})
            fused_plates.append({
                'id': i, 'has_3d': True,
                'confidence': sum(w for w,_,_ in accs)/len(accs),
                'center_left': center_l or {'x':0,'y':0},
                'center_right': center_r or {'x':0,'y':0},
                'position_3d': avg_pos, 'corners_3d': corners3d,
            })
        else:
            fused_plates.append({'id': i, 'has_3d': False,
                'confidence': 0.0,
                'center_left': center_l or {'x':0,'y':0},
                'center_right': center_r or {'x':0,'y':0}})
    out['plates'] = fused_plates

    # Pipes + junctions: cluster endpoints across all pairs.
    out['pipes'], out['junctions'] = _cluster_pipes_junctions(valid)

    # length / height: median.
    lengths = [s.get('length', -1) for s in valid if s.get('length', -1) >= 0]
    heights = [s.get('height', -1) for s in valid if s.get('height', -1) >= 0]
    out['length'] = statistics.median(lengths) if lengths else -1.0
    out['height'] = statistics.median(heights) if heights else -1.0

    # scale: median of k.
    ks = [(s.get('scale') or {}).get('k', 1.0) for s in valid]
    out['scale'] = {
        'k': statistics.median(ks) if ks else 1.0,
        'confidence': min(0.7, 0.3 + 0.05 * len(valid)),
        'observations_used': len(valid),
        'source': 'video-fusion',
        'reason': f'fused {len(valid)} pairs',
    }
    return out

# ────────────────────────────────────────────────────────────────────────
# Mode 2: HYBRID AI — AI does plate localization, classical does geometry
# ────────────────────────────────────────────────────────────────────────

@bp.route('/api/task1_2/analyze_hybrid', methods=['POST'])
def analyze_hybrid():
    a = _shared()
    if 'left_image' not in request.files or 'right_image' not in request.files:
        return jsonify({'error':'left_image and right_image required'}), 400
    try:
        paths = _resolve_calibs()
    except RuntimeError as e:
        return jsonify({'error': str(e)}), 400

    ai_model = request.form.get('ai_model', '')
    if ':' not in ai_model:
        return jsonify({'error':'ai_model required, e.g. "openai:gpt-4o"'}), 400
    provider, model = ai_model.split(':', 1)

    lf = _save_uploaded(request.files['left_image'],  '.jpg')
    rf = _save_uploaded(request.files['right_image'], '.jpg')
    try:
        # 1. Ask AI for plate 2D localizations in both images.
        ai_json = _invoke_ai_caller(provider, model, 'hybrid', [lf, rf])

        # 2. Write AI's plate 2D coords to a temp file, pass via --plates-json
        #    so the C++ binary skips HSV and uses these directly.
        import cv2
        img = cv2.imread(lf); h, w = img.shape[:2]
        ly, ry = _yaml_pair(paths, w, h)
        plates_file = tempfile.NamedTemporaryFile(
            mode='w', delete=False, suffix='.json')
        json.dump({'plates': ai_json.get('plates', [])}, plates_file)
        plates_file.close()
        opts = {
            'target_h': request.form.get('target_h', 135),
            'hue_tolerance': request.form.get('hue_tolerance', 25),
            'expected_plates': len(ai_json.get('plates', [])) or 8,
            'plate_side_m': request.form.get('plate_side_m', 0.10),
            'underwater': request.form.get('underwater','0') in ('1','true','True'),
            'water_n': request.form.get('water_n', 1.333),
            'plates_json': plates_file.name,
        }
        dbg = tempfile.mkdtemp(prefix='task1_2_hybrid_')
        try:
            scene = _invoke_binary_stereo(lf, rf, ly, ry, paths['stereo_yaml'], opts, dbg)
        finally:
            try: os.unlink(plates_file.name)
            except Exception: pass

        # 3. Annotate AI guidance.
        scene['ai_assist'] = {
            'provider': provider, 'model': model,
            'plates_seen_by_ai': len(ai_json.get('plates', [])),
            'warnings': ai_json.get('warnings', []),
        }
        scene.setdefault('run', {})['mode'] = 'hybrid_ai'
        # Attach images.
        scene['uploaded_images'] = {'left': _file_to_data_url(lf),
                                    'right': _file_to_data_url(rf)}
        if os.path.isdir(dbg):
            dbg_imgs = {}
            for f in sorted(os.listdir(dbg)):
                if f.endswith(('.png','.jpg','.jpeg')):
                    dbg_imgs[f] = _file_to_data_url(os.path.join(dbg, f))
            scene['debug_images'] = dbg_imgs
        scene['run_token'] = a._store_recent_run(scene)
        return jsonify(scene)
    except Exception as e:
        return jsonify({'error': f'hybrid_ai failed: {e}'}), 500
    finally:
        for p in (lf, rf):
            try: os.unlink(p)
            except Exception: pass

# ────────────────────────────────────────────────────────────────────────
# Mode 3: AI ONLY — model returns the entire 3D scene
# ────────────────────────────────────────────────────────────────────────

@bp.route('/api/task1_2/analyze_ai_only', methods=['POST'])
def analyze_ai_only():
    a = _shared()
    files = [f for k, f in request.files.items()
             if k.startswith('image') or k in ('left_image','right_image')]
    if not files:
        return jsonify({'error':'at least one image required'}), 400
    ai_model = request.form.get('ai_model', '')
    if ':' not in ai_model:
        return jsonify({'error':'ai_model required'}), 400
    provider, model = ai_model.split(':', 1)

    saved = [_save_uploaded(f, '.jpg') for f in files]
    try:
        result = _invoke_ai_caller(provider, model, 'ai_only', saved)
        scene = {
            'run': {'mode':'ai_only', 'version':'0.4',
                    'warning':'AI-only mode: do NOT trust for sub-cm accuracy'},
            'calibration': {'present': False},
            'unit': 'm',
            'plates':    [_normalize_ai_plate(p) for p in result.get('plates', [])],
            'pipes':     [_normalize_ai_pipe(i, p) for i, p in enumerate(result.get('pipes', []))],
            'junctions': [],
            'length':    float(result.get('length', -1) or -1),
            'height':    float(result.get('height', -1) or -1),
            'scale':     {'k':1.0,'confidence':0.1,'source':'ai-only',
                          'reason':'AI direct output'},
            'underwater': False, 'water_n': 0.0,
            'ai_assist': {'provider': provider, 'model': model,
                          'warnings': result.get('warnings', [])},
            'uploaded_images': {f'image_{i}': _file_to_data_url(p)
                                for i, p in enumerate(saved)},
        }
        scene['run_token'] = a._store_recent_run(scene)
        return jsonify(scene)
    except Exception as e:
        return jsonify({'error': f'ai_only failed: {e}'}), 500
    finally:
        for p in saved:
            try: os.unlink(p)
            except Exception: pass

def _normalize_ai_plate(p):
    pos = p.get('position_3d') or {'x':0,'y':0,'z':0}
    corners = p.get('corners_3d') or []
    return {
        'id': int(p.get('id', 0)),
        'has_3d': True,
        'confidence': 50.0,
        'center_left':  {'x':0,'y':0},
        'center_right': {'x':0,'y':0},
        'position_3d': pos,
        'corners_3d':  corners,
    }

def _normalize_ai_pipe(i, p):
    return {
        'id': i, 'junction_a': -1, 'junction_b': -1,
        'a': p.get('a') or {'x':0,'y':0,'z':0},
        'b': p.get('b') or {'x':0,'y':0,'z':0},
        'length': p.get('length', 0.0),
    }

def _invoke_ai_caller(provider, model, kind, image_paths):
    """Spawn ai_caller.py in the standalone python runtime and parse JSON."""
    a = _shared()
    cmd = [a._TASK1_2_PYTHON,
           os.path.join(a.PROJECT_ROOT, 'scripts','task1_2','python','ai_caller.py'),
           provider, model, kind] + image_paths
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
    if proc.returncode != 0:
        raise RuntimeError(f'ai_caller failed: {proc.stderr.strip() or proc.stdout.strip()}')
    return json.loads(proc.stdout)
