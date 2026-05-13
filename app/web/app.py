import os
import base64
import subprocess
import re
import time
import tempfile
import threading
import pickle
import json
import cv2
import numpy as np
from flask import Flask, request, jsonify, render_template, Response, stream_with_context

app = Flask(__name__,
            template_folder='templates',
            static_folder='static')

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BINARY_PATH = os.path.join(PROJECT_ROOT, 'scripts', 'task1_2', 'stereo_distance')
TASK1_2_DIR = os.path.join(PROJECT_ROOT, 'scripts', 'task1_2')

# ── App-support paths (mirrors what SettingsHelper uses) ─────────────────────
_APP_SUPPORT_DIR = os.path.expanduser(
    '~/Library/Application Support/mate-2026-robot-controller'
)
_SETTINGS_JSON  = os.path.join(_APP_SUPPORT_DIR, 'settings.json')
_UNDISTORT_DIR  = os.path.join(_APP_SUPPORT_DIR, 'undistort')
os.makedirs(_UNDISTORT_DIR, exist_ok=True)

# ── Camera state ──────────────────────────────────────────────────────────────

CHANNELS = ('CH01', 'CH02', 'CH03', 'CH04')

_cam_lock   = threading.Lock()   # guards _cam_cap / _cam_index / _cam_seq
_cam_cap    = None
_cam_index  = None
_cam_seq    = 0

# Background frame-reader thread — continuously grabs into _latest_frame so all
# MJPEG generator threads can read without ever blocking each other on cap.read().
_frame_lock   = threading.Lock()   # guards _latest_frame / _frame_seq
_latest_frame = None
_frame_seq    = 0
_reader_thread = None

_undistort_raw  = None      # raw (mtx, dist) from pkl — recompute maps when frame size changes
_undistort_maps = {}        # (w,h) → (mapx, mapy, roi)
_undistort_id   = None      # active UUID (no extension)
_distortion_on  = False

def _settings_read() -> dict:
    try:
        with open(_SETTINGS_JSON) as f:
            return json.load(f)
    except Exception:
        return {}

# ── Camera listing via system_profiler (fast, no Swift compile) ───────────────
# system_profiler SPCameraDataType -json returns the macOS camera list in <1s.
# We cross-reference with AVFoundation order via a cached Swift run (only on
# first call or when the list changes), so cv2 indices stay correct.

_cam_list_cache = []   # last known [{uniqueID, name, builtin}]
_cam_list_lock  = threading.Lock()

def list_cameras():
    """Return [{uniqueID, name, builtin}] via the Electron addon's AVFoundation call.
    The Electron main process exposes this on 127.0.0.1:5002/cameras."""
    import urllib.request
    try:
        with urllib.request.urlopen('http://127.0.0.1:5002/cameras', timeout=3) as r:
            cameras = json.loads(r.read().decode())
        with _cam_list_lock:
            if cameras:
                _cam_list_cache[:] = cameras
        return cameras
    except Exception:
        with _cam_list_lock:
            return list(_cam_list_cache)

# ── Background frame reader ───────────────────────────────────────────────────

def _bg_reader():
    """Runs in a daemon thread. Continuously grabs frames so generators never
    block each other waiting on cap.read().
    Holds _cam_lock during cap.read() to prevent _open_camera from releasing
    the cap under us, but releases immediately after so camera switches aren't
    blocked for long."""
    global _latest_frame, _frame_seq
    while True:
        with _cam_lock:
            cap = _cam_cap
            if cap is None or not cap.isOpened():
                # Release lock before sleeping
                pass
            else:
                ret, frame = cap.read()
                if ret and frame is not None:
                    with _frame_lock:
                        _latest_frame = frame
                        _frame_seq   += 1
                # Lock releases here automatically; no sleep so we grab next frame fast
                continue
        # Only reach here when cap is None/not open
        time.sleep(0.05)

def _start_reader():
    global _reader_thread
    if _reader_thread is not None and _reader_thread.is_alive():
        return
    _reader_thread = threading.Thread(target=_bg_reader, daemon=True)
    _reader_thread.start()

def _open_camera(avf_index: int):
    global _cam_cap, _cam_index, _cam_seq, _latest_frame, _frame_seq
    with _cam_lock:
        if _cam_cap is not None:
            _cam_cap.release()
            _cam_cap = None
            _cam_index = None
        cap = cv2.VideoCapture(avf_index, cv2.CAP_AVFOUNDATION)
        if cap.isOpened():
            _cam_cap   = cap
            _cam_index = avf_index
            _cam_seq  += 1
    with _frame_lock:
        _latest_frame = None
        _frame_seq    = 0
    _start_reader()

def _close_camera():
    global _cam_cap, _cam_index, _cam_seq, _latest_frame
    with _cam_lock:
        if _cam_cap is not None:
            _cam_cap.release()
            _cam_cap   = None
            _cam_index = None
            _cam_seq  += 1
    with _frame_lock:
        _latest_frame = None

def _get_frame():
    with _frame_lock:
        frame = _latest_frame
        seq   = _frame_seq
    return (frame.copy() if frame is not None else None), seq

def _split_quad(frame):
    h, w = frame.shape[:2]
    mh, mw = h // 2, w // 2
    return {
        'CH01': frame[0:mh,  0:mw],
        'CH02': frame[0:mh,  mw:w],
        'CH03': frame[mh:h,  0:mw],
        'CH04': frame[mh:h,  mw:w],
    }

def _get_undistort_maps(w: int, h: int):
    """Return (mapx, mapy, roi) precomputed for this frame size, building if needed."""
    key = (w, h)
    if key not in _undistort_maps and _undistort_raw is not None:
        mtx, dist = _undistort_raw
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
        mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w, h), 5)
        _undistort_maps[key] = (mapx, mapy, roi)
    return _undistort_maps.get(key)

def _apply_undistort(frame):
    if _undistort_raw is None:
        return frame
    h, w = frame.shape[:2]
    maps = _get_undistort_maps(w, h)
    if maps is None:
        return frame
    mapx, mapy, roi = maps
    out = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)
    x, y, rw, rh = roi
    if rw > 0 and rh > 0:
        out = out[y:y+rh, x:x+rw]
        out = cv2.resize(out, (w, h))
    return out

def _load_undistort_pkl(pkl_path: str, file_id: str):
    """Load raw calibration data from a .pkl. Maps are computed lazily per frame size."""
    global _undistort_raw, _undistort_maps, _undistort_id
    with open(pkl_path, 'rb') as f:
        cal = pickle.load(f)
    _undistort_raw  = (cal['camera_matrix'], cal['distortion_coefficients'])
    _undistort_maps = {}   # invalidate cached maps for old frame sizes
    _undistort_id   = file_id

def _clear_undistort():
    global _undistort_raw, _undistort_maps, _undistort_id
    _undistort_raw  = None
    _undistort_maps = {}
    _undistort_id   = None

def _init_from_settings():
    """On startup, read settings.json and activate the selected undistort PKL if any."""
    s = _settings_read()
    active_id = s.get('activeUndistortId')
    if active_id:
        pkl_path = os.path.join(_UNDISTORT_DIR, active_id + '.pkl')
        if os.path.exists(pkl_path):
            try:
                _load_undistort_pkl(pkl_path, active_id)
            except Exception:
                pass

ALLOWED_EXTENSIONS = {'png', 'jpg', 'jpeg', 'bmp', 'tiff', 'tif'}

def allowed_file(filename):
    return '.' in filename and filename.rsplit('.', 1)[1].lower() in ALLOWED_EXTENSIONS

# ── Camera API ────────────────────────────────────────────────────────────────

_CAMERAS_JSON = os.path.join(PROJECT_ROOT, 'cameras.json')

def _read_cameras_config() -> dict:
    try:
        with open(_CAMERAS_JSON) as f:
            return json.load(f)
    except Exception:
        return {}

@app.route('/api/cameras_defaults')
def api_cameras_defaults():
    return jsonify(_read_cameras_config())

@app.route('/api/cameras_config')
def api_cameras_config():
    cfg = _read_cameras_config()
    # Merge overrides from settings.json (written by Swift General pane)
    s = _settings_read()
    slots = s.get('defaultCameraSlots')
    if slots:
        cfg.setdefault('defaultView', {})
        for k in ('TL', 'TR', 'BL', 'BR'):
            v = slots.get(k)
            if v is not None:
                cfg['defaultView'][k] = v or None
    sc = s.get('screenshotCameras')
    if sc:
        cfg.setdefault('screenshots', {})
        if sc.get('crabChannel'):
            cfg['screenshots'].setdefault('crabDetection', {})['splitChannel'] = sc['crabChannel']
        if sc.get('photogrammetryLeft'):
            cfg['screenshots'].setdefault('photogrammetryLeft', {})['channel'] = sc['photogrammetryLeft']
        if sc.get('photogrammetryRight'):
            cfg['screenshots'].setdefault('photogrammetryRight', {})['channel'] = sc['photogrammetryRight']
    return jsonify(cfg)

@app.route('/api/cameras')
def api_cameras():
    cams = list_cameras()
    return jsonify({'cameras': cams})

@app.route('/api/camera/select', methods=['POST'])
def api_camera_select():
    data = request.get_json(force=True)
    unique_id = data.get('uniqueID')
    if not unique_id:
        _close_camera()
        return jsonify({'ok': True, 'uniqueID': None})
    cam = next((c for c in list_cameras() if c.get('uniqueID') == unique_id), None)
    if cam is None:
        return jsonify({'error': 'Camera not found'}), 404
    _open_camera(int(cam['index']))
    return jsonify({'ok': True, 'uniqueID': unique_id})

@app.route('/api/settings/default_camera')
def api_settings_default_camera():
    s = _settings_read()
    return jsonify({'name': s.get('defaultCameraName')})

@app.route('/api/settings/camera_slots')
def api_settings_camera_slots():
    s = _settings_read()
    slots = s.get('defaultCameraSlots') or {}
    return jsonify({k: slots.get(k) or None for k in ('TL', 'TR', 'BL', 'BR')})

@app.route('/api/settings/photo', methods=['GET', 'POST'])
def api_settings_photo():
    s = _settings_read()
    # Base defaults — all keys the Swift PhotoDefaults struct can write.
    defaults = {
        'focal': '35.0', 'sensorW': '36.0', 'baseline': '0.1',
        'plateW': '0.10', 'plateH': '0.10',
        'plateColorR': 128.0, 'plateColorG': 0.0, 'plateColorB': 255.0,
        'plateColorTol': 25.0, 'underwater': False,
        'expectedPlateCount': 8, 'defaultAIModel': '',
    }
    photo = {**defaults, **s.get('photo', {})}
    if request.method == 'GET':
        return jsonify(photo)
    data = request.get_json(force=True)
    merged = {**photo, **{k: v for k, v in data.items() if k in defaults}}
    _write_settings_key('photo', merged)
    return jsonify({'ok': True})

@app.route('/api/camera/distortion', methods=['POST'])
def api_camera_distortion():
    global _distortion_on
    data = request.get_json(force=True)
    _distortion_on = bool(data.get('enabled', False))
    return jsonify({'ok': True, 'enabled': _distortion_on})

def _gen_feed(channel: str):
    """MJPEG generator. channel='full' → whole frame; 'CH01'-'CH04' → quad quadrant."""
    boundary = b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
    while True:
        frame, _ = _get_frame()
        if frame is None:
            time.sleep(0.05)
            continue
        if channel != 'full':
            quads = _split_quad(frame)
            frame = quads.get(channel)
            if frame is None:
                time.sleep(0.05)
                continue
        if _distortion_on:
            frame = _apply_undistort(frame)
        ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            time.sleep(0.05)
            continue
        yield boundary + buf.tobytes() + b'\r\n'
        time.sleep(0.033)

@app.route('/api/camera/feed/<channel>')
def api_camera_feed(channel):
    if channel not in CHANNELS and channel != 'full':
        return jsonify({'error': 'invalid channel'}), 400
    return Response(_gen_feed(channel), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/camera/screenshot/<channel>')
def api_camera_screenshot(channel):
    """Return a single JPEG frame for the given channel ('full' or 'CH01'–'CH04')."""
    if channel not in CHANNELS and channel != 'full':
        return jsonify({'error': 'invalid channel'}), 400
    frame, _ = _get_frame()
    if frame is None:
        return jsonify({'error': 'No camera frame available'}), 503
    if channel != 'full':
        quads = _split_quad(frame)
        frame = quads.get(channel)
        if frame is None:
            return jsonify({'error': 'Channel not available'}), 503
    if _distortion_on:
        frame = _apply_undistort(frame)
    ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
    if not ok:
        return jsonify({'error': 'Encode failed'}), 500
    return Response(buf.tobytes(), mimetype='image/jpeg')

# ── Undistort model API (reads/writes settings.json via _UNDISTORT_DIR) ───────
# The source of truth for the model list is settings.json (managed by Swift).
# These routes let the web UI read that list and change the active selection,
# which writes back to settings.json so the Swift UI stays in sync.

def _read_undistort_models():
    """Read undistort model list from settings.json."""
    s = _settings_read()
    return s.get('undistortModels', []), s.get('activeUndistortId')

def _write_settings_key(key: str, value):
    """Patch a single key in settings.json atomically."""
    s = _settings_read()
    s[key] = value
    tmp = _SETTINGS_JSON + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(s, f, indent=2)
    os.replace(tmp, _SETTINGS_JSON)

@app.route('/api/undistort/reload', methods=['POST'])
def api_undistort_reload():
    """Re-read settings.json and reload the active undistort model.
    Called by Electron whenever the Swift settings binary writes settings-updated."""
    global _undistort_raw, _undistort_maps, _undistort_id, _distortion_on
    s = _settings_read()
    active_id = s.get('activeUndistortId')
    if active_id:
        pkl_path = os.path.join(_UNDISTORT_DIR, active_id + '.pkl')
        if os.path.exists(pkl_path):
            try:
                _load_undistort_pkl(pkl_path, active_id)
                return jsonify({'ok': True, 'active': _undistort_id})
            except Exception as e:
                return jsonify({'ok': False, 'error': str(e)})
    # No active model — clear
    _clear_undistort()
    if not active_id:
        _distortion_on = False
    return jsonify({'ok': True, 'active': None})

@app.route('/api/undistort/list')
def api_undistort_list():
    models, active = _read_undistort_models()
    return jsonify({'models': models, 'active': _undistort_id or active})

@app.route('/api/undistort/select', methods=['POST'])
def api_undistort_select():
    data = request.get_json(force=True)
    file_id = data.get('id')
    if not file_id:
        _clear_undistort()
        _write_settings_key('activeUndistortId', None)
        return jsonify({'ok': True, 'active': None})
    dest = os.path.join(_UNDISTORT_DIR, file_id + '.pkl')
    if not os.path.exists(dest):
        return jsonify({'error': 'Not found'}), 404
    try:
        _load_undistort_pkl(dest, file_id)
        _write_settings_key('activeUndistortId', file_id)
    except Exception as e:
        return jsonify({'error': str(e)}), 500
    return jsonify({'ok': True, 'active': _undistort_id})

# ── Task 1.2 photogrammetry helpers ──────────────────────────────────────────
#
# Pipeline overview:
#   1. Caller hits /api/task1_2/analyze with two images, plate-color HSV,
#      mode ("stereo" for now; ai modes route via :5002 in step 9), and
#      optional overrides.
#   2. We resolve the active calibration files from settings.json (left
#      pkl id, right pkl id, active stereo extrinsics yaml id).
#   3. We convert each pkl to YAML on demand (cached on disk so we don't
#      repeat work). The C++ binary reads only YAML.
#   4. We invoke scripts/task1_2/stereo_distance with --mode stereo and the
#      resolved paths. The binary emits ONE JSON document on stdout.
#   5. We parse it, attach base64 debug images from --debug-dir, return.
#
# Nothing is persisted between runs (per spec). All temp dirs cleaned up.

_PKL_YAML_CACHE_DIR = os.path.join(_APP_SUPPORT_DIR, 'undistort_yaml')
os.makedirs(_PKL_YAML_CACHE_DIR, exist_ok=True)
_STEREO_EXTRINSICS_DIR = os.path.join(_APP_SUPPORT_DIR, 'stereo_extrinsics')
os.makedirs(_STEREO_EXTRINSICS_DIR, exist_ok=True)

_TASK1_2_PYTHON = os.path.join(
    PROJECT_ROOT, 'app', 'Resources', 'python-runtime', 'bin', 'python3')
_PKL_TO_YAML = os.path.join(
    PROJECT_ROOT, 'scripts', 'task1_2', 'python', 'pkl_to_yaml.py')


def _pkl_to_yaml_cached(pkl_id: str, image_w: int = 0, image_h: int = 0) -> str:
    """Convert ~/Library/.../undistort/<pkl_id>.pkl to a YAML the C++ binary
    can read. Cached on disk; regenerates if the pkl is newer than the YAML
    or the image dimensions changed."""
    pkl_path  = os.path.join(_UNDISTORT_DIR, pkl_id + '.pkl')
    if not os.path.exists(pkl_path):
        raise FileNotFoundError(f'pkl not found: {pkl_path}')
    # Cache key includes the requested dimensions because pkl_to_yaml.py
    # writes them into the YAML and the rectifier reads them.
    yaml_name = f'{pkl_id}_{image_w}x{image_h}.yaml' if (image_w and image_h) \
                else f'{pkl_id}.yaml'
    yaml_path = os.path.join(_PKL_YAML_CACHE_DIR, yaml_name)
    if (os.path.exists(yaml_path)
            and os.path.getmtime(yaml_path) >= os.path.getmtime(pkl_path)):
        return yaml_path
    cmd = [_TASK1_2_PYTHON, _PKL_TO_YAML, pkl_path, yaml_path]
    if image_w: cmd += ['--width',  str(image_w)]
    if image_h: cmd += ['--height', str(image_h)]
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
    if proc.returncode != 0:
        raise RuntimeError(
            f'pkl_to_yaml.py failed: {proc.stderr.strip() or proc.stdout.strip()}')
    return yaml_path


def _resolve_calibration_paths():
    """Read settings.json and return the absolute paths of the configured
    left calibration pkl, right calibration pkl, and active stereo extrinsics
    yaml. Any missing entry is returned as None — the caller decides whether
    to error out or proceed."""
    s = _settings_read()
    left_id   = s.get('photogrammetryLeftCalibId')
    right_id  = s.get('photogrammetryRightCalibId')
    stereo_id = s.get('activeStereoExtrinsicsId')
    paths = {
        'left_pkl':  os.path.join(_UNDISTORT_DIR, left_id  + '.pkl') if left_id  else None,
        'right_pkl': os.path.join(_UNDISTORT_DIR, right_id + '.pkl') if right_id else None,
        'stereo_yaml': os.path.join(_STEREO_EXTRINSICS_DIR, stereo_id + '.yaml')
                       if stereo_id else None,
        'left_id':  left_id,
        'right_id': right_id,
        'stereo_id': stereo_id,
    }
    for k in ('left_pkl', 'right_pkl', 'stereo_yaml'):
        p = paths[k]
        if p and not os.path.exists(p):
            paths[k] = None
    return paths


def _file_to_data_url(path):
    """Read a local image file and return a base64 data URL."""
    ext = os.path.splitext(path)[1].lstrip('.').lower()
    mime = 'image/png' if ext == 'png' else 'image/jpeg'
    with open(path, 'rb') as f:
        return f'data:{mime};base64,{base64.b64encode(f.read()).decode()}'


# NOTE: `_scale_scene_in_place` was removed during the May 2026 overhaul.
# Manual scale override is now served by the new task1_2 blueprint at
# POST /api/task1_2/manual_width (see web/task1_2.py).


@app.route('/api/task1_2/calibrations')
def api_task1_2_calibrations():
    """Report the calibration files available + which roles are assigned.
    The frontend uses this to render its calibration QC banner."""
    s = _settings_read()
    pkls = s.get('undistortModels', []) or []
    extrinsics = s.get('stereoExtrinsicsFiles', []) or []
    return jsonify({
        'pkls': pkls,
        'stereo_extrinsics': extrinsics,
        'photogrammetryLeftCalibId':   s.get('photogrammetryLeftCalibId'),
        'photogrammetryRightCalibId':  s.get('photogrammetryRightCalibId'),
        'activeStereoExtrinsicsId':    s.get('activeStereoExtrinsicsId'),
    })


@app.route('/api/task1_2/ai_providers')
def api_task1_2_ai_providers():
    """Proxy to the Electron main process's :5002 endpoint."""
    import urllib.request
    try:
        with urllib.request.urlopen(
                'http://127.0.0.1:5002/ai_providers', timeout=3) as r:
            return Response(r.read(), mimetype='application/json')
    except Exception as e:
        return jsonify({'openai': False, 'anthropic': False, 'google': False,
                        'appleIntelligence': False,
                        'error': str(e)}), 200


@app.route('/api/task1_2/enhance', methods=['POST'])
def api_task1_2_enhance():
    import urllib.request, urllib.error, json as _json, sys
    data = request.get_json(force=True, silent=True) or {}
    provider = data.get('provider', '')
    model_id  = data.get('model', '')
    if not provider or not model_id:
        return jsonify({'error': 'provider and model required'}), 400

    current_model = data.get('model_json') or {}
    images_b64    = data.get('images', [])

    # Compact current section sizes for the prompt
    sections = current_model.get('sections', [])
    cur_sizes = [[round(s['size'][0],3), round(s['size'][2],3)] for s in sections if s.get('size')]
    cur_json = _json.dumps(cur_sizes, separators=(',', ':'))

    # ── Shared prompt text ─────────────────────────────────────────────────────
    prompt = (
        "You are a photogrammetry measurement expert. Analyze the stereo camera images of a "
        "PVC pipe coral-garden structure and correct the 3D reconstruction measurements.\n\n"
        "The structure has 3 sections: left, middle, right. Each section has a fixed depth of "
        "0.36 m. You must estimate the WIDTH (horizontal span) and HEIGHT (vertical span) of "
        "each section in metres by examining the images carefully.\n\n"
        "Current algorithm estimates (may be wrong): " + cur_json + "\n"
        "Format: [[left_width, left_height], [mid_width, mid_height], [right_width, right_height]]\n\n"
        "Study the images. The PVC pipes are white/grey. Use the 10 cm square coloured plates "
        "visible in the images as scale references — each plate is exactly 0.10 m × 0.10 m.\n\n"
        "You MUST output corrected values. Do not return the same numbers unless you are certain "
        "they are correct. Be precise to 3 decimal places.\n\n"
        "Respond with ONLY a raw JSON array — no markdown, no text, no explanation:\n"
        "[[Lx,Hz],[Lx,Hz],[Lx,Hz]]"
    )

    # ── Build provider body ────────────────────────────────────────────────────
    def parse_image(img):
        url = img.get('data_url', '')
        if not url.startswith('data:') or ';base64,' not in url:
            return None
        head, b64 = url.split(';base64,', 1)
        return head.replace('data:', ''), b64

    if provider == 'anthropic':
        content = []
        for img in images_b64[:6]:
            p = parse_image(img)
            if p:
                content.append({'type': 'image', 'source': {
                    'type': 'base64', 'media_type': p[0], 'data': p[1]}})
        content.append({'type': 'text', 'text': prompt})
        body = {'model': model_id, 'max_tokens': 1024,
                'messages': [{'role': 'user', 'content': content}]}

    elif provider == 'google':
        parts = []
        for img in images_b64[:6]:
            p = parse_image(img)
            if p:
                parts.append({'inline_data': {'mime_type': p[0], 'data': p[1]}})
        parts.append({'text': prompt})
        body = {'contents': [{'role': 'user', 'parts': parts}],
                'generationConfig': {'maxOutputTokens': 1024}}

    elif provider == 'apple':
        # Pass prompt + images as a JSON payload; Swift side uses Vision to describe images.
        apple_imgs = []
        for img in images_b64[:4]:
            p = parse_image(img)
            if p:
                apple_imgs.append({'mime': p[0], 'b64': p[1]})
        body = {'messages': [{'role': 'user', 'content': _json.dumps(
            {'prompt': prompt, 'images': apple_imgs}, separators=(',', ':')
        )}]}

    else:
        # OpenAI
        content = []
        for img in images_b64[:6]:
            p = parse_image(img)
            if p:
                content.append({'type': 'image_url',
                                 'image_url': {'url': img['data_url']}})
        content.append({'type': 'text', 'text': prompt})
        body = {'model': model_id, 'max_tokens': 1024,
                'messages': [{'role': 'user', 'content': content}]}

    # ── Call :5002 ─────────────────────────────────────────────────────────────
    payload = _json.dumps({'provider': provider, 'model': model_id, 'body': body}).encode()
    print(f'[enhance] {provider}/{model_id} payload={len(payload)}b', file=sys.stderr, flush=True)
    try:
        req = urllib.request.Request(
            'http://127.0.0.1:5002/ai_call', data=payload,
            headers={'Content-Type': 'application/json'}, method='POST')
        try:
            with urllib.request.urlopen(req, timeout=120) as r:
                raw = r.read()
        except urllib.error.HTTPError as he:
            raw = he.read()
            print(f'[enhance] HTTPError {he.code} body: {raw[:500]}', file=sys.stderr, flush=True)
        print(f'[enhance] response first 300: {raw[:300]}', file=sys.stderr, flush=True)
        ai_resp = _json.loads(raw)
    except Exception as e:
        return jsonify({'error': f'AI call failed: {str(e)}'}), 502

    # Surface provider-level errors (error key present, no response key)
    if isinstance(ai_resp, dict) and 'error' in ai_resp and not any(
            k in ai_resp for k in ('content', 'choices', 'candidates', 'text')):
        err = ai_resp['error']
        if isinstance(err, dict):
            err = err.get('message') or err.get('status') or str(err)
        return jsonify({'error': f'AI error: {err}'}), 502

    # ── Extract text ───────────────────────────────────────────────────────────
    try:
        if provider == 'anthropic':
            text = ai_resp['content'][0]['text']
        elif provider == 'google':
            text = ai_resp['candidates'][0]['content']['parts'][0]['text']
        elif provider == 'apple':
            text = ai_resp['text']
        else:
            c = ai_resp['choices'][0]['message']['content']
            text = c if isinstance(c, str) else c[0].get('text', '')
    except (KeyError, IndexError, TypeError) as e:
        return jsonify({'error': f'Unexpected response shape: {str(e)}',
                        'raw': ai_resp}), 502

    # ── Parse the [[Lx,Hz],[Lx,Hz],[Lx,Hz]] array the model returned ─────────
    clean = text.strip()
    if clean.startswith('```'):
        clean = clean.split('\n', 1)[1].rsplit('```', 1)[0].strip()
    # Grab the first JSON array if model prepended text
    if not clean.startswith('['):
        m = re.search(r'\[.*\]', clean, re.DOTALL)
        if m:
            clean = m.group(0)
    try:
        sizes = _json.loads(clean)
        if not isinstance(sizes, list) or len(sizes) != 3:
            raise ValueError(f'expected 3-element array, got {type(sizes).__name__} len={len(sizes) if isinstance(sizes, list) else "?"}')
        for pair in sizes:
            if not isinstance(pair, list) or len(pair) != 2:
                raise ValueError(f'each element must be [Lx, Hz], got {pair!r}')
    except Exception as e:
        return jsonify({'error': f'Model returned unexpected format: {str(e)}',
                        'raw_text': text}), 502

    return jsonify({'sizes': sizes})


# In-memory store of recently-completed runs so /scale_override can rescale
# without re-running C++. Keyed by a short token returned with /analyze. TTL
# is implicit: clobbered on next analyze, never persisted.
_recent_runs = {}
_recent_runs_lock = threading.Lock()


def _store_recent_run(scene: dict) -> str:
    import secrets
    token = secrets.token_urlsafe(12)
    with _recent_runs_lock:
        # Keep only the last few runs in memory to bound footprint.
        if len(_recent_runs) > 8:
            _recent_runs.clear()
        _recent_runs[token] = scene
    return token


def _get_recent_run(token: str):
    with _recent_runs_lock:
        return _recent_runs.get(token)


# ─── REMOVED in May 2026 overhaul ──────────────────────────────────────
# The legacy in-app routes
#     POST /api/task1_2/analyze          (used field names left_image/right_image)
#     POST /api/task1_2/scale_override
# have been deleted from this file. Both URLs are now served by the new
# `task1_2_bp` blueprint (see web/task1_2.py), which expects the modern
# multipart field names `lefts[]` / `rights[]` and exposes
# /api/task1_2/manual_width as the manual-scale entry point.
# Helpers above (_resolve_calibration_paths, _pkl_to_yaml_cached,
# _file_to_data_url, _store_recent_run, _get_recent_run) are kept because
# task1_2_modes.py (video / hybrid_ai / ai_only modes) still uses them.



@app.route('/')
def index():
    return render_template('index.html')

@app.route('/t/1_2')
def t1_2():
    return render_template('1_2.html')

@app.route('/t/2_2')
def t2_2():
    return render_template('2_2.html')

@app.route('/t/2_1')
def t2_1():
    return render_template('2_1.html')

@app.route('/t/2_5')
def t2_5():
    return render_template('2_5.html')

@app.route('/t/tasks')
def t_tasks():
    return render_template('tasks.html')

# ── Task file API ──────────────────────────────────────────────────────────────

_TASKS_DIR = os.path.join(_APP_SUPPORT_DIR, 'tasks')
_TASKS_JSON = os.path.join(PROJECT_ROOT, 'tasks.json')
os.makedirs(_TASKS_DIR, exist_ok=True)

def _read_tasks_json():
    try:
        with open(_TASKS_JSON) as f:
            return json.load(f)
    except Exception:
        return {'tasks': [], 'order': {}}

def _parse_m26tl(content):
    """Plain list of task IDs, one per line. Comments (#) and blank lines ignored."""
    order = []
    for raw_line in content.splitlines():
        line = raw_line.strip()
        if line and not line.startswith('#'):
            order.append(line)
    return order

# ── In-process session state (survives window close/reopen within same app run) ──
_task_session = {
    'fileId': None,
    'secIdx': 0,
    'taskIdx': 0,
    'states': {},
    'timerSeconds': 15 * 60,
}
_task_session_lock = threading.Lock()

@app.route('/api/tasks/all')
def api_tasks_all():
    task_data = _read_tasks_json()
    return jsonify({'tasks': task_data.get('tasks', [])})

@app.route('/api/tasks/active')
def api_tasks_active():
    s = _settings_read()
    active_id = s.get('activeTaskFileId')
    save_on_relaunch = bool(s.get('saveTasksOnRelaunch', False))
    if not active_id:
        return jsonify({'error': 'no active file'}), 404
    m26tl_path = os.path.join(_TASKS_DIR, active_id + '.m26tl')
    if not os.path.exists(m26tl_path):
        return jsonify({'error': 'file not found'}), 404
    try:
        with open(m26tl_path) as f:
            content = f.read()
    except Exception as e:
        return jsonify({'error': str(e)}), 500
    demo_order = _parse_m26tl(content)
    task_data = _read_tasks_json()
    tasks_by_id = {t['id']: t for t in task_data.get('tasks', [])}
    d_order = task_data.get('order', {}).get('D', [])
    p_order = task_data.get('order', {}).get('P', [])
    # All tasks combined for lookup
    all_tasks = list(task_data.get('tasks', []))
    with _task_session_lock:
        session = dict(_task_session)
    return jsonify({
        'fileId': active_id,
        'dOrder': d_order,
        'pOrder': p_order,
        'demoOrder': demo_order,
        'allTasks': all_tasks,
        'saveOnRelaunch': save_on_relaunch,
        'session': session,
    })

@app.route('/api/tasks/session', methods=['POST'])
def api_tasks_session():
    """Save session state server-side so it survives window close/reopen."""
    data = request.get_json(force=True)
    with _task_session_lock:
        for key in ('fileId', 'secIdx', 'taskIdx', 'states', 'timerSeconds'):
            if key in data:
                _task_session[key] = data[key]
    # Always write to disk — load at startup only when saveTasksOnRelaunch is on.
    # saveTasksOnRelaunch is owned by main.js (not Swift) to prevent clobbering.
    try:
        sess_path = os.path.join(_APP_SUPPORT_DIR, 'task_session.json')
        with open(sess_path, 'w') as fh:
            json.dump(dict(_task_session), fh)
    except Exception:
        pass
    return jsonify({'ok': True})

def _load_task_session_from_disk():
    """Startup: load session from disk only if saveTasksOnRelaunch is on."""
    try:
        s = _settings_read()
        if not bool(s.get('saveTasksOnRelaunch', False)):
            return
        sess_path = os.path.join(_APP_SUPPORT_DIR, 'task_session.json')
        with open(sess_path) as fh:
            saved = json.load(fh)
        with _task_session_lock:
            _task_session.update(saved)
    except Exception:
        pass

_load_task_session_from_disk()

@app.route('/api/tasks/reset', methods=['POST'])
def api_tasks_reset():
    with _task_session_lock:
        _task_session['secIdx']       = 0
        _task_session['taskIdx']      = 0
        _task_session['states']       = {}
        _task_session['timerSeconds'] = 15 * 60
    try:
        sess_path = os.path.join(_APP_SUPPORT_DIR, 'task_session.json')
        os.remove(sess_path)
    except Exception:
        pass
    return jsonify({'ok': True})

@app.route('/api/tasks/list')
def api_tasks_list():
    s = _settings_read()
    return jsonify({'files': s.get('taskFiles', []), 'active': s.get('activeTaskFileId')})

@app.route('/api/tasks/upload', methods=['POST'])
def api_tasks_upload():
    if 'file' not in request.files:
        return jsonify({'error': 'no file'}), 400
    f = request.files['file']
    if not f.filename.endswith('.m26tl'):
        return jsonify({'error': 'must be .m26tl'}), 400
    import uuid
    file_id = str(uuid.uuid4())
    dest = os.path.join(_TASKS_DIR, file_id + '.m26tl')
    f.save(dest)
    name = f.filename[:-6]
    s = _settings_read()
    files = s.get('taskFiles', [])
    files.append({'id': file_id, 'name': name})
    s['taskFiles'] = files
    _write_settings_key('taskFiles', files)
    return jsonify({'ok': True, 'id': file_id, 'name': name})

@app.route('/api/tasks/rename', methods=['POST'])
def api_tasks_rename():
    data = request.get_json(force=True)
    file_id = data.get('id')
    name = (data.get('name') or '').strip()
    if not file_id or not name:
        return jsonify({'error': 'missing id or name'}), 400
    s = _settings_read()
    files = s.get('taskFiles', [])
    for entry in files:
        if entry['id'] == file_id:
            entry['name'] = name
            break
    _write_settings_key('taskFiles', files)
    return jsonify({'ok': True})

@app.route('/api/tasks/select', methods=['POST'])
def api_tasks_select():
    data = request.get_json(force=True)
    file_id = data.get('id')
    if file_id and not os.path.exists(os.path.join(_TASKS_DIR, file_id + '.m26tl')):
        return jsonify({'error': 'not found'}), 404
    _write_settings_key('activeTaskFileId', file_id)
    return jsonify({'ok': True, 'active': file_id})

@app.route('/api/tasks/delete', methods=['POST'])
def api_tasks_delete():
    data = request.get_json(force=True)
    file_id = data.get('id')
    if not file_id:
        return jsonify({'error': 'missing id'}), 400
    try:
        os.remove(os.path.join(_TASKS_DIR, file_id + '.m26tl'))
    except Exception:
        pass
    s = _settings_read()
    files = [f for f in s.get('taskFiles', []) if f['id'] != file_id]
    _write_settings_key('taskFiles', files)
    if s.get('activeTaskFileId') == file_id:
        _write_settings_key('activeTaskFileId', None)
    return jsonify({'ok': True})

# ── Model listing ──────────────────────────────────────────────────────────────
 
MODELS_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'models')
 
# Naming scheme: M26.CD.{a}{b}.{c}.{x}__{d}.{e}.{f}_{g}.{h}
# a = type letter (P=Prototype, F=Production)
# b = model number
# c = dataset size
# x = epochs
# d = month, e = day, f = year, g = hour, h = minute
MODEL_PATTERN = re.compile(
    r'^M26\.CD\.([PF])(\d+)\.(\d+)\.(\d+)__(\d+)\.(\d+)\.(\d+)_(\d+)\.(\d+)$'
)
 
TYPE_LABELS = {'P': 'Prototype', 'F': 'Production'}
 
def parse_model_dir(name):
    m = MODEL_PATTERN.match(name)
    if not m:
        return None
    type_letter, model_num, dataset_size, epochs, month, day, year, hour, minute = m.groups()
    try:
        dt = (int(year), int(month), int(day), int(hour), int(minute))
        label = f"{TYPE_LABELS.get(type_letter, type_letter)} {model_num}  ({dataset_size} imgs)"
        return {'name': name, 'label': label, 'dt': dt}
    except ValueError:
        return None
 
@app.route('/api/crab_models')
def crab_models():
    models = []
    if os.path.isdir(MODELS_DIR):
        for f in os.listdir(MODELS_DIR):
            if not f.endswith('.pt'):
                continue
            name = f[:-3]  # strip .pt
            parsed = parse_model_dir(name)
            if parsed is None:
                continue
            parsed['path'] = os.path.join(MODELS_DIR, f)
            models.append(parsed)

    models.sort(key=lambda x: x['dt'], reverse=True)
    return jsonify({'models': [{'label': m['label'], 'path': m['path']} for m in models]})
 
 
# ── Inference ──────────────────────────────────────────────────────────────────
 
@app.route('/api/crab_detect', methods=['POST'])
def crab_detect():
    from ultralytics import YOLO
 
    if 'image' not in request.files:
        return jsonify({'error': 'No image provided'}), 400
 
    image_file = request.files['image']
    model_path = request.form.get('model_path', '').strip()
    conf       = float(request.form.get('conf', 0.25))
    iou        = float(request.form.get('iou', 0.45))
 
    if not model_path or not os.path.exists(model_path):
        return jsonify({'error': f'Model file not found: {model_path}'}), 400
 
    suffix = os.path.splitext(image_file.filename)[1] or '.jpg'
    with tempfile.NamedTemporaryFile(delete=False, suffix=suffix) as tmp:
        image_file.save(tmp.name)
        tmp_path = tmp.name
 
    try:
        model = YOLO(model_path)
 
        t0      = time.time()
        results = model.predict(source=tmp_path, conf=conf, iou=iou, verbose=False)
        inf_ms  = round((time.time() - t0) * 1000)
 
        result  = results[0]
        img_h, img_w = result.orig_shape
 
        detections = []
        for box in result.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            detections.append({
                'class': model.names[int(box.cls)],
                'conf':  round(float(box.conf), 4),
                'box':   [round(x1), round(y1), round(x2), round(y2)],
            })
        detections.sort(key=lambda d: d['conf'], reverse=True)
 
        raw = '\n'.join(
            f"{d['class']:20s}  conf={d['conf']:.3f}  box={d['box']}"
            for d in detections
        ) or 'No detections.'
 
        return jsonify({
            'detections':   detections,
            'image_width':  img_w,
            'image_height': img_h,
            'inference_ms': inf_ms,
            'raw_output':   raw,
        })
 
    except Exception as e:
        return jsonify({'error': str(e)}), 500
 
    finally:
        try:
            os.unlink(tmp_path)
        except Exception:
            pass
 

# Old /api/analyze and the in-app /api/task1_2/analyze + /scale_override
# have been removed. /api/task1_2/* is now served by the task1_2_bp
# blueprint registered just below.



_init_from_settings()
_start_reader()
if not os.environ.get('ELECTRON_IS_PACKAGED'):
    subprocess.run(['make', '-C', TASK1_2_DIR, 'all'], capture_output=True)

from task1_2 import task1_2_bp
app.register_blueprint(task1_2_bp)

if __name__ == '__main__':
    import signal, socket as _socket
    from werkzeug.serving import make_server
    server = make_server('0.0.0.0', 5001, app, threaded=True)
    server.socket.setsockopt(_socket.SOL_SOCKET, _socket.SO_REUSEADDR, 1)
    def _shutdown(signum, frame):
        # Shut down in a thread so the signal handler returns immediately.
        # Closing the server socket unblocks serve_forever's select() loop.
        t = threading.Thread(target=server.shutdown, daemon=True)
        t.start()
        try: server.socket.close()
        except Exception: pass
    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT,  _shutdown)
    print('MATE_SERVER_READY', flush=True)
    server.serve_forever()
