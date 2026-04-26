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

_cam_list_cache      = []   # last known [{index, name, builtin}]
_cam_list_cache_key  = ''   # hash of sp output to detect changes
_cam_list_lock       = threading.Lock()

def list_cameras():
    """Return [{index, name, builtin}] — fast native macOS enumeration.
    Uses system_profiler for quick presence check; Swift for index ordering.
    Result is cached and only recomputed when the device list changes."""
    global _cam_list_cache, _cam_list_cache_key

    try:
        sp = subprocess.run(
            ['system_profiler', 'SPCameraDataType', '-json'],
            capture_output=True, text=True, timeout=5
        )
        sp_key = sp.stdout[:512]  # cheap change-detection fingerprint
    except Exception:
        with _cam_list_lock:
            return list(_cam_list_cache)

    with _cam_list_lock:
        if sp_key == _cam_list_cache_key and _cam_list_cache:
            return list(_cam_list_cache)

    # Device list changed — re-enumerate order via AVFoundation Swift snippet
    cameras = []
    try:
        swift_src = (
            "import AVFoundation\n"
            "let s = AVCaptureDevice.DiscoverySession("
            "deviceTypes: [.builtInWideAngleCamera, .external],"
            "mediaType: .video, position: .unspecified)\n"
            "for (i, d) in s.devices.enumerated() {\n"
            "    let b = d.deviceType == .builtInWideAngleCamera ? 1 : 0\n"
            '    print("\\(i)\\t\\(b)\\t\\(d.localizedName)")\n'
            "}"
        )
        with tempfile.NamedTemporaryFile(suffix='.swift', delete=False, mode='w') as tf:
            tf.write(swift_src)
            tf_path = tf.name
        result = subprocess.run(
            ['swift', tf_path],
            capture_output=True, text=True, timeout=15
        )
        os.unlink(tf_path)
        for line in result.stdout.splitlines():
            parts = line.split('\t', 2)
            if len(parts) == 3:
                cameras.append({
                    'index':   int(parts[0]),
                    'builtin': parts[1] == '1',
                    'name':    parts[2].strip(),
                })
    except Exception:
        pass

    with _cam_list_lock:
        _cam_list_cache_key = sp_key
        _cam_list_cache     = cameras
    return list(cameras)

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

def _open_camera(index: int):
    global _cam_cap, _cam_index, _cam_seq, _latest_frame, _frame_seq
    with _cam_lock:
        if _cam_cap is not None:
            _cam_cap.release()
            _cam_cap = None
            _cam_index = None
        cap = cv2.VideoCapture(index)
        if cap.isOpened():
            _cam_cap   = cap
            _cam_index = index
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
    index = data.get('index')
    if index is None:
        _close_camera()
        return jsonify({'ok': True, 'index': None})
    _open_camera(int(index))
    return jsonify({'ok': True, 'index': _cam_index})

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
    defaults = {'focal': '35.0', 'sensorW': '36.0', 'baseline': '0.1', 'plateW': '0.3', 'plateH': '0.2'}
    photo = {**defaults, **s.get('photo', {})}
    if request.method == 'GET':
        return jsonify(photo)
    data = request.get_json(force=True)
    merged = {**photo, **{k: str(v) for k, v in data.items() if k in defaults}}
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

def parse_plate_output(text):
    result = {
        'raw_output': text,
        'mode': 'plates',
        'config': {},
        'detections': [],
        'stereo': {},
        'plate_distances': [],
    }

    focal_m = re.search(r'Focal length:\s+([\d.]+)\s+mm', text)
    if focal_m:
        result['config']['focal_length_mm'] = float(focal_m.group(1))

    baseline_m = re.search(r'Baseline:\s+([\d.]+)\s+m', text)
    if baseline_m:
        result['config']['baseline_m'] = float(baseline_m.group(1))

    for m in re.finditer(r'Plate #(\d+):\s*\n\s*Centroid:\s*\(([\d.]+),\s*([\d.]+)\)\s*\n\s*Bounding Box:\s*\[([^\]]+)\]\s*\n\s*Rotated Rect:\s*([\d.]+)\s*x\s*([\d.]+)\s*@\s*([\d.]+)\s*deg\s*\n\s*Area:\s*([\d.]+)\s*px\^2\s*\n\s*Confidence:\s*([\d.]+)%', text):
        result['detections'].append({
            'id': int(m.group(1)),
            'centroid': [float(m.group(2)), float(m.group(3))],
            'confidence': float(m.group(9))
        })

    rot_m = re.search(r'Rotation angle:\s+([\d.]+)\s+degrees', text)
    if rot_m:
        result['stereo']['rotation_angle'] = float(rot_m.group(1))

    reproj_m = re.search(r'Reprojection error:\s+([\d.]+)\s+px', text)
    if reproj_m:
        result['stereo']['reprojection_error'] = float(reproj_m.group(1))

    for m in re.finditer(r'Plate #(\d+):\s+([\d.]+)\s+in\s+\(([\d.]+)\s+m\)', text):
        result['plate_distances'].append({
            'plate_id': int(m.group(1)),
            'distance_inches': float(m.group(2)),
            'distance_meters': float(m.group(3))
        })

    for m in re.finditer(r'Plate #(\d+):\s+UNABLE TO DETERMINE', text):
        result['plate_distances'].append({
            'plate_id': int(m.group(1)),
            'distance_inches': None,
            'distance_meters': None
        })

    return result

def parse_pipe_output(text):
    result = {
        'raw_output': text,
        'mode': 'pipes',
        'scale': {},
        'ref_squares': [],
        'pipe_lengths': [],
    }

    ppc_m = re.search(r'Pixels per cm:\s+([\d.]+)', text)
    if ppc_m:
        result['scale']['pixels_per_cm'] = float(ppc_m.group(1))

    cpp_m = re.search(r'Cm per pixel:\s+([\d.]+)', text)
    if cpp_m:
        result['scale']['cm_per_pixel'] = float(cpp_m.group(1))

    refs_m = re.search(r'References used:\s+(\d+)', text)
    if refs_m:
        result['scale']['references_used'] = int(refs_m.group(1))

    conf_m = re.search(r'Scale confidence:\s+(\d+)%', text)
    if conf_m:
        result['scale']['confidence'] = int(conf_m.group(1))

    for m in re.finditer(r'Square #(\d+):\s+(\w+)\s+\|\s+side=(\d+)\s+px\s+\|\s+conf=(\d+)%', text):
        result['ref_squares'].append({
            'id': int(m.group(1)),
            'color': m.group(2),
            'side_px': int(m.group(3)),
            'confidence': int(m.group(4))
        })

    for m in re.finditer(r'Pipe #(\d+):\s+([\d.]+)\s+cm\s+\(([\d.]+)\s+in\)', text):
        if int(m.group(1)) not in [p['pipe_id'] for p in result['pipe_lengths']]:
            result['pipe_lengths'].append({
                'pipe_id': int(m.group(1)),
                'length_cm': float(m.group(2)),
                'length_inches': float(m.group(3))
            })

    for m in re.finditer(r'Pipe #(\d+):\s+(\d+)\s+px\s*$', text, re.MULTILINE):
        if int(m.group(1)) not in [p['pipe_id'] for p in result['pipe_lengths']]:
            result['pipe_lengths'].append({
                'pipe_id': int(m.group(1)),
                'length_px': int(m.group(2)),
                'length_cm': None,
                'length_inches': None
            })

    return result


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
 

@app.route('/api/analyze', methods=['POST'])
def analyze():
    mode = request.form.get('mode', 'plates')

    def file_to_data_url(path):
        ext = os.path.splitext(path)[1].lstrip('.').lower()
        mime = 'image/png' if ext == 'png' else 'image/jpeg'
        with open(path, 'rb') as f:
            return f'data:{mime};base64,{base64.b64encode(f.read()).decode()}'

    if mode == 'pipes':
        if 'image' not in request.files:
            return jsonify({'error': 'Please upload an image'}), 400

        img_file = request.files['image']
        if not img_file.filename:
            return jsonify({'error': 'Please select an image file'}), 400

        if not allowed_file(img_file.filename):
            return jsonify({'error': 'Unsupported file format. Use JPG, PNG, BMP, or TIFF.'}), 400

        img_ext = img_file.filename.rsplit('.', 1)[1].lower()

        tmp_img = tempfile.NamedTemporaryFile(delete=False, suffix=f'.{img_ext}')
        tmp_out = tempfile.mkdtemp()
        try:
            img_file.save(tmp_img.name)
            tmp_img.close()

            ref_side = request.form.get('ref_square_side', '10.0')
            cmd = [
                BINARY_PATH,
                '--mode', 'pipes',
                tmp_img.name,
                '--ref-square-side', str(ref_side),
                '--save-debug',
                '--output', tmp_out
            ]

            proc = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
            output_text = proc.stdout + proc.stderr
            parsed = parse_pipe_output(output_text)

            debug_images = {}
            for f in sorted(os.listdir(tmp_out)):
                if f.endswith(('.png', '.jpg')):
                    debug_images[f] = file_to_data_url(os.path.join(tmp_out, f))
            parsed['debug_images'] = debug_images

            parsed['uploaded_images'] = {
                'image': file_to_data_url(tmp_img.name)
            }

            return jsonify(parsed)

        except subprocess.TimeoutExpired:
            return jsonify({'error': 'Analysis timed out.'}), 500
        except Exception as e:
            return jsonify({'error': f'Analysis failed: {str(e)}'}), 500
        finally:
            try: os.unlink(tmp_img.name)
            except Exception: pass
            try:
                import shutil; shutil.rmtree(tmp_out, ignore_errors=True)
            except Exception: pass

    else:
        if 'left_image' not in request.files or 'right_image' not in request.files:
            return jsonify({'error': 'Please upload both left and right images'}), 400

        left_file = request.files['left_image']
        right_file = request.files['right_image']

        if not left_file.filename or not right_file.filename:
            return jsonify({'error': 'Please select files for both images'}), 400

        if not allowed_file(left_file.filename) or not allowed_file(right_file.filename):
            return jsonify({'error': 'Unsupported file format. Use JPG, PNG, BMP, or TIFF.'}), 400

        left_ext = left_file.filename.rsplit('.', 1)[1].lower()
        right_ext = right_file.filename.rsplit('.', 1)[1].lower()

        tmp_left = tempfile.NamedTemporaryFile(delete=False, suffix=f'.{left_ext}')
        tmp_right = tempfile.NamedTemporaryFile(delete=False, suffix=f'.{right_ext}')
        tmp_out = tempfile.mkdtemp()
        try:
            left_file.save(tmp_left.name)
            right_file.save(tmp_right.name)
            tmp_left.close()
            tmp_right.close()

            focal = request.form.get('focal_length', '35.0')
            sensor = request.form.get('sensor_width', '36.0')
            baseline = request.form.get('baseline', '0.1')
            plate_w = request.form.get('plate_width', '0.3')
            plate_h = request.form.get('plate_height', '0.2')

            cmd = [
                BINARY_PATH,
                tmp_left.name, tmp_right.name,
                '--focal', str(focal),
                '--sensor', str(sensor),
                '--baseline', str(baseline),
                '--plate-width', str(plate_w),
                '--plate-height', str(plate_h),
                '--save-debug',
                '--output', tmp_out
            ]

            proc = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
            output_text = proc.stdout + proc.stderr

            parsed = parse_plate_output(output_text)

            debug_images = {}
            for f in sorted(os.listdir(tmp_out)):
                if f.endswith(('.png', '.jpg')):
                    debug_images[f] = file_to_data_url(os.path.join(tmp_out, f))
            parsed['debug_images'] = debug_images

            parsed['uploaded_images'] = {
                'left': file_to_data_url(tmp_left.name),
                'right': file_to_data_url(tmp_right.name),
            }

            return jsonify(parsed)

        except subprocess.TimeoutExpired:
            return jsonify({'error': 'Analysis timed out. Images may be too large.'}), 500
        except Exception as e:
            return jsonify({'error': f'Analysis failed: {str(e)}'}), 500
        finally:
            try: os.unlink(tmp_left.name)
            except Exception: pass
            try: os.unlink(tmp_right.name)
            except Exception: pass
            try:
                import shutil; shutil.rmtree(tmp_out, ignore_errors=True)
            except Exception: pass


_init_from_settings()
_start_reader()
subprocess.run(['make', '-C', TASK1_2_DIR, 'all'], capture_output=True)

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
