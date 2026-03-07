import os
import uuid
import subprocess
import re
import shutil
from flask import Flask, request, jsonify, render_template, send_from_directory

app = Flask(__name__,
            template_folder='templates',
            static_folder='static')

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
UPLOAD_DIR = os.path.join(PROJECT_ROOT, 'outputs', 'uploads')
RESULTS_DIR = os.path.join(PROJECT_ROOT, 'outputs', 'task1_2')
BINARY_PATH = os.path.join(PROJECT_ROOT, 'scripts', 'task1_2', 'stereo_distance')
TASK1_2_DIR = os.path.join(PROJECT_ROOT, 'scripts', 'task1_2')

os.makedirs(UPLOAD_DIR, exist_ok=True)
os.makedirs(RESULTS_DIR, exist_ok=True)

ALLOWED_EXTENSIONS = {'png', 'jpg', 'jpeg', 'bmp', 'tiff', 'tif'}

def allowed_file(filename):
    return '.' in filename and filename.rsplit('.', 1)[1].lower() in ALLOWED_EXTENSIONS

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


@app.route('/api/analyze', methods=['POST'])
def analyze():
    mode = request.form.get('mode', 'plates')

    if mode == 'pipes':
        if 'image' not in request.files:
            return jsonify({'error': 'Please upload an image'}), 400

        img_file = request.files['image']
        if not img_file.filename:
            return jsonify({'error': 'Please select an image file'}), 400

        if not allowed_file(img_file.filename):
            return jsonify({'error': 'Unsupported file format. Use JPG, PNG, BMP, or TIFF.'}), 400

        job_id = str(uuid.uuid4())[:8]
        job_dir = os.path.join(UPLOAD_DIR, job_id)
        output_dir = os.path.join(RESULTS_DIR, job_id)
        os.makedirs(job_dir, exist_ok=True)
        os.makedirs(output_dir, exist_ok=True)

        img_ext = img_file.filename.rsplit('.', 1)[1].lower()
        img_path = os.path.join(job_dir, f'image.{img_ext}')
        img_file.save(img_path)

        ref_side = request.form.get('ref_square_side', '10.0')

        cmd = [
            BINARY_PATH,
            '--mode', 'pipes',
            img_path,
            '--ref-square-side', str(ref_side),
            '--save-debug',
            '--output', output_dir
        ]

        try:
            proc = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
            output_text = proc.stdout + proc.stderr
            parsed = parse_pipe_output(output_text)
            parsed['job_id'] = job_id

            debug_files = []
            if os.path.isdir(output_dir):
                for f in sorted(os.listdir(output_dir)):
                    if f.endswith(('.png', '.jpg')):
                        debug_files.append(f'/api/results/{job_id}/{f}')
            parsed['debug_images'] = debug_files

            parsed['uploaded_images'] = {
                'image': f'/api/uploads/{job_id}/image.{img_ext}'
            }

            return jsonify(parsed)

        except subprocess.TimeoutExpired:
            return jsonify({'error': 'Analysis timed out.'}), 500
        except Exception as e:
            return jsonify({'error': f'Analysis failed: {str(e)}'}), 500

    else:
        if 'left_image' not in request.files or 'right_image' not in request.files:
            return jsonify({'error': 'Please upload both left and right images'}), 400

        left_file = request.files['left_image']
        right_file = request.files['right_image']

        if not left_file.filename or not right_file.filename:
            return jsonify({'error': 'Please select files for both images'}), 400

        if not allowed_file(left_file.filename) or not allowed_file(right_file.filename):
            return jsonify({'error': 'Unsupported file format. Use JPG, PNG, BMP, or TIFF.'}), 400

        job_id = str(uuid.uuid4())[:8]
        job_dir = os.path.join(UPLOAD_DIR, job_id)
        output_dir = os.path.join(RESULTS_DIR, job_id)
        os.makedirs(job_dir, exist_ok=True)
        os.makedirs(output_dir, exist_ok=True)

        left_ext = left_file.filename.rsplit('.', 1)[1].lower()
        right_ext = right_file.filename.rsplit('.', 1)[1].lower()
        left_path = os.path.join(job_dir, f'left.{left_ext}')
        right_path = os.path.join(job_dir, f'right.{right_ext}')

        left_file.save(left_path)
        right_file.save(right_path)

        focal = request.form.get('focal_length', '35.0')
        sensor = request.form.get('sensor_width', '36.0')
        baseline = request.form.get('baseline', '0.1')
        plate_w = request.form.get('plate_width', '0.3')
        plate_h = request.form.get('plate_height', '0.2')

        cmd = [
            BINARY_PATH,
            left_path, right_path,
            '--focal', str(focal),
            '--sensor', str(sensor),
            '--baseline', str(baseline),
            '--plate-width', str(plate_w),
            '--plate-height', str(plate_h),
            '--save-debug',
            '--output', output_dir
        ]

        try:
            proc = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
            output_text = proc.stdout + proc.stderr

            parsed = parse_plate_output(output_text)
            parsed['job_id'] = job_id

            debug_files = []
            if os.path.isdir(output_dir):
                for f in sorted(os.listdir(output_dir)):
                    if f.endswith(('.png', '.jpg')):
                        debug_files.append(f'/api/results/{job_id}/{f}')
            parsed['debug_images'] = debug_files

            parsed['uploaded_images'] = {
                'left': f'/api/uploads/{job_id}/left.{left_ext}',
                'right': f'/api/uploads/{job_id}/right.{right_ext}'
            }

            return jsonify(parsed)

        except subprocess.TimeoutExpired:
            return jsonify({'error': 'Analysis timed out. Images may be too large.'}), 500
        except Exception as e:
            return jsonify({'error': f'Analysis failed: {str(e)}'}), 500


@app.route('/api/results/<job_id>/<filename>')
def serve_result(job_id, filename):
    job_dir = os.path.join(RESULTS_DIR, job_id)
    return send_from_directory(job_dir, filename)


@app.route('/api/uploads/<job_id>/<filename>')
def serve_upload(job_id, filename):
    job_dir = os.path.join(UPLOAD_DIR, job_id)
    return send_from_directory(job_dir, filename)


if __name__ == '__main__':
    subprocess.run(['make', '-C', TASK1_2_DIR, 'all'], capture_output=True)
    app.run(host='0.0.0.0', port=5001, debug=False)
