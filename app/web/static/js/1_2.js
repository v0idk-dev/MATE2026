const leftInput = document.getElementById('left-image');
const rightInput = document.getElementById('right-image');
const leftPreview = document.getElementById('left-preview');
const rightPreview = document.getElementById('right-preview');
const analyzeBtn = document.getElementById('analyze-btn');

function setupUpload(input, preview, cardId) {
    const card = document.getElementById(cardId);
    input.addEventListener('change', function() {
        if (this.files && this.files[0]) {
            const reader = new FileReader();
            reader.onload = function(e) {
                preview.src = e.target.result;
                card.classList.add('has-image');
                checkReady();
            };
            reader.readAsDataURL(this.files[0]);
        }
    });

    preview.addEventListener('click', function() {
        input.click();
    });
}

setupUpload(leftInput, leftPreview, 'left-upload');
setupUpload(rightInput, rightPreview, 'right-upload');

function checkReady() {
    const hasLeft = leftInput.files && leftInput.files.length > 0;
    const hasRight = rightInput.files && rightInput.files.length > 0;
    analyzeBtn.disabled = !(hasLeft && hasRight);
}

function toggleRaw() {
    const el = document.getElementById('raw-output');
    const toggle = document.getElementById('raw-toggle');
    if (el.style.display === 'none') {
        el.style.display = 'block';
        toggle.textContent = '\u2212';
    } else {
        el.style.display = 'none';
        toggle.textContent = '+';
    }
}

document.querySelectorAll('.tab').forEach(tab => {
    tab.addEventListener('click', function() {
        document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
        document.querySelectorAll('.tab-panel').forEach(p => p.classList.remove('active'));
        this.classList.add('active');
        document.getElementById('panel-' + this.dataset.tab).classList.add('active');
    });
});

async function runAnalysis() {
    const formData = new FormData();
    formData.append('mode', 'plates');

    formData.append('left_image', leftInput.files[0]);
    formData.append('right_image', rightInput.files[0]);
    formData.append('focal_length', document.getElementById('focal-length').value);
    formData.append('sensor_width', document.getElementById('sensor-width').value);
    formData.append('baseline', document.getElementById('baseline').value);
    formData.append('plate_width', document.getElementById('plate-width').value);
    formData.append('plate_height', document.getElementById('plate-height').value);

    document.getElementById('loading-section').style.display = 'block';
    document.getElementById('results-section').style.display = 'none';
    document.getElementById('error-section').style.display = 'none';
    analyzeBtn.disabled = true;

    const loadingSub = document.getElementById('loading-sub-text');
    loadingSub.textContent = 'Detecting purple plates, matching features, computing distance';

    try {
        const response = await fetch('/api/analyze', {
            method: 'POST',
            body: formData
        });

        const data = await response.json();

        document.getElementById('loading-section').style.display = 'none';
        analyzeBtn.disabled = false;

        if (!response.ok || data.error) {
            showError(data.error || 'Analysis failed');
            return;
        }

        showPlateResults(data);

    } catch (err) {
        document.getElementById('loading-section').style.display = 'none';
        analyzeBtn.disabled = false;
        showError('Network error. Please try again.');
    }
}

function showError(msg) {
    document.getElementById('error-section').style.display = 'block';
    document.getElementById('error-message').textContent = msg;
}

let _annotatedDataUrl = null;
let _annotatedFilename = 'annotated.png';

function downloadAnnotated() {
    if (!_annotatedDataUrl) return;
    const a = document.createElement('a');
    a.href = _annotatedDataUrl;
    a.download = _annotatedFilename;
    a.click();
}

function setAnnotatedResult(dataUrl, filename) {
    _annotatedDataUrl = dataUrl;
    _annotatedFilename = filename;
    const btn = document.getElementById('annotated-dl-btn');
    const hint = document.getElementById('annotated-dl-hint');
    if (dataUrl) {
        btn.style.display = '';
        hint.textContent = filename;
    } else {
        btn.style.display = 'none';
        hint.textContent = '';
    }
}

function showPlateResults(data) {
    document.getElementById('results-section').style.display = 'block';
    document.getElementById('plate-results').style.display = 'block';
    document.getElementById('pipe-results').style.display = 'none';
    document.getElementById('results-title').textContent = 'Annotated Distance Results';

    const annotatedContainer = document.getElementById('annotated-image-container');
    annotatedContainer.innerHTML = '';
    const annotatedEntry = data.debug_images && Object.entries(data.debug_images).find(([k]) => k.includes('annotated'));
    const annotatedUrl = annotatedEntry ? annotatedEntry[1] : null;
    if (annotatedUrl) {
        annotatedContainer.innerHTML = `<img src="${annotatedUrl}" alt="Annotated results" class="annotated-img">`;
        setAnnotatedResult(annotatedUrl, annotatedEntry[0]);
    } else {
        annotatedContainer.innerHTML = '<p style="color:var(--text-muted);padding:20px;">No annotated image available</p>';
        setAnnotatedResult(null, '');
    }

    const plateList = document.getElementById('plate-distances-list');
    plateList.innerHTML = '';
    if (data.plate_distances && data.plate_distances.length > 0) {
        data.plate_distances.forEach(pd => {
            const distStr = pd.distance_inches !== null ? pd.distance_inches.toFixed(1) + ' in' : 'N/A';
            const mStr = pd.distance_meters !== null ? pd.distance_meters.toFixed(4) + ' m' : '';
            plateList.innerHTML += `
                <div class="plate-distance-card">
                    <div class="plate-id">Plate #${pd.plate_id}</div>
                    <div class="plate-dist-value">${distStr}</div>
                    <div class="plate-dist-secondary">${mStr}</div>
                </div>`;
        });
    } else {
        plateList.innerHTML = '<p style="color:var(--text-muted);">No plate distances computed</p>';
    }

    const stereoStats = document.getElementById('stereo-stats');
    stereoStats.innerHTML = '';
    const stereoData = [
        ['Rotation Angle', data.stereo?.rotation_angle ? data.stereo.rotation_angle.toFixed(2) + '\u00b0' : 'N/A'],
        ['Reprojection Error', data.stereo?.reprojection_error ? data.stereo.reprojection_error.toFixed(4) + ' px' : 'N/A'],
        ['Plates Matched', data.plate_distances ? data.plate_distances.length : 0],
    ];
    stereoData.forEach(([label, value]) => {
        stereoStats.innerHTML += `
            <div class="stat-row">
                <span class="stat-label">${label}</span>
                <span class="stat-value">${value}</span>
            </div>`;
    });

    setupDebugTabs(data, 'plates');

    document.getElementById('raw-output').textContent = data.raw_output || 'No output available';
    document.getElementById('results-section').scrollIntoView({ behavior: 'smooth', block: 'start' });
}

function showPipeResults(data) {
    document.getElementById('results-section').style.display = 'block';
    document.getElementById('plate-results').style.display = 'none';
    document.getElementById('pipe-results').style.display = 'block';
    document.getElementById('results-title').textContent = 'PVC Pipe Length Results';

    const annotatedContainer = document.getElementById('annotated-image-container');
    annotatedContainer.innerHTML = '';
    const annotatedEntry = data.debug_images && Object.entries(data.debug_images).find(([k]) => k.includes('annotated'));
    const annotatedUrl = annotatedEntry ? annotatedEntry[1] : null;
    if (annotatedUrl) {
        annotatedContainer.innerHTML = `<img src="${annotatedUrl}" alt="Pipe measurement results" class="annotated-img">`;
        setAnnotatedResult(annotatedUrl, annotatedEntry[0]);
    } else {
        annotatedContainer.innerHTML = '<p style="color:var(--text-muted);padding:20px;">No annotated image available</p>';
        setAnnotatedResult(null, '');
    }

    const pipeList = document.getElementById('pipe-lengths-list');
    pipeList.innerHTML = '';
    if (data.pipe_lengths && data.pipe_lengths.length > 0) {
        data.pipe_lengths.forEach(pl => {
            const lenStr = pl.length_cm !== null ? pl.length_cm.toFixed(1) + ' cm' : pl.length_px + ' px';
            const secStr = pl.length_inches !== null ? pl.length_inches.toFixed(1) + ' in' : '';
            pipeList.innerHTML += `
                <div class="plate-distance-card">
                    <div class="plate-id">Pipe #${pl.pipe_id}</div>
                    <div class="plate-dist-value">${lenStr}</div>
                    <div class="plate-dist-secondary">${secStr}</div>
                </div>`;
        });
    } else {
        pipeList.innerHTML = '<p style="color:var(--text-muted);">No pipe lengths detected</p>';
    }

    const scaleStats = document.getElementById('scale-stats');
    scaleStats.innerHTML = '';
    const scaleData = [
        ['Pixels per cm', data.scale?.pixels_per_cm ? data.scale.pixels_per_cm.toFixed(2) : 'N/A'],
        ['References Used', data.scale?.references_used ?? 'N/A'],
        ['Scale Confidence', data.scale?.confidence ? data.scale.confidence + '%' : 'N/A'],
        ['Pipes Found', data.pipe_lengths ? data.pipe_lengths.length : 0],
    ];
    scaleData.forEach(([label, value]) => {
        scaleStats.innerHTML += `
            <div class="stat-row">
                <span class="stat-label">${label}</span>
                <span class="stat-value">${value}</span>
            </div>`;
    });

    setupDebugTabs(data, 'pipes');

    document.getElementById('raw-output').textContent = data.raw_output || 'No output available';
    document.getElementById('results-section').scrollIntoView({ behavior: 'smooth', block: 'start' });
}

function makeDownloadBtn(dataUrl, filename) {
    const wrap = document.createElement('div');
    wrap.className = 'cd-download-bar';
    wrap.style.marginTop = '10px';

    const btn = document.createElement('button');
    btn.className = 'cd-download-btn';
    btn.innerHTML = `<svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5"><path d="M12 3v13"/><path d="M7 11l5 5 5-5"/><path d="M4 20h16"/></svg> Download`;
    btn.onclick = () => {
        const a = document.createElement('a');
        a.href = dataUrl;
        a.download = filename;
        a.click();
    };

    const hint = document.createElement('span');
    hint.className = 'cd-download-hint';
    hint.textContent = filename;

    if (filename === 'pair_1_left_detection.png') wrap.style.marginBottom = '14px';

    wrap.appendChild(btn);
    wrap.appendChild(hint);
    return wrap;
}

function setupDebugTabs(data, mode) {
    const uploadedPanel = document.getElementById('panel-uploaded');
    const detectionsPanel = document.getElementById('panel-detections');
    const matchesPanel = document.getElementById('panel-matches');
    const epipolarPanel = document.getElementById('panel-epipolar');

    uploadedPanel.innerHTML = '';
    detectionsPanel.innerHTML = '';
    matchesPanel.innerHTML = '';
    epipolarPanel.innerHTML = '';

    if (mode === 'plates' && data.uploaded_images) {
        uploadedPanel.innerHTML = `
            <div class="image-pair">
                <img src="${data.uploaded_images.left}" alt="Left camera">
                <img src="${data.uploaded_images.right}" alt="Right camera">
            </div>`;
    } else if (mode === 'pipes' && data.uploaded_images?.image) {
        uploadedPanel.innerHTML = `<img src="${data.uploaded_images.image}" alt="Input image" style="width:100%;border-radius:8px;">`;
    }

    if (data.debug_images) {
        Object.entries(data.debug_images).forEach(([fname, url]) => {
            const img = `<img src="${url}" alt="${fname}">`;
            const btn = makeDownloadBtn(url, fname);
            if (fname.includes('detection') || fname.includes('references')) {
                detectionsPanel.insertAdjacentHTML('beforeend', img);
                detectionsPanel.appendChild(btn);
            } else if (fname.includes('matches') || fname.includes('edges')) {
                matchesPanel.insertAdjacentHTML('beforeend', img);
                matchesPanel.appendChild(btn);
            } else if (fname.includes('epipolar')) {
                epipolarPanel.insertAdjacentHTML('beforeend', img);
                epipolarPanel.appendChild(btn);
            }
        });
    }

    if (!detectionsPanel.querySelector('img')) detectionsPanel.innerHTML = '<p style="color:var(--text-muted);padding:20px;">No detection images</p>';
    if (!matchesPanel.querySelector('img')) matchesPanel.innerHTML = '<p style="color:var(--text-muted);padding:20px;">No match/edge images</p>';
    if (!epipolarPanel.querySelector('img')) epipolarPanel.innerHTML = '<p style="color:var(--text-muted);padding:20px;">No epipolar images</p>';
}

// ── Screenshot from camera ────────────────────────────────────────────────────
async function screenshotChannel(channel) {
    const res = await fetch(`/api/camera/screenshot/${channel}`);
    if (!res.ok) throw new Error(`No frame on ${channel}`);
    const blob = await res.blob();
    return new File([blob], `screenshot_${channel}.jpg`, { type: 'image/jpeg' });
}

function loadImageFile(file, inputEl, previewEl, cardId) {
    const dt = new DataTransfer();
    dt.items.add(file);
    inputEl.files = dt.files;
    const reader = new FileReader();
    reader.onload = e => {
        previewEl.src = e.target.result;
        document.getElementById(cardId).classList.add('has-image');
        checkReady();
    };
    reader.readAsDataURL(file);
}

async function takeScreenshots() {
    let leftCh = 'CH03', rightCh = 'CH04';
    try {
        const r = await fetch('/api/cameras_config');
        if (r.ok) {
            const cfg = await r.json();
            leftCh  = cfg.screenshots?.photogrammetryLeft?.channel  || leftCh;
            rightCh = cfg.screenshots?.photogrammetryRight?.channel || rightCh;
        }
    } catch (_) {}
    try {
        const [lFile, rFile] = await Promise.all([
            screenshotChannel(leftCh),
            screenshotChannel(rightCh),
        ]);
        loadImageFile(lFile, leftInput,  leftPreview,  'left-upload');
        loadImageFile(rFile, rightInput, rightPreview, 'right-upload');
    } catch (e) {
        alert('Screenshot failed: ' + e.message);
    }
}

// ── Photo defaults from settings ──────────────────────────────────────────────
async function loadPhotoDefaults() {
    try {
        const r = await fetch('/api/settings/photo');
        if (!r.ok) return;
        const d = await r.json();
        if (d.focal)    document.getElementById('focal-length').value  = d.focal;
        if (d.sensorW)  document.getElementById('sensor-width').value  = d.sensorW;
        if (d.baseline) document.getElementById('baseline').value      = d.baseline;
        if (d.plateW)   document.getElementById('plate-width').value   = d.plateW;
        if (d.plateH)   document.getElementById('plate-height').value  = d.plateH;
    } catch {}
}

function savePhotoDefaults() {
    const data = {
        focal:    document.getElementById('focal-length').value,
        sensorW:  document.getElementById('sensor-width').value,
        baseline: document.getElementById('baseline').value,
        plateW:   document.getElementById('plate-width').value,
        plateH:   document.getElementById('plate-height').value,
    };
    fetch('/api/settings/photo', { method: 'POST', headers: {'Content-Type':'application/json'}, body: JSON.stringify(data) })
        .catch(() => {});
}

['focal-length', 'sensor-width', 'baseline', 'plate-width', 'plate-height'].forEach(id => {
    const el = document.getElementById(id);
    if (el) el.addEventListener('change', savePhotoDefaults);
});

loadPhotoDefaults();

if (window.electronAPI?.onSettingsPhotoChanged) {
    window.electronAPI.onSettingsPhotoChanged(() => loadPhotoDefaults());
}
