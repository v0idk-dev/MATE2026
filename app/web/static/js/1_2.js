const leftInput = document.getElementById('left-image');
const rightInput = document.getElementById('right-image');
const leftPreview = document.getElementById('left-preview');
const rightPreview = document.getElementById('right-preview');
const pipeInput = document.getElementById('pipe-image');
const pipePreview = document.getElementById('pipe-preview');
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
setupUpload(pipeInput, pipePreview, 'pipe-upload');

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

function showPlateResults(data) {
    document.getElementById('results-section').style.display = 'block';
    document.getElementById('plate-results').style.display = 'block';
    document.getElementById('pipe-results').style.display = 'none';
    document.getElementById('results-title').textContent = 'Annotated Distance Results';

    const annotatedContainer = document.getElementById('annotated-image-container');
    annotatedContainer.innerHTML = '';
    const annotatedUrl = data.debug_images?.find(u => u.includes('annotated'));
    if (annotatedUrl) {
        annotatedContainer.innerHTML = `<img src="${annotatedUrl}" alt="Annotated results" class="annotated-img">`;
    } else {
        annotatedContainer.innerHTML = '<p style="color:var(--text-muted);padding:20px;">No annotated image available</p>';
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
    const annotatedUrl = data.debug_images?.find(u => u.includes('annotated'));
    if (annotatedUrl) {
        annotatedContainer.innerHTML = `<img src="${annotatedUrl}" alt="Pipe measurement results" class="annotated-img">`;
    } else {
        annotatedContainer.innerHTML = '<p style="color:var(--text-muted);padding:20px;">No annotated image available</p>';
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
        data.debug_images.forEach(url => {
            if (url.includes('detection') || url.includes('references')) {
                detectionsPanel.innerHTML += `<img src="${url}" alt="Detection visualization">`;
            } else if (url.includes('matches') || url.includes('edges')) {
                matchesPanel.innerHTML += `<img src="${url}" alt="Feature analysis">`;
            } else if (url.includes('epipolar')) {
                epipolarPanel.innerHTML += `<img src="${url}" alt="Epipolar lines">`;
            }
        });
    }

    if (!detectionsPanel.innerHTML) detectionsPanel.innerHTML = '<p style="color:var(--text-muted);padding:20px;">No detection images</p>';
    if (!matchesPanel.innerHTML) matchesPanel.innerHTML = '<p style="color:var(--text-muted);padding:20px;">No match/edge images</p>';
    if (!epipolarPanel.innerHTML) epipolarPanel.innerHTML = '<p style="color:var(--text-muted);padding:20px;">No epipolar images</p>';
}
