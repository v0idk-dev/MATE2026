// Two-iframe crossfade setup
const taskButtons = document.querySelectorAll('.task-btn');
const taskContent = document.getElementById('task-content');

const frameA = document.getElementById('task-frame');
frameA.style.cssText = 'position:absolute;top:0;left:0;width:100%;height:100%;border:none;opacity:1;transition:opacity 0.25s ease;';
frameA.setAttribute('scrolling', 'yes');

const frameB = document.createElement('iframe');
frameB.style.cssText = 'position:absolute;top:0;left:0;width:100%;height:100%;border:none;opacity:0;transition:opacity 0.25s ease;';
frameB.setAttribute('scrolling', 'yes');
taskContent.appendChild(frameB);

let active = frameA;
let inactive = frameB;

const wip = document.createElement('div');
wip.className = 'wip-placeholder';
wip.style.cssText = 'position:absolute;top:0;left:0;right:0;bottom:0;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:16px;color:var(--text-muted);background:var(--surface);opacity:0;transition:opacity 0.25s ease;pointer-events:none;';
wip.innerHTML = `
    <svg width="64" height="64" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5">
        <path d="M12 2L2 7l10 5 10-5-10-5z"/>
        <path d="M2 17l10 5 10-5"/>
        <path d="M2 12l10 5 10-5"/>
    </svg>
    <span>Work In Progress</span>
`;
taskContent.appendChild(wip);

let currentTask = '1_2';

taskButtons.forEach(button => {
    button.addEventListener('click', () => {
        const taskId = button.dataset.task;

        if (button.classList.contains('active')) return;

        taskButtons.forEach(btn => btn.classList.remove('active'));
        button.classList.add('active');

        if (taskId === '2_1') {
            // Crossfade current frame out, WIP in
            active.style.opacity = '0';
            wip.style.opacity = '1';
            wip.style.pointerEvents = 'auto';
        } else if (currentTask === '2_1') {
            // Crossfade WIP out, new frame in
            inactive.src = `/t/${taskId}`;
            inactive.style.opacity = '1';
            wip.style.opacity = '0';
            wip.style.pointerEvents = 'none';
            [active, inactive] = [inactive, active];
        } else {
            // Crossfade frame to frame
            inactive.src = `/t/${taskId}`;
            inactive.style.opacity = '1';
            active.style.opacity = '0';
            [active, inactive] = [inactive, active];
        }

        currentTask = taskId;
    });
});

// Camera selector (placeholder - no actual switching implemented)
const cameraSelect = document.getElementById('camera-select');
cameraSelect.addEventListener('change', (e) => {
    console.log('Camera switched to:', e.target.value);
    // TODO: Implement actual camera switching when camera feeds are connected
});