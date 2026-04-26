/* ═══════════════════════════════════════════════════════════════
   MAIN.JS — MATE ROV Control Station v2
   ═══════════════════════════════════════════════════════════════ */

const _sp = new URLSearchParams(location.search);
const SIMULATE_CONNECTED = _sp.get('sc') === '1' ? 1 : 0;
const SIMULATE_OPMODE    = _sp.get('so') === '1' ? 1 : 0;
const SIMULATE_DATA      = _sp.get('sd') === '1' ? 1 : 0;

const state = {
  voltage:      12.73,
  ping:         201,
  depth:        3.0,
  heading:      12,
  wsConnected:  false,
  opModeState:  'idle',
  opModeTimer:  0,
  timerInterval: null,
  telemetry:    {},
};

function updateConnectionState(connected) {
  state.wsConnected = connected;
  const overlay = document.getElementById('disconnect-overlay');
  if (connected) { overlay.classList.add('hidden'); }
  else           { overlay.classList.remove('hidden'); }
  updateWSDot(connected ? 'connected' : 'disconnected', connected ? 'Connected' : 'Disconnected');
  setWidgetsDimmed(!connected);
}

function setWidgetsDimmed(dimmed) {
  document.querySelectorAll('.widget-card').forEach(el => el.classList.toggle('disconnected-dim', dimmed));
  if (dimmed) {
    document.getElementById('qdw-voltage-val').textContent = '—';
    document.getElementById('qdw-ping-val').textContent    = '—';
    document.getElementById('qdw-depth-val').textContent   = '—';
    document.getElementById('qdw-heading-val').textContent = '—';
    document.getElementById('bat-fill').style.width = '0%';
    for (let i = 1; i <= 4; i++) document.getElementById('pb'+i).className = 'ping-bar';
  }
}

let ws = null;

function connectWS() {
  if (SIMULATE_CONNECTED) {
    updateConnectionState(true);
    if (SIMULATE_OPMODE) startSimulatedOpMode();
    return;
  }
  updateWSDot('connecting', 'Connecting…');
  try {
    ws = new WebSocket('ws://localhost:5001/ws');
    ws.onopen  = () => updateConnectionState(true);
    ws.onclose = () => { updateConnectionState(false); setTimeout(connectWS, 3000); };
    ws.onerror = () => ws.close();
    ws.onmessage = (e) => { try { handleWSMessage(JSON.parse(e.data)); } catch (_) {} };
  } catch (_) {
    updateConnectionState(false);
    setTimeout(connectWS, 3000);
  }
}

function handleWSMessage(msg) {
  if (msg.type === 'telemetry') { state.telemetry = msg.data || {}; renderTelemetry(); }
  if (msg.type === 'status') {
    if (msg.voltage !== undefined) { state.voltage = msg.voltage; updateVoltage(); }
    if (msg.ping    !== undefined) { state.ping    = msg.ping;    updatePing(); }
    if (msg.depth   !== undefined) { state.depth   = msg.depth;   updateDepth(); }
    if (msg.heading !== undefined) { state.heading = msg.heading; updateHeading(); }
  }
  if (msg.type === 'opModes') {
    const sel = document.getElementById('opmode-select');
    sel.innerHTML = '<option value="">Select...</option>';
    (msg.list || []).forEach(n => { const o = document.createElement('option'); o.value = o.textContent = n; sel.appendChild(o); });
  }
  if (msg.type === 'opModeState') handleOpModeState(msg.state);
}

function sendWS(obj) { if (ws && ws.readyState === 1) ws.send(JSON.stringify(obj)); }

function updateWSDot(cls, label) {
  document.getElementById('ws-dot').className     = 'status-dot ' + cls;
  document.getElementById('ws-label').textContent = label;
}

function startSimulatedOpMode() {
  state.telemetry = {
    'Depth (m)': '3.2', 'Heading': '14°', 'Voltage': '12.41V',
    'Claw 1': 'OPEN', 'Claw 2': 'CLOSED', 'Drive Mode': 'WaterDrive',
    'Thruster FL': '72%', 'Thruster FR': '68%',
  };
  renderTelemetry();
  const sel = document.getElementById('opmode-select');
  if (sel.options.length <= 1) {
    ['WaterDrive','AutoSample','ManualControl'].forEach(n => {
      const o = document.createElement('option'); o.value = o.textContent = n; sel.appendChild(o);
    });
  }
}

function updateVoltage() {
  const v   = state.voltage;
  const pct = Math.min(1, Math.max(0, (v - 10) / 4.4));
  const col = pct > 0.5 ? 'var(--green)' : pct > 0.25 ? 'var(--yellow)' : 'var(--red)';
  const fill = document.getElementById('bat-fill');
  fill.style.width = (pct * 100) + '%'; fill.style.background = col;
  const el = document.getElementById('qdw-voltage-val');
  el.textContent = v.toFixed(2) + 'V'; el.style.color = pct > 0.5 ? 'var(--text)' : col;
}

function updatePing() {
  const p = state.ping;
  const bars = p <= 50 ? 4 : p <= 100 ? 3 : p <= 200 ? 2 : p <= 500 ? 1 : 0;
  const cls  = p <= 100 ? 'active' : p <= 300 ? 'warn' : 'bad';
  for (let i = 1; i <= 4; i++) document.getElementById('pb'+i).className = 'ping-bar' + (i <= bars ? ' '+cls : '');
  const el = document.getElementById('qdw-ping-val');
  el.textContent = p + 'ms'; el.style.color = p > 500 ? 'var(--red)' : p > 300 ? 'var(--yellow)' : 'var(--text)';
}

// ── Depth update ─────────────────────────────────────────────────────────────
const RULER_PX_PER_M   = 150;                        // pixels per meter (set by user)
const RULER_PX_PER_FT  = RULER_PX_PER_M / 3.28084;  // pixels per foot  ≈ 33.5px
const WIDGET_PX_PER_M  = 8;

function updateDepth() {
  const d   = state.depth;
  const dFt = d * 3.28084;

  document.getElementById('qdw-depth-val').textContent = d.toFixed(1) + 'm';

  const wStrip = document.getElementById('wc-depth-strip');
  if (wStrip) wStrip.style.transform = `translateY(${-d * WIDGET_PX_PER_M}px)`;

  const mInner  = document.getElementById('df-m-inner');
  const ftInner = document.getElementById('df-ft-inner');
  if (mInner)  mInner.style.transform  = `translateY(${-d   * RULER_PX_PER_M}px)`;
  if (ftInner) ftInner.style.transform = `translateY(${-dFt * RULER_PX_PER_FT}px)`;

  const mLabel  = document.getElementById('df-m-label');
  const ftLabel = document.getElementById('df-ft-label');
  if (mLabel)  mLabel.textContent  = d.toFixed(1) + 'm';
  if (ftLabel) {
    ftLabel.textContent = dFt.toFixed(1) + 'ft';
    // 2-digit ft values (>=10) need more bleed room
    ftLabel.style.right = dFt >= 10 ? '-29px' : '-22px';
  }
}

function updateHeading() {
  const h = state.heading;
  document.getElementById('qdw-heading-val').textContent = Math.round(h) + '°N';
  const needle = document.getElementById('compass-needle-qdw');
  if (needle) needle.setAttribute('transform', `rotate(${-h}, 9, 9)`);
  robotHeading = h;
}

// ── Ruler tick builder helpers ────────────────────────────────────────────────
// nearMultiple: true if `val` is within `tol` of being a multiple of `step`
function nearMultiple(val, step, tol) {
  const r = Math.abs(val % step);
  return r < tol || r > step - tol;
}

// ── Build visualizer ruler ticks ─────────────────────────────────────────────
// Each tick is a full-column-width div (left:0,right:0) so that labels inside
// it are positioned correctly relative to the column edges.
// The visible colored bar is a child <span class="df-bar"> with the actual width.
// Labels are siblings of the bar inside the same full-width container.
//
// Meters:  minorStep=0.5m, midStep=1m,  majorStep=5m
// Feet:    minorStep=1ft,  midStep=5ft, majorStep=10ft
function buildVizRuler(containerId, maxDepth, majorStep, midStep, minorStep, unit, pxPerUnit) {
  const el = document.getElementById(containerId);
  if (!el) return;

  const totalSteps = Math.round(maxDepth / minorStep);
  const tol = minorStep * 0.05;
  let html = '';

  for (let i = -totalSteps; i <= totalSteps; i++) {
    const d      = Math.round(i * minorStep * 1000) / 1000;
    const topPx  = d * pxPerUnit;
    const isMajor = nearMultiple(d, majorStep, tol);
    const isMid   = !isMajor && nearMultiple(d, midStep, tol);
    const cls     = isMajor ? 'major' : isMid ? 'mid' : 'minor';

    const showLabel = isMajor && Math.abs(d) > tol;
    let labelStyle = '';
    if (showLabel) {
      if (unit === 'ft') {
        // Left ruler (feet): 1-digit → right:5px, 2-digit (|d|>=10) → right:2px
        labelStyle = Math.abs(d) >= 10 ? ' style="right:2px"' : ' style="right:6px"';
      } else {
        // Right ruler (meters): negative or 2-digit (|d|>=10) → left:5px, else → left:9px
        labelStyle = (d < 0 || Math.abs(d) >= 10) ? ' style="left:5px"' : ' style="left:9px"';
      }
    }
    const labelHtml = showLabel
      ? `<span class="df-tick-label"${labelStyle}>${Number.isInteger(d) ? d : d.toFixed(1)}${unit}</span>`
      : '';

    html += `<div class="df-tick-row" style="top:${topPx}px"><span class="df-bar ${cls}"></span>${labelHtml}</div>`;
  }
  el.innerHTML = html;
}

// ── Build widget ruler ticks ─────────────────────────────────────────────────
function buildWidgetRuler(containerId, maxDepth, majorStep, midStep, minorStep) {
  const el = document.getElementById(containerId);
  if (!el) return;

  const totalSteps = Math.round(maxDepth / minorStep);
  const tol = minorStep * 0.05;
  let html = '';

  for (let i = -totalSteps; i <= totalSteps; i++) {
    const d      = Math.round(i * minorStep * 1000) / 1000;
    const topPx  = d * WIDGET_PX_PER_M;
    const isMajor = nearMultiple(d, majorStep, tol);
    const isMid   = !isMajor && nearMultiple(d, midStep, tol);
    const cls     = isMajor ? 'major' : isMid ? 'mid' : 'minor';
    html += `<div class="wc-tick ${cls}" style="top:${topPx}px"></div>`;
  }
  el.innerHTML = html;
}

// ═══════════════════════════════════════════════════════════════
//  2D ROBOT VISUALIZATION
// ═══════════════════════════════════════════════════════════════

let robotCanvas, robotCtx;
let drawLoopRunning = false;
let drawLoopPaused  = false;
let robotRotX = -0.25, robotRotY = 0.35;
let targetRotX = -0.25, targetRotY = 0.35;
const DEF_ROT_X = -0.25, DEF_ROT_Y = 0.35;
let isDragging = false, prevMouse = {x:0, y:0};
let isHovered  = false, autoResetTO = null;
let robotHeading = 12;
let robotHeadingDisplay = 12; // smoothly lerped for compass animation
let prevVoltageDisplay = 12.73; // smoothly lerped voltage for mini battery
let miniVoltPctDisplay = 0;      // smoothly lerped for canvas battery fill

function initRobot2D() {
  robotCanvas = document.getElementById('robot-canvas');
  if (!robotCanvas) return;
  robotCtx = robotCanvas.getContext('2d');
  const wrap = document.getElementById('robot-canvas-wrap');

  function resize() {
    const r = wrap.getBoundingClientRect();
    robotCanvas.width  = r.width  * devicePixelRatio;
    robotCanvas.height = r.height * devicePixelRatio;
    robotCanvas.style.width  = r.width  + 'px';
    robotCanvas.style.height = r.height + 'px';
    robotCtx.setTransform(devicePixelRatio, 0, 0, devicePixelRatio, 0, 0);
  }
  resize();
  new ResizeObserver(resize).observe(wrap);

  wrap.addEventListener('mousedown', e => { isDragging = true; prevMouse = {x: e.clientX, y: e.clientY}; clearAR(); });
  window.addEventListener('mousemove', e => {
    if (!isDragging) return;
    // Negate X delta so dragging right rotates right (not reversed)
    targetRotY -= (e.clientX - prevMouse.x) * 0.012;
    targetRotX -= (e.clientY - prevMouse.y) * 0.012;
    targetRotX  = Math.max(-1.1, Math.min(1.1, targetRotX));
    prevMouse   = {x: e.clientX, y: e.clientY};
  });
  window.addEventListener('mouseup', () => { if (isDragging) { isDragging = false; scheduleAR(); } });
  wrap.addEventListener('mouseenter', () => { isHovered = true;  clearAR(); });
  wrap.addEventListener('mouseleave', () => { isHovered = false; scheduleAR(); });

  drawLoopRunning = true;
  drawLoop();
}

function pauseDrawLoop() {
  drawLoopPaused = true;
}

function resumeDrawLoop() {
  drawLoopPaused = false;
  if (!drawLoopRunning) {
    drawLoopRunning = true;
    requestAnimationFrame(drawLoop);
  }
}

function scheduleAR() {
  clearAR();
  autoResetTO = setTimeout(() => { if (!isHovered && !isDragging) { targetRotX = DEF_ROT_X; targetRotY = DEF_ROT_Y; } }, 2500);
}
function clearAR() { if (autoResetTO) { clearTimeout(autoResetTO); autoResetTO = null; } }

function drawLoop() {
  if (drawLoopPaused) { drawLoopRunning = false; return; }
  requestAnimationFrame(drawLoop);
  robotRotX += (targetRotX - robotRotX) * 0.1;
  robotRotY += (targetRotY - robotRotY) * 0.1;

  // Smooth heading display — lerp by shortest arc
  let dh = ((robotHeading - robotHeadingDisplay + 540) % 360) - 180;
  robotHeadingDisplay += dh * 0.08;
  const targetVoltPct = Math.min(1, Math.max(0, (state.voltage - 10) / 4.4));
  miniVoltPctDisplay += (targetVoltPct - miniVoltPctDisplay) * 0.06;
  robotHeadingDisplay = ((robotHeadingDisplay % 360) + 360) % 360;

  drawRobot();
}

function project(x, y, z, cx, cy, scale) {
  const x1 = x * Math.cos(robotRotY) + z * Math.sin(robotRotY);
  const z1 = -x * Math.sin(robotRotY) + z * Math.cos(robotRotY);
  const y2 = y * Math.cos(robotRotX) - z1 * Math.sin(robotRotX);
  const z2 = y * Math.sin(robotRotX) + z1 * Math.cos(robotRotX);
  const fov = 5, dist = fov + z2;
  return { x: cx + (x1 / dist) * scale * fov, y: cy - (y2 / dist) * scale * fov, z: z2 };
}

function drawArrow(ctx, from, to, color, lineW) {
  const dx = to.x - from.x, dy = to.y - from.y;
  const len = Math.sqrt(dx*dx + dy*dy);
  if (len < 1) return;
  ctx.strokeStyle = color; ctx.lineWidth = lineW;
  ctx.beginPath(); ctx.moveTo(from.x, from.y); ctx.lineTo(to.x, to.y); ctx.stroke();
  // Arrowhead
  const ux = dx/len, uy = dy/len;
  const hs = Math.min(len * 0.35, 7);
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.moveTo(to.x, to.y);
  ctx.lineTo(to.x - ux*hs - uy*hs*0.5, to.y - uy*hs + ux*hs*0.5);
  ctx.lineTo(to.x - ux*hs + uy*hs*0.5, to.y - uy*hs - ux*hs*0.5);
  ctx.closePath(); ctx.fill();
}

function drawRobot() {
  if (!robotCtx || !robotCanvas) return;
  const W  = robotCanvas.width  / devicePixelRatio;
  const H  = robotCanvas.height / devicePixelRatio;

  // Reserve space: top overlay (~28px), bottom overlay (~28px)
  // Center robot+compass in the remaining space
  const topReserve = 28, botReserve = 28;
  const availH = H - topReserve - botReserve;
  const cx = W / 2;
  const cy = topReserve + availH / 2;
  const S  = Math.min(W, availH) * 0.24;
  const oFont = Math.max(11, S * 0.13); // shared font size for all overlays

  robotCtx.clearRect(0, 0, W, H);
  robotCtx.fillStyle = '#111827';
  robotCtx.fillRect(0, 0, W, H);

  // ── Compass rose (drawn first, behind robot) ──
  const compassR = Math.min(W, availH) * 0.37;
  drawCompassRose(cx, cy, compassR, oFont);

  // ── Robot cube: w=1.8, h=0.7, d=1.1 ──
  const hw = 0.9, hh = 0.35, hd = 0.55;
  // Verts: 0-3 = front (+Z), 4-7 = back (-Z)
  //   0=TL, 1=TR, 2=BR, 3=BL (T=top Y+, L=left X-)
  const verts = [
    [-hw,  hh,  hd], [ hw,  hh,  hd], [ hw, -hh,  hd], [-hw, -hh,  hd],
    [-hw,  hh, -hd], [ hw,  hh, -hd], [ hw, -hh, -hd], [-hw, -hh, -hd],
  ];
  const pv = verts.map(([x,y,z]) => project(x, y, z, cx, cy, S));

  // ── Robot cube — back-face culling (correct solid rendering for convex shapes) ──
  // For each face, compute the cross-product of two edges in screen space.
  // If the Z component of the cross product is positive, the face is front-facing; draw it.
  // This is always correct for convex objects and needs no sorting.
  function faceNormalZ(pts) {
    // Cross product of (p1-p0) x (p2-p0), take Z component
    const ax = pts[1].x - pts[0].x, ay = pts[1].y - pts[0].y;
    const bx = pts[2].x - pts[0].x, by = pts[2].y - pts[0].y;
    return ax * by - ay * bx; // positive = CCW in screen = facing viewer (canvas Y is down)
  }

  const faceList = [
    { idx: [0,1,2,3], fill: '#0d1a2e', stroke: '#00bfff', lw: 1.5, isFront: true }, // [0] +Z front
    { idx: [7,6,5,4], fill: '#0b1622', stroke: '#1e2d45', lw: 1   },                // [1] -Z back
    { idx: [3,7,4,0], fill: '#0c1828', stroke: '#1e2d45', lw: 1   },                // [2] -X left
    { idx: [1,5,6,2], fill: '#0c1828', stroke: '#1e2d45', lw: 1   },                // [3] +X right
    { idx: [0,4,5,1], fill: '#0e1c32', stroke: '#1e2d45', lw: 1   },                // [4] +Y top
    { idx: [2,6,7,3], fill: '#0a1520', stroke: '#1e2d45', lw: 1   },                // [5] -Y bottom
  ];

  // Pass 1: Determine which faces are visible (facing the camera)
  const visibleFaces = faceList.filter(({ idx }) => {
    const pts = idx.map(i => pv[i]);
    return faceNormalZ(pts) < 0; 
  });

  // Pass 2: Draw the background fills for all visible faces
  visibleFaces.forEach(({ idx, fill, isFront }) => {
    const pts = idx.map(i => pv[i]);
    robotCtx.beginPath();
    robotCtx.moveTo(pts[0].x, pts[0].y);
    pts.slice(1).forEach(p => robotCtx.lineTo(p.x, p.y));
    robotCtx.closePath();
    
    robotCtx.fillStyle = fill;
    robotCtx.fill();
    
    // Add the glowing highlight for the front face
    if (isFront) {
      robotCtx.fillStyle = 'rgba(0,191,255,0.06)';
      robotCtx.fill();
    }
  });

  // Pass 3: Draw the standard strokes for all non-front faces
  visibleFaces.forEach(({ idx, stroke, lw, isFront }) => {
    if (isFront) return; // We handle front edges separately below
    
    const pts = idx.map(i => pv[i]);
    robotCtx.beginPath();
    robotCtx.moveTo(pts[0].x, pts[0].y);
    pts.slice(1).forEach(p => robotCtx.lineTo(p.x, p.y));
    robotCtx.closePath();
    robotCtx.strokeStyle = stroke;
    robotCtx.lineWidth = lw;
    robotCtx.stroke();
  });

  // Pass 4: Draw the front edges on top to prevent their thickness from being covered
  // An edge is drawn if at least one of its adjacent faces is visible
  const frontEdges = [
    { edge: [0,1], faces: [faceList[0], faceList[4]] }, // Top
    { edge: [1,2], faces: [faceList[0], faceList[3]] }, // Right
    { edge: [2,3], faces: [faceList[0], faceList[5]] }, // Bottom
    { edge: [3,0], faces: [faceList[0], faceList[2]] }, // Left
  ];

  robotCtx.strokeStyle = '#00bfff'; 
  robotCtx.lineWidth = 1.5;
  
  frontEdges.forEach(({ edge, faces }) => {
    // Check if at least one connected face is in our visibleFaces array
    const isEdgeVisible = faces.some(f => visibleFaces.includes(f));
    if (isEdgeVisible) {
      robotCtx.beginPath();
      robotCtx.moveTo(pv[edge[0]].x, pv[edge[0]].y);
      robotCtx.lineTo(pv[edge[1]].x, pv[edge[1]].y);
      robotCtx.stroke();
    }
  });

  // ── XYZ axis arrows (Minecraft F3 style) from center, equal length ──
  const frontVisible = faceNormalZ([pv[0], pv[1], pv[2], pv[3]]) < 0;
  if (frontVisible) {
    const camY = hh * 0.65;
    [-0.3, 0.3].forEach(xOff => {
      const p = project(xOff, camY, hd + 0.01, cx, cy, S);
      const r = S * 0.052;
      robotCtx.beginPath(); robotCtx.arc(p.x, p.y, r, 0, Math.PI*2);
      robotCtx.fillStyle = '#050a14'; robotCtx.fill();
      robotCtx.strokeStyle = 'rgba(0,191,255,0.75)'; robotCtx.lineWidth = 1.2; robotCtx.stroke();
      robotCtx.beginPath(); robotCtx.arc(p.x, p.y, r * 0.55, 0, Math.PI*2);
      robotCtx.fillStyle = '#020508'; robotCtx.fill();
      robotCtx.beginPath(); robotCtx.arc(p.x - r*0.2, p.y - r*0.2, r * 0.22, 0, Math.PI*2);
      robotCtx.fillStyle = 'rgba(0,191,255,0.35)'; robotCtx.fill();
    });
  }
  // Adjust AXIS_LENGTH to change how long all three arrows are.
  const AXIS_LENGTH = 0.9; // shorter than before; all three arrows are this length from center
  const origin3d = project(0, 0, 0, cx, cy, S);
  const axesDef = [
    { tip: [ AXIS_LENGTH, 0, 0],           color: '#ff4444', lw: 2 }, // X red   (+right)
    { tip: [0,  AXIS_LENGTH, 0],           color: '#44ff44', lw: 2 }, // Y green (+up)
    { tip: [0, 0,  AXIS_LENGTH],           color: '#4499ff', lw: 2 }, // Z blue  (+forward)
  ];
  axesDef.forEach(({ tip, color, lw }) => {
    const pTip = project(tip[0], tip[1], tip[2], cx, cy, S);
    drawArrow(robotCtx, origin3d, pTip, color, lw);
  });

  // ── Top overlay: battery + ping ──
  // oFont already defined above, same as opmode text size
  const overlayY = topReserve * 0.5;

  drawMiniVoltage(robotCtx, cx - 72, overlayY, oFont);
  drawMiniPing(robotCtx, cx + 26, overlayY, oFont);

  // ── Bottom overlay: OpMode + timer — same distance from bottom as top overlay from top ──
  if (state.opModeState === 'running' || state.opModeState === 'init') {
    const opName = document.getElementById('opmode-select').value || '—';
    const t = state.opModeTimer;
    const hh2 = Math.floor(t / 3600);
    const mm = String(Math.floor((t % 3600) / 60)).padStart(2, '0');
    const ss = String(t % 60).padStart(2, '0');
    const ts = `${hh2}h ${mm}m ${ss}s`;
    const bFont = oFont; // same size as top overlay
    // Mirror top: top overlay is at topReserve*0.5 from top. Bottom: same from bottom.
    const baseY = H - botReserve + (topReserve * 0.5) - bFont * 2.4; // mirror top overlay
    robotCtx.textAlign = 'center'; robotCtx.textBaseline = 'top';
    robotCtx.fillStyle = 'rgba(100,116,139,0.85)';
    robotCtx.font = `${bFont}px "Space Mono", monospace`;
    robotCtx.fillText('OpMode: ' + opName, cx, baseY);
    robotCtx.font = `bold ${bFont}px "Space Mono", monospace`;
    robotCtx.fillStyle = 'rgba(148,163,184,0.9)';
    robotCtx.fillText(ts, cx, baseY + bFont * 1.25);
  }

  robotCtx.textAlign = 'left'; robotCtx.textBaseline = 'alphabetic';
}

// ── Mini overlay helpers ──────────────────────────────────────────────────────

function drawMiniVoltage(ctx, cx, cy, fontSize) {
  const fs = fontSize || 10;
  const pct = Math.min(1, Math.max(0, (prevVoltageDisplay - 10) / 4.4));
  const bw = 22, bh = 11;
  const bx = cx, by = cy - bh / 2;
  ctx.strokeStyle = 'rgba(148,163,184,0.45)'; ctx.lineWidth = 1.2;
  ctx.strokeRect(bx, by, bw, bh);
  ctx.fillStyle = 'rgba(148,163,184,0.35)';
  ctx.fillRect(bx + bw, by + bh * 0.3, 2.5, bh * 0.4);
  // No color — uniform muted fill
  ctx.fillStyle = 'rgba(148,163,184,0.55)';
  ctx.fillRect(bx + 1.5, by + 1.5, Math.max(0, (bw - 3) * pct), bh - 3);
  ctx.fillStyle = 'rgba(200,214,230,0.85)';
  ctx.font = `${fs}px "Space Mono", monospace`;
  ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
  ctx.fillText(state.voltage.toFixed(1) + 'V', bx + bw + 5, cy + 1.5);
}

function drawMiniPing(ctx, cx, cy, fontSize) {
  const fs = fontSize || 10;
  const ping = state.ping;
  const bars = ping <= 50 ? 4 : ping <= 100 ? 3 : ping <= 200 ? 2 : ping <= 500 ? 1 : 0;
  const barW = 4, gap = 2;
  const heights = [4, 6, 9, 12];
  const totalW = 4 * (barW + gap) - gap;
  for (let i = 0; i < 4; i++) {
    const bx = cx + i * (barW + gap);
    const bh = heights[i];
    const by = cy + 6 - bh;
    // No color — active bars are brighter muted, inactive are dim
    ctx.fillStyle = i < bars ? 'rgba(148,163,184,0.65)' : 'rgba(100,116,139,0.25)';
    ctx.fillRect(bx, by, barW, bh);
  }
  ctx.fillStyle = 'rgba(200,214,230,0.85)';
  ctx.font = `${fs}px "Space Mono", monospace`;
  ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
  ctx.fillText(ping + 'ms', cx + totalW + 4, cy + 1.5);
}

function drawCompassRose(cx, cy, r, fontSize) {
  const ctx = robotCtx;
  const fs  = fontSize || 11;
  // Use the smoothly-lerped heading for animation
  const hRad = -(robotHeadingDisplay * Math.PI / 180);

  // Outer dotted ring
  ctx.save(); ctx.setLineDash([3, 6]);
  ctx.beginPath(); ctx.arc(cx, cy, r, 0, Math.PI*2);
  ctx.strokeStyle = 'rgba(0,191,255,0.22)'; ctx.lineWidth = 1; ctx.stroke();
  ctx.setLineDash([]); ctx.restore();

  // Inner ring
  ctx.beginPath(); ctx.arc(cx, cy, r * 0.88, 0, Math.PI*2);
  ctx.strokeStyle = 'rgba(0,191,255,0.07)'; ctx.lineWidth = 1; ctx.stroke();

  // Rotating cardinal + intercardinal labels and ticks
  const labels = ['N','NE','E','SE','S','SW','W','NW'];
  labels.forEach((lbl, i) => {
    const angle  = hRad + i * Math.PI / 4;
    const isCard = i % 2 === 0, isN = i === 0;
    const tickLen = isN ? 18 : isCard ? 12 : 7;
    const ix = cx + Math.sin(angle) * (r - tickLen);
    const iy = cy - Math.cos(angle) * (r - tickLen);
    const ox = cx + Math.sin(angle) * r;
    const oy = cy - Math.cos(angle) * r;
    ctx.strokeStyle = isN ? 'rgba(239,68,68,0.75)' : isCard ? 'rgba(0,191,255,0.55)' : 'rgba(0,191,255,0.28)';
    ctx.lineWidth = isN ? 2 : isCard ? 1.4 : 0.9;
    ctx.beginPath(); ctx.moveTo(ix, iy); ctx.lineTo(ox, oy); ctx.stroke();
    if (isCard) {
      const lx = cx + Math.sin(angle) * (r - tickLen - 9);
      const ly = cy - Math.cos(angle) * (r - tickLen - 9);
      ctx.fillStyle = isN ? 'rgba(239,68,68,0.85)' : 'rgba(0,191,255,0.75)';
      ctx.font = `bold ${isN ? 12 : 10}px "Space Mono", monospace`;
      ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText(lbl, lx, ly);
    }
  });

  // Minor degree ticks (every 10°)
  for (let deg = 0; deg < 360; deg += 10) {
    if (deg % 45 === 0) continue;
    const angle   = hRad + deg * Math.PI / 180;
    const isMajor = deg % 30 === 0;
    ctx.strokeStyle = 'rgba(0,191,255,0.10)'; ctx.lineWidth = 0.7;
    const inn = r - (isMajor ? 5 : 3);
    ctx.beginPath();
    ctx.moveTo(cx + Math.sin(angle)*inn, cy - Math.cos(angle)*inn);
    ctx.lineTo(cx + Math.sin(angle)*r,   cy - Math.cos(angle)*r);
    ctx.stroke();
  }

  // Center dot
  ctx.beginPath(); ctx.arc(cx, cy, 2.5, 0, Math.PI*2);
  ctx.fillStyle = 'rgba(0,191,255,0.6)'; ctx.fill();

  // ── Fixed heading indicator — plain tick line at top of ring + heading text above ──
  const tickTop    = cy - r;
  const tickBottom = cy - r + 16;
  ctx.strokeStyle = '#00bfff'; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(cx, tickTop - 2); ctx.lineTo(cx, tickBottom); ctx.stroke();

  // Heading degrees text — 11px, 8px above the tick top
  const hdgPx = Math.max(fs, 13);
  ctx.font = `bold ${hdgPx}px "Space Mono", monospace`;
  const hdgText = Math.round(robotHeadingDisplay) + '°';
  ctx.fillStyle = 'rgba(0,191,255,0.9)';
  ctx.textAlign = 'center'; ctx.textBaseline = 'bottom';
  ctx.fillText(hdgText, cx + 4, tickTop - 10);

  ctx.textAlign = 'left'; ctx.textBaseline = 'alphabetic';
}

// ── Camera system ─────────────────────────────────────────────────────────────

const camSwitcherEl  = document.getElementById('cam-switcher');
const csSwitchToggle = document.getElementById('cam-switcher-toggle');

let activeCameraIndex    = null;
let distortionEnabled    = false;
let _lastKnownCamList    = '';
let _selectedCamName     = null;
let _splitView           = true;
window._splitView        = true;   // exposed so child iframes can read it
let _savedSplitAssignment = null;

// camAssignment: pos → channel or null. Positions: TL TR BL BR
const camAssignment = { TL: null, TR: null, BL: null, BR: null };

// Slot state keyed by position
const _slotState = {};

function _initSlotState() {
  ['TL','TR','BL','BR'].forEach(pos => {
    _slotState[pos] = {
      imgEl:   document.getElementById(`cam-img-${pos}`),
      naEl:    document.getElementById(`cam-na-${pos}`),
      slotEl:  document.getElementById(`cam-slot-${pos}`),
      labelEl: document.getElementById(`cam-label-${pos}`),
      watchdog: null,
    };
  });
}

function _feedUrl(channel) {
  // In non-split mode CH01 slot shows the full frame; split mode uses quadrants
  return `/api/camera/feed/${_splitView ? channel : 'full'}?t=${Date.now()}`;
}

function _connectSlot(pos, channel) {
  const st = _slotState[pos];
  st.imgEl.src = _feedUrl(channel);
  st.imgEl.style.display = 'block';
  st.naEl.style.display = 'none';
  _startWatchdog(pos, channel);
}

function _disconnectSlot(pos) {
  const st = _slotState[pos];
  _stopWatchdog(pos);
  st.imgEl.src = '';
  st.imgEl.style.display = 'none';
  st.naEl.style.display = 'flex';
}

function _startWatchdog(pos, channel) {
  _stopWatchdog(pos);
  const cvs = document.createElement('canvas');
  cvs.width = 8; cvs.height = 8;
  let lastHash = '';
  let ticks = 0;
  _slotState[pos].watchdog = setInterval(() => {
    const st = _slotState[pos];
    if (!st.imgEl || st.imgEl.style.display === 'none') return;
    const ctx = cvs.getContext('2d');
    try {
      ctx.drawImage(st.imgEl, 0, 0, 8, 8);
      const hash = ctx.getImageData(0, 0, 8, 8).data.slice(0, 16).join(',');
      ticks++;
      if (hash === lastHash && ticks > 1 && activeCameraIndex !== null) _connectSlot(pos, channel);
      lastHash = hash;
    } catch (_) {}
  }, 4000);
}

function _stopWatchdog(pos) {
  const st = _slotState[pos];
  if (st.watchdog) { clearInterval(st.watchdog); st.watchdog = null; }
}

function refreshCamFeeds() {
  ['TL','TR','BL','BR'].forEach(pos => {
    const ch = camAssignment[pos];
    if (!ch || activeCameraIndex === null) _disconnectSlot(pos);
    else _connectSlot(pos, ch);
  });
}

// Recompute grid layout based on 3 modes:
//   quad      — ≥3 cams selected: all 4 slots always visible (empty → "No Feed"), 2×2
//   side-by-side — 2 cams on opposite sides (left+right): 2-col 1-row
//   top-bottom   — 2 cams on same side (TL+BL or TR+BR): 1-col 2-row
//   single    — 0-1 cams: 1-col 1-row
function updateCamGrid() {
  const grid = document.getElementById('cam-grid');
  const positions = ['TL','TR','BL','BR'];

  const hasTL = !!camAssignment.TL, hasTR = !!camAssignment.TR;
  const hasBL = !!camAssignment.BL, hasBR = !!camAssignment.BR;
  const activeCount = [hasTL, hasTR, hasBL, hasBR].filter(Boolean).length;

  const hasLeft  = hasTL || hasBL;
  const hasRight = hasTR || hasBR;
  const isQuad   = activeCount >= 3;
  const isSideBySide = !isQuad && activeCount === 2 && hasLeft && hasRight;
  const isTopBottom  = !isQuad && activeCount === 2 && !isSideBySide;

  const twoCol = isQuad || isSideBySide;
  const twoRow = isQuad || isTopBottom;

  positions.forEach(pos => {
    const st = _slotState[pos];
    const on = !!camAssignment[pos];
    // Quad mode: all slots always visible (empty ones show "No Feed" naturally)
    st.slotEl.classList.toggle('hidden-slot', !isQuad && !on);
    st.slotEl.style.order = '';
    st.slotEl.classList.remove('border-right', 'border-bottom');
    if (on) {
      st.labelEl.textContent = camAssignment[pos];
      st.labelEl.style.visibility = '';
    } else {
      st.labelEl.textContent = ' '; // non-breaking space keeps height
      st.labelEl.style.visibility = 'hidden';
    }
  });

  grid.style.gridTemplateColumns = twoCol ? '1fr 1fr' : '1fr';
  grid.style.gridTemplateRows    = twoRow ? '1fr 1fr' : '1fr';

  // Borders between adjacent visible slots
  if (twoCol) {
    if (hasTL || isQuad) _slotState.TL.slotEl.classList.add('border-right');
    if (hasBL || isQuad) _slotState.BL.slotEl.classList.add('border-right');
  }
  if (twoRow) {
    if (hasTL || isQuad) _slotState.TL.slotEl.classList.add('border-bottom');
    if (hasTR || isQuad) _slotState.TR.slotEl.classList.add('border-bottom');
  }

  refreshCamFeeds();
  updateCsiActiveStates();
}

// Grid position buttons
document.querySelectorAll('.csi-gbtn').forEach(btn => {
  btn.addEventListener('click', e => {
    e.stopPropagation();
    const cam = btn.dataset.cam;
    const pos = btn.dataset.pos;
    if (camAssignment[pos] === cam) {
      // Toggle off
      camAssignment[pos] = null;
    } else {
      // If this cam is already assigned elsewhere, unassign it there first
      ['TL','TR','BL','BR'].forEach(p => { if (camAssignment[p] === cam) camAssignment[p] = null; });
      // If another cam occupies this pos, unassign it
      camAssignment[pos] = cam;
    }
    updateCamGrid();
  });
});

function updateCsiActiveStates() {
  document.querySelectorAll('.csi-gbtn').forEach(btn => {
    btn.classList.toggle('active', camAssignment[btn.dataset.pos] === btn.dataset.cam);
  });
}

async function _refreshCameraList() {
  try {
    const res  = await fetch('/api/cameras');
    const cams = ((await res.json()).cameras || []).filter(c => !c.hidden);

    const listKey = JSON.stringify(cams.map(c => c.name));
    const sourceSelect = document.getElementById('csi-source-select');
    document.getElementById('csi-source-row').style.display = 'flex';

    if (listKey === _lastKnownCamList) return;
    _lastKnownCamList = listKey;

    // Rebuild dropdown
    while (sourceSelect.options.length > 1) sourceSelect.remove(1);
    cams.forEach(c => {
      const opt = document.createElement('option');
      opt.value = c.uniqueID;
      opt.textContent = c.name + (c.builtin ? ' (built-in)' : '');
      sourceSelect.appendChild(opt);
    });

    // Re-select by name (uniqueID stays stable across index shifts from OBS/etc)
    if (_selectedCamName) {
      const match = cams.find(c => c.name === _selectedCamName);
      if (match) {
        sourceSelect.value = match.uniqueID;
        if (match.uniqueID !== activeCameraIndex) {
          activeCameraIndex = match.uniqueID;
          await fetch('/api/camera/select', {
            method: 'POST', headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ uniqueID: activeCameraIndex })
          });
          refreshCamFeeds();
        }
        return;
      }
      // Device gone — keep name for when it returns
      sourceSelect.value = '';
      activeCameraIndex = null;
      await fetch('/api/camera/select', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ uniqueID: null })
      });
      refreshCamFeeds();
      return;
    }

    if (cams.length === 0) return;
    try {
      const sr = await fetch('/api/settings/default_camera');
      const defaultName = (await sr.json()).name || null;
      const picked = defaultName ? cams.find(c => c.name === defaultName) : null;
      if (picked) {
        sourceSelect.value = picked.uniqueID;
        await selectCameraSource(picked.uniqueID);
      }
    } catch (_) {}
  } catch (_) {}
}

async function initCamera() {
  await _refreshCameraList();
  setInterval(_refreshCameraList, 3000);
}

async function selectCameraSource(uniqueID) {
  activeCameraIndex = (uniqueID === null || uniqueID === undefined || uniqueID === '') ? null : uniqueID;
  const sourceSelect = document.getElementById('csi-source-select');
  const selOpt = [...sourceSelect.options].find(o => o.value === activeCameraIndex);
  // Store the raw device name (strip the " (built-in)" suffix we may have added)
  _selectedCamName = selOpt ? selOpt.textContent.replace(/ \(built-in\)$/, '') : null;
  await fetch('/api/camera/select', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ uniqueID: activeCameraIndex })
  });
  refreshCamFeeds();
}

document.getElementById('csi-source-select').addEventListener('change', async function(e) {
  e.stopPropagation();
  const val = this.value === '' ? null : this.value;
  await selectCameraSource(val);
});

// Distortion toggle
document.getElementById('distort-toggle').addEventListener('change', async function(e) {
  e.stopPropagation();
  distortionEnabled = this.checked;
  await fetch('/api/camera/distortion', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ enabled: distortionEnabled })
  });
  refreshCamFeeds();
});

// Sync undistort toggle state with backend — called on startup and when settings change.
async function loadUndistortModels() {
  try {
    const res  = await fetch('/api/undistort/list');
    const data = await res.json();
    const hasActive = !!data.active;
    const toggle = document.getElementById('distort-toggle');
    const label  = document.getElementById('distort-toggle-label');
    toggle.disabled = !hasActive;
    label.style.opacity = hasActive ? '1' : '0.35';
    label.style.cursor  = hasActive ? 'pointer' : 'not-allowed';
    if (!hasActive) {
      toggle.checked = false;
      distortionEnabled = false;
      // Tell backend distortion is off so frames aren't processed with a stale/missing map
      await fetch('/api/camera/distortion', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ enabled: false })
      });
      refreshCamFeeds();
    }
  } catch (_) {}
}

// Camera switcher dropdown toggle
csSwitchToggle.addEventListener('click', e => { e.stopPropagation(); camSwitcherEl.classList.toggle('open'); });
document.addEventListener('click', e => { if (!camSwitcherEl.contains(e.target)) camSwitcherEl.classList.remove('open'); });

// Split View toggle
function setSplitView(on) {
  _splitView = on;
  window._splitView = on;
  // Dim + disable channel rows when split is off
  const rows = document.getElementById('csi-channel-rows');
  if (rows) { rows.style.opacity = on ? '' : '0.4'; rows.style.pointerEvents = on ? '' : 'none'; }

  if (!on) {
    // Save current assignment (buttons stay visually reflecting it), show full feed in TL only
    _savedSplitAssignment = { ...camAssignment };
    // Don't touch camAssignment — grid buttons remain showing saved state
    // Internally route TL to the full feed by showing only TL with channel 'CH01' (full feed via _feedUrl)
    // We override just the display: hide all slots except TL, show it with label "Main"
    ['TL','TR','BL','BR'].forEach(pos => {
      const st = _slotState[pos];
      if (pos === 'TL') {
        st.slotEl.classList.remove('hidden-slot');
        st.labelEl.textContent = 'Main';
        st.labelEl.style.visibility = '';
        st.imgEl.src = `/api/camera/feed/full?t=${Date.now()}`;
        st.imgEl.style.display = 'block';
        st.naEl.style.display = 'none';
        _startWatchdog(pos, 'full');
      } else {
        st.slotEl.classList.add('hidden-slot');
        _stopWatchdog(pos);
      }
    });
    const grid = document.getElementById('cam-grid');
    grid.style.gridTemplateColumns = '1fr';
    grid.style.gridTemplateRows = '1fr';
    updateCsiActiveStates();
  } else {
    // Restore saved assignment back to grid
    if (_savedSplitAssignment) {
      Object.assign(camAssignment, _savedSplitAssignment);
      _savedSplitAssignment = null;
    }
    updateCamGrid();
  }
}

document.getElementById('split-view-toggle').addEventListener('change', function() {
  setSplitView(this.checked);
});

// OCT panel: toggle .oct-split class when component width ≥ 1200px
(function() {
  const octPanel = document.querySelector('.oct-panel');
  if (octPanel && typeof ResizeObserver !== 'undefined') {
    new ResizeObserver(entries => {
      const w = entries[0].contentRect.width;
      octPanel.classList.toggle('oct-split', w >= 1200);
    }).observe(octPanel);
  }
})();

// OCT Switcher
document.querySelectorAll('.oct-btn').forEach(btn => {
  btn.addEventListener('click', () => {
    document.querySelectorAll('.oct-btn').forEach(b => b.classList.remove('active'));
    document.querySelectorAll('.oct-content').forEach(c => c.classList.remove('active'));
    btn.classList.add('active');
    document.getElementById('oct-' + btn.dataset.oct).classList.add('active');
  });
});

// OpMode
const initBtn  = document.getElementById('opmode-init');
const startBtn = document.getElementById('opmode-start');
const stopBtn  = document.getElementById('opmode-stop');

initBtn.addEventListener('click', () => {
  const sel = document.getElementById('opmode-select').value;
  if (!sel) return;
  sendWS({ type: 'opMode', action: 'init', name: sel });
  handleOpModeState('init');
});

// Gray out INIT whenever no opmode is selected
document.getElementById('opmode-select').addEventListener('change', function() {
  if (state.opModeState === 'idle') {
    initBtn.disabled = !this.value;
  }
});

startBtn.addEventListener('click', () => {
  if (state.opModeState === 'init') {
    sendWS({ type: 'opMode', action: 'start' });
    handleOpModeState('running');
  }
});

stopBtn.addEventListener('click', () => {
  sendWS({ type: 'opMode', action: 'stop' });
  handleOpModeState('idle'); // go all the way back to idle so INIT re-enables
});

function handleOpModeState(s) {
  state.opModeState = s;

  // INIT: only enabled when idle AND an opmode is selected
  initBtn.disabled = s !== 'idle' || !document.getElementById('opmode-select').value;

  if (s === 'running') {
    startBtn.innerHTML = `<span>RUNNING</span>`;
    startBtn.classList.add('running');
    startBtn.style.background = '#4ade80'; // brighter green
    startBtn.disabled = true;
  } else {
    startBtn.innerHTML = `<svg width="18" height="18" viewBox="0 0 24 24" fill="currentColor"><polygon points="5,3 19,12 5,21"/></svg><span>START</span>`;
    startBtn.classList.remove('running');
    startBtn.style.background = 'var(--green)';
    startBtn.disabled = s !== 'init';
  }

  // STOP: enabled when init'd or running
  stopBtn.disabled = s !== 'running' && s !== 'init';

  if (s === 'running') {
    state.opModeTimer = 0;
    clearInterval(state.timerInterval);
    state.timerInterval = setInterval(() => { state.opModeTimer++; updateTimerDisplay(); }, 1000);
  } else {
    clearInterval(state.timerInterval);
    state.timerInterval = null;
    if (s === 'idle') state.opModeTimer = 0;
  }
  updateTimerDisplay();
}

function updateTimerDisplay() {
  const t = state.opModeTimer;
  const str = `${Math.floor(t/3600)}h ${String(Math.floor((t%3600)/60)).padStart(2,'0')}m ${String(t%60).padStart(2,'0')}s`;
  document.getElementById('opmode-timer-val').textContent = state.opModeState === 'idle' ? '—' : str;
  const op = document.getElementById('opmode-select').value || '—';
  ['telem-opmode-badge','ctrl-opmode-badge'].forEach(id => { const el = document.getElementById(id); if (el) el.textContent = op; });
  ['telem-time-badge','ctrl-time-badge'].forEach(id => { const el = document.getElementById(id); if (el) el.textContent = state.opModeState === 'idle' ? '—' : str; });
}

function renderTelemetry() {
  const body = document.getElementById('telemetry-body');
  const entries = Object.entries(state.telemetry);
  body.innerHTML = entries.length
    ? entries.map(([k,v]) => `<div class="telem-row"><span class="telem-key">${k}</span><span class="telem-val">${v}</span></div>`).join('')
    : '<div class="telem-empty">No telemetry data</div>';
}

// ── Control mode: 'controller' | 'keyboard', multi allows both ──
let activeCtab = 'controller';
let multiControl = false;

function releaseAllPressed() {
  document.querySelectorAll('.ctrl-key-btn.pressed').forEach(b => b.classList.remove('pressed'));
  pressedKeys.clear();
}

function setCtab(id) {
  releaseAllPressed();
  activeCtab = id;
  document.querySelectorAll('.ctrl-tab').forEach(t => t.classList.toggle('active', t.dataset.ctab === id));
  document.querySelectorAll('.ctrl-tab-content').forEach(c => c.classList.toggle('active', c.id === 'ctab-' + id));
}

function setMultiControl(on) {
  multiControl = on;
  const label = document.getElementById('ctrl-multi-label');
  const tabs  = document.querySelector('.ctrl-tabs');
  label.classList.toggle('checked', on);
  tabs.classList.toggle('ctrl-multi-active', on);
}

document.querySelectorAll('.ctrl-tab').forEach(tab => {
  tab.addEventListener('click', () => setCtab(tab.dataset.ctab));
});

document.getElementById('ctrl-multi-check').addEventListener('change', e => {
  setMultiControl(e.target.checked);
});

document.querySelectorAll('.ctrl-key-btn').forEach(btn => {
  btn.addEventListener('mousedown', () => { btn.classList.add('pressed'); sendWS({ type: 'control', action: btn.dataset.action, value: 1 }); });
  btn.addEventListener('mouseup',    () => { btn.classList.remove('pressed'); sendWS({ type: 'control', action: btn.dataset.action, value: 0 }); });
  btn.addEventListener('mouseleave', () => btn.classList.remove('pressed'));
});

function drawKbGroupPath() {
  const grid = document.querySelector('.ctrl-kb');
  const svg  = grid && grid.querySelector('.kb-group-svg');
  const path = svg  && svg.querySelector('.kb-group-path');
  if (!path) return;

  const GAP  = 8, COLS = 13, ROWS = 5, PAD_X = 8, PAD_Y = 6, BLEED = 4;
  const W    = grid.clientWidth  - PAD_X * 2;
  const H    = grid.clientHeight - PAD_Y * 2;
  const colW = (W - (COLS - 1) * GAP) / COLS;
  const rowH = (H - (ROWS - 1) * GAP) / ROWS;

  const cellX = i => PAD_X + i * (colW + GAP);
  const cellY = i => PAD_Y + i * (rowH + GAP);

  // W = col2 row0, ASD = cols1-3 row1  (0-indexed, matching grid-template-areas)
  const wL = cellX(2) - BLEED,  wR = cellX(2) + colW + BLEED;
  const wT = cellY(0) - BLEED,  wB = cellY(0) + rowH + BLEED;
  const aL = cellX(1) - BLEED,  dR = cellX(3) + colW + BLEED;
  const aB = cellY(1) + rowH + BLEED;
  const R  = 5;

  const CW  = (x, y) => `A${R},${R} 0 0,1 ${x},${y}`;
  const CCW = (x, y) => `A${R},${R} 0 0,0 ${x},${y}`;

  const d = [
    `M ${wL + R},${wT}`,
    `L ${wR - R},${wT}`,     CW(wR,      wT + R),  // B: top-right of W
    `L ${wR},    ${wB - R}`, CCW(wR + R,  wB),     // F: same as L (BR)
    `L ${dR - R},${wB}`,     CW(dR,      wB + R),  // H: bottom-right of D
    `L ${dR},    ${aB - R}`, CW(dR - R,  aB),      // L: bottom-right of ASD
    `L ${aL + R},${aB}`,     CW(aL,      aB - R),  // I: bottom-left of ASD
    `L ${aL},    ${wB + R}`, CW(aL + R,  wB),      // G: same as I
    `L ${wL - R},${wB}`,     CCW(wL,     wB - R),  // E: bottom-left of W (BL)
    `L ${wL},    ${wT + R}`, CW(wL + R,  wT),      // A: top-left of W
    'Z'
  ].join(' ');

  path.setAttribute('d', d);
}

// Use ResizeObserver on the grid itself so it redraws on any size change, not just window resize
(function() {
  let _kbRo = null;
  function _attachKbObserver() {
    const grid = document.querySelector('.ctrl-kb');
    if (!grid) return;
    if (_kbRo) _kbRo.disconnect();
    _kbRo = new ResizeObserver(() => drawKbGroupPath());
    _kbRo.observe(grid);
  }
  window.addEventListener('resize', drawKbGroupPath);
  // Attach observer after DOM is ready; re-attach if ctrl tab switches make it visible
  document.addEventListener('DOMContentLoaded', _attachKbObserver);
  // Also re-draw whenever the keyboard tab becomes active
  document.addEventListener('click', e => {
    if (e.target.closest && e.target.closest('[data-ctab="keyboard"]')) {
      requestAnimationFrame(drawKbGroupPath);
    }
  });
})();

const KB_MAP = {
  'w': 'drive_fwd', 'a': 'drive_left', 's': 'drive_back', 'd': 'drive_right',
  ' ': 'ascend', 'shift': 'descend',
  'arrowup': 'pitch_up', 'arrowdown': 'pitch_down',
  'arrowleft': 'pitch_left', 'arrowright': 'pitch_right',
  'q': 'rotate_claw1_ccw', 'e': 'rotate_claw1_cw', 'z': 'rotate_claw2_ccw', 'c': 'rotate_claw2_cw',
  'r': 'claw1_open', 't': 'claw2_open', 'f': 'claw1_close', 'g': 'claw2_close',
};
const pressedKeys = new Set();
const kbContent = () => document.getElementById('ctab-keyboard');
window.addEventListener('keydown', e => {
  if (activeCtab !== 'keyboard' && !multiControl) return;
  const key = e.key.toLowerCase();
  if (pressedKeys.has(key)) return; pressedKeys.add(key);
  const action = KB_MAP[key];
  if (action) {
    e.preventDefault();
    sendWS({ type: 'control', action, value: 1 });
    kbContent().querySelectorAll(`.ctrl-key-btn[data-key="${e.key}"], .ctrl-key-btn[data-key="${e.key.toUpperCase()}"]`).forEach(b => b.classList.add('pressed'));
  }
});
window.addEventListener('keyup', e => {
  if (activeCtab !== 'keyboard' && !multiControl) return;
  const key = e.key.toLowerCase(); pressedKeys.delete(key);
  const action = KB_MAP[key];
  if (action) {
    sendWS({ type: 'control', action, value: 0 });
    kbContent().querySelectorAll(`.ctrl-key-btn[data-key="${e.key}"], .ctrl-key-btn[data-key="${e.key.toUpperCase()}"]`).forEach(b => b.classList.remove('pressed'));
  }
});

// ── Gamepad polling (all connected controllers, standard mapping) ──
// Standard button indices for F310 in "D" mode (XInput/standard layout):
const GP_BTN_MAP = {
  0:  'claw2_close',       // A
  1:  'claw1_close',       // B
  2:  'claw2_open',        // X
  3:  'claw1_open',        // Y
  4:  'rotate_claw1_ccw',  // LB
  5:  'rotate_claw1_cw',   // RB
  6:  'rotate_claw2_ccw',  // LT (digital)
  7:  'rotate_claw2_cw',   // RT (digital)
  12: 'drive_fwd',         // DPAD up
  13: 'drive_back',        // DPAD down
  14: 'drive_left',        // DPAD left
  15: 'drive_right',       // DPAD right
};
// Axes: left stick — index 1 (up/down), index 0 (left/right)
const GP_AXIS_ACTIONS = {
  '1-': 'ascend',       // LS up
  '1+': 'descend',      // LS down
  '2-': 'pitch_left',   // RS horizontal left  (axis 2)
  '2+': 'pitch_right',  // RS horizontal right
};
const GP_AXIS_THRESHOLD = 0.25;

const gpBtnState  = {};  // key: `${gpIndex}-${btnIndex}` → bool
const gpAxisState = {};  // key: `${gpIndex}-${axisKey}`  → bool

function pollGamepads() {
  if (activeCtab !== 'controller' && !multiControl) {
    requestAnimationFrame(pollGamepads); return;
  }
  const pads = navigator.getGamepads ? Array.from(navigator.getGamepads()).filter(Boolean) : [];
  for (const gp of pads) {
    // Buttons
    gp.buttons.forEach((btn, i) => {
      const action = GP_BTN_MAP[i]; if (!action) return;
      const key    = `${gp.index}-${i}`;
      const pressed = btn.pressed;
      if (pressed === gpBtnState[key]) return;
      gpBtnState[key] = pressed;
      sendWS({ type: 'control', action, value: pressed ? 1 : 0 });
      document.querySelectorAll(`.ctrl-key-btn[data-action="${action}"]`).forEach(b => b.classList.toggle('pressed', pressed));
    });
    // Axes
    [1, 2].forEach(ai => {
      const val = gp.axes[ai] ?? 0;
      const negKey = `${gp.index}-${ai}-`;
      const posKey = `${gp.index}-${ai}+`;
      const negAction = GP_AXIS_ACTIONS[`${ai}-`];
      const posAction = GP_AXIS_ACTIONS[`${ai}+`];
      const negActive = val < -GP_AXIS_THRESHOLD;
      const posActive = val >  GP_AXIS_THRESHOLD;
      if (negAction && negActive !== !!gpAxisState[negKey]) {
        gpAxisState[negKey] = negActive;
        sendWS({ type: 'control', action: negAction, value: negActive ? 1 : 0 });
        document.querySelectorAll(`.ctrl-key-btn[data-action="${negAction}"]`).forEach(b => b.classList.toggle('pressed', negActive));
      }
      if (posAction && posActive !== !!gpAxisState[posKey]) {
        gpAxisState[posKey] = posActive;
        sendWS({ type: 'control', action: posAction, value: posActive ? 1 : 0 });
        document.querySelectorAll(`.ctrl-key-btn[data-action="${posAction}"]`).forEach(b => b.classList.toggle('pressed', posActive));
      }
    });
  }
  requestAnimationFrame(pollGamepads);
}
requestAnimationFrame(pollGamepads);

// Task Switcher
const taskButtons = document.querySelectorAll('.task-btn');
const taskContent = document.getElementById('task-content');
const taskFrames  = {}, taskLoaded = {};
let   currentTask = null;

taskButtons.forEach(btn => {
  const id = btn.dataset.task;
  const frame = document.createElement('iframe');
  frame.className = 'task-frame'; frame.scrolling = 'yes'; frame.dataset.task = id;
  // On each load, briefly dip opacity then restore — masks jittery reloads
  frame.addEventListener('load', () => {
    if (!frame.classList.contains('active')) return;
    frame.style.transition = 'none';
    frame.style.opacity = '0.3';
    requestAnimationFrame(() => requestAnimationFrame(() => {
      frame.style.transition = 'opacity 0.35s ease-in-out';
      frame.style.opacity = '1';
      setTimeout(() => { frame.style.transition = ''; frame.style.opacity = ''; }, 250);
    }));
  });
  taskContent.appendChild(frame);
  taskFrames[id] = frame;
});

function switchTask(taskId, forceReload) {
  taskButtons.forEach(b => b.classList.toggle('active', b.dataset.task === taskId));

  const incoming = taskFrames[taskId];
  const outgoing = Object.values(taskFrames).find(f => f.classList.contains('active') && f !== incoming);

  if (forceReload || !taskLoaded[taskId]) {
    // Briefly hide to mask jittery reload, fade back in once loaded
    incoming._loadPending = true;
    incoming.style.transition = 'none';
    incoming.style.opacity = '0';
    incoming.onload = () => {
      incoming._loadPending = false;
      requestAnimationFrame(() => {
        requestAnimationFrame(() => {
          incoming.style.transition = 'opacity 0.3s ease';
          incoming.style.opacity = incoming.classList.contains('active') ? '1' : '0';
        });
      });
    };
    incoming.src = `/t/${taskId}`;
    taskLoaded[taskId] = true;
  }

  // Tab switch: instant show/hide (no crossfade between tabs)
  if (outgoing) {
    outgoing.classList.remove('active');
    outgoing.style.opacity = '';
    outgoing.style.transition = '';
  }
  Object.values(taskFrames).forEach(f => {
    if (f !== incoming) { f.classList.remove('active'); f.style.opacity = ''; f.style.transition = ''; }
  });
  incoming.classList.add('active');
  // If reload just set the src, onload will handle the fade-in
  if (!incoming.onload && incoming.style.opacity !== '0') {
    incoming.style.opacity = '';
    incoming.style.transition = '';
  }

  currentTask = taskId;
}

taskButtons.forEach(btn => {
  btn.addEventListener('click', () => switchTask(btn.dataset.task, btn.dataset.task === currentTask));
});

// ── Viz square enforcer — width always equals height ─────────────────────────
function initVizSquare() {
  const viz     = document.getElementById('robot-viz-outer');
  const widgets = document.getElementById('widgets-panel');
  const bottom  = document.querySelector('.bottom-area');
  if (!viz || !bottom) return;

  function enforceSquare() {
    const colGap   = 10;
    const bottomH  = bottom.getBoundingClientRect().height;
    const widgetsH = widgets ? widgets.getBoundingClientRect().height : 0;
    const vizH     = Math.max(0, bottomH - widgetsH - colGap);
    viz.style.width   = vizH + 'px';
    viz.style.height  = vizH + 'px';
    if (widgets) widgets.style.width = vizH + 'px';
  }

  enforceSquare();
  new ResizeObserver(enforceSquare).observe(bottom);
}

// ── Camera panel resize handle ────────────────────────────────────────────────
function initCamResize() {
  const handle   = document.getElementById('cam-resize-handle');
  const panel    = document.getElementById('cameras-panel');
  const mainCol  = panel.parentElement;
  if (!handle || !panel) return;

  const MIN_CAM    = 350;  // minimum camera panel height
  const MIN_BOTTOM = 410;  // minimum bottom-area height
  const MAX_BOTTOM = 800;  // maximum bottom-area height

  function clampedH(desired, colH) {
    const maxFromBottom = colH - MIN_BOTTOM;
    const minFromBottom = colH - MAX_BOTTOM;
    return Math.max(MIN_CAM, Math.max(minFromBottom, Math.min(desired, maxFromBottom)));
  }

  // Clamp on load so a reload never lands in an illegal state
  const colHInit = mainCol.getBoundingClientRect().height;
  panel.style.height = clampedH(panel.getBoundingClientRect().height, colHInit) + 'px';
  panel.style.flex   = 'none';

  let startY, startH;

  handle.addEventListener('mousedown', e => {
    e.preventDefault();
    startY = e.clientY;
    startH = panel.getBoundingClientRect().height;
    handle.classList.add('dragging');
    document.body.style.cursor = 'ns-resize';
    document.body.style.userSelect = 'none';

    function onMove(e) {
      const dy   = e.clientY - startY;
      const colH = mainCol.getBoundingClientRect().height;
      panel.style.height = clampedH(startH + dy, colH) + 'px';
      panel.style.flex   = 'none';
    }
    function onUp() {
      handle.classList.remove('dragging');
      document.body.style.cursor = '';
      document.body.style.userSelect = '';
      window.removeEventListener('mousemove', onMove);
      window.removeEventListener('mouseup', onUp);
    }
    window.addEventListener('mousemove', onMove);
    window.addEventListener('mouseup', onUp);
  });

  // Re-clamp on every window resize so the camera panel stays within bounds.
  window.addEventListener('resize', () => {
    const colH = mainCol.getBoundingClientRect().height;
    panel.style.height = clampedH(panel.getBoundingClientRect().height, colH) + 'px';
    panel.style.flex   = 'none';
  });
}

// Init
window.addEventListener('DOMContentLoaded', () => {
  // Build visualizer rulers
  // Meters: 110px/m, major=1m, mid=0.2m (1/5th), minor=0.1m (1/10th)
  buildVizRuler('df-m-inner',  40,  1, 0.2, 0.1, 'm',  RULER_PX_PER_M);
  // Feet: ~33.5px/ft, major=1ft, mid=0.5ft (1/2), minor=0.25ft (1/4)
  buildVizRuler('df-ft-inner', 130, 1, 0.5, 0.25, 'ft', RULER_PX_PER_FT);

  // Build widget ruler (no labels)
  // depth 0–30m, major every 5m, mid every 1m, minor every 0.5m
  buildWidgetRuler('wc-depth-strip', 30, 5, 1, 0.5);

  _initSlotState();
  initRobot2D();
  // Load default cam assignments from user settings (settings.json), fall back to all-null
  fetch('/api/settings/camera_slots').then(r => r.ok ? r.json() : {}).then(slots => {
    ['TL','TR','BL','BR'].forEach(p => { camAssignment[p] = (slots && slots[p]) || null; });
    updateCamGrid();
  }).catch(() => { updateCamGrid(); });
  initVizSquare();
  initCamResize();
  updateConnectionState(false);
  initCamera();
  loadUndistortModels();

  if (window.electronAPI?.onUndistortChanged) {
    window.electronAPI.onUndistortChanged(() => loadUndistortModels());
  }

  if (window.electronAPI?.onTrafficLightsHidden) {
    window.electronAPI.onTrafficLightsHidden(() => document.documentElement.classList.add('no-traffic-lights'));
    window.electronAPI.onTrafficLightsVisible(() => document.documentElement.classList.remove('no-traffic-lights'));
  }

  if (window.electronAPI?.onWindowResizeStart) {
    window.electronAPI.onWindowResizeStart(pauseDrawLoop);
    window.electronAPI.onWindowResizeEnd(resumeDrawLoop);
  }

  // INIT starts disabled — no opmode selected yet
  document.getElementById('opmode-init').disabled = true;

  drawKbGroupPath();
  connectWS();
  switchTask('1_2', true);

  if (SIMULATE_DATA) {
    updateVoltage(); updatePing(); updateDepth(); updateHeading();
    setInterval(() => {
      if (!state.wsConnected) return;
      state.voltage = 11.8 + Math.random() * 2.6;
      state.ping    = Math.floor(40 + Math.random() * 460);
      state.depth   = Math.max(0, state.depth + (Math.random() - 0.5) * 0.4);
      state.heading = (state.heading + (Math.random() - 0.5) * 5 + 360) % 360;
      updateVoltage(); updatePing(); updateDepth(); updateHeading();
      if (SIMULATE_OPMODE && state.opModeState !== 'idle') {
        state.telemetry['Depth (m)']   = state.depth.toFixed(1);
        state.telemetry['Heading']     = Math.round(state.heading) + '°';
        state.telemetry['Voltage']     = state.voltage.toFixed(2) + 'V';
        state.telemetry['Thruster FL'] = Math.floor(60 + Math.random() * 30) + '%';
        state.telemetry['Thruster FR'] = Math.floor(60 + Math.random() * 30) + '%';
        renderTelemetry();
      }
    }, 2000);
  }
});