// 1_2_viewer.js — Three.js viewer for the Task 1.2 coral-garden Model3D.
//
// C++ model3d.hpp coordinate system:
//   model +X = length (left→right)
//   model +Y = width/depth (front→back, ~0.36 m)
//   model +Z = height (floor→top)
//   section.size   = [length_x, width_y, height_z]
//   section.origin = center of BOTTOM face  (origin.z=0, origin.y=-width/2)
//
// Viewer (Three.js) coordinate system:
//   viewer X = model X  (length, left→right)
//   viewer Y = model Z  (height, floor→top)
//   viewer Z = model Y  (depth,  back→front)
//
// Mapping in setModel():
//   vSize   = [size[0], size[2], size[1]]           // [length, height, depth]
//   vOrigin = [origin[0] - size[0]/2,  0,  0]       // [x_left, y_floor, z_back]
//
// Plate face remapping (C++ → viewer):
//   "+y" → "+z"  (C++ front  = viewer front)
//   "-y" → "-z"  (C++ back   = viewer back)
//   "+z" → "+y"  (C++ top    = viewer top)
//   "-z" → "-y"  (C++ bottom = viewer bottom)
//   "+x" / "-x" unchanged
//
// Exposes window.Task12Viewer = { setModel, resetView, toggleDimensions,
//                                  exportGLB, exportOBJ }.
(() => {
"use strict";

const canvas = document.getElementById("pg-3d-canvas");
if (!canvas) return;

if (typeof THREE === "undefined") {
  const ctx = canvas.getContext("2d");
  const draw = () => {
    canvas.width  = canvas.clientWidth  * devicePixelRatio;
    canvas.height = canvas.clientHeight * devicePixelRatio;
    ctx.fillStyle = "#0a0e1a"; ctx.fillRect(0,0,canvas.width,canvas.height);
    ctx.fillStyle = "#64748b"; ctx.font = `${14*devicePixelRatio}px 'Space Mono', monospace`;
    ctx.textAlign = "center";
    ctx.fillText("Three.js failed to load (offline?). Model JSON still available.",
                 canvas.width/2, canvas.height/2);
  };
  draw(); window.addEventListener("resize", draw);
  window.Task12Viewer = { setModel(){draw();}, resetView(){}, toggleDimensions(){}, exportGLB(){}, exportOBJ(){} };
  return;
}

// ── Three setup ────────────────────────────────────────────────────────
const renderer = new THREE.WebGLRenderer({ canvas, antialias:true, alpha:true });
renderer.setPixelRatio(devicePixelRatio);
renderer.shadowMap.enabled = false;

const scene  = new THREE.Scene();
scene.background = new THREE.Color(0x0a0e1a);

const camera = new THREE.PerspectiveCamera(45, 1, 0.001, 100000);
camera.position.set(1.5, 0.8, 2.2);

const controls = new THREE.OrbitControls(camera, canvas);
controls.enableDamping = true; controls.dampingFactor = 0.08;

// Lighting: hemisphere for ambient fill + directional for shape definition
scene.add(new THREE.HemisphereLight(0xbfd8ff, 0x152033, 0.7));
const sun = new THREE.DirectionalLight(0xffffff, 1.0);
sun.position.set(3, 5, 4);
scene.add(sun);
// Subtle fill from the front
const fill = new THREE.DirectionalLight(0x8090b0, 0.3);
fill.position.set(-1, 1, 5);
scene.add(fill);

// Grid sits on the floor (Y=0 plane). Replaced per-model in setModel().
let grid = new THREE.GridHelper(4, 16, 0x1a2840, 0x1a2840);
scene.add(grid);

let modelGroup = new THREE.Group(); scene.add(modelGroup);
let dimGroup   = new THREE.Group(); scene.add(dimGroup);
let dimsVisible = true;

// Current model state — kept so we can re-render on flip/reset
let _currentModel = null;
let _flipped = false;

// Canonical default model (shown when detections are insufficient)
const DEFAULT_MODEL = {
  sections: [
    { size: [0.30, 0.36, 0.25], origin: [0.15, -0.18, 0] },
    { size: [0.40, 0.36, 0.55], origin: [0.50, -0.18, 0] },
    { size: [0.30, 0.36, 0.40], origin: [0.85, -0.18, 0] },
  ],
  plates: [],
  totals: { length: 1.00, width: 0.36, height: 0.55 },
  scale: { source: "canonical default", confidence: 0 },
  n_pairs_used: 0,
};

function fit() {
  const r = canvas.getBoundingClientRect();
  renderer.setSize(r.width, r.height, false);
  camera.aspect = r.width / r.height;
  camera.updateProjectionMatrix();
}
new ResizeObserver(fit).observe(canvas);
fit();

(function loop(){
  requestAnimationFrame(loop);
  controls.update();
  renderer.render(scene, camera);
})();

// ── PVC material cache ─────────────────────────────────────────────────
const PVC_COLOR   = 0xe8e4dc;
const JOINT_SCALE = 1.5;

const pvcMat = new THREE.MeshStandardMaterial({
  color: PVC_COLOR, roughness: 0.6, metalness: 0.02,
});

function clear(g) {
  while (g.children.length) {
    const c = g.children[0];
    c.geometry?.dispose?.();
    g.remove(c);
  }
}

// ── Geometry helpers ───────────────────────────────────────────────────

// One PVC cylinder between two world-space points.
function makePipeSeg(p1, p2, radius) {
  const dir = new THREE.Vector3().subVectors(p2, p1);
  const len = dir.length();
  if (len < 1e-9) return null;
  const geo = new THREE.CylinderGeometry(radius, radius, len, 12, 1, false);
  const mesh = new THREE.Mesh(geo, pvcMat);
  mesh.position.copy(new THREE.Vector3().addVectors(p1, p2).multiplyScalar(0.5));
  mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0), dir.clone().normalize());
  return mesh;
}

// Joint sphere at a corner.
const _jointGeoCache = {};
function makeJoint(pos, radius) {
  const key = radius.toFixed(5);
  if (!_jointGeoCache[key]) {
    _jointGeoCache[key] = new THREE.SphereGeometry(radius * JOINT_SCALE, 12, 8);
  }
  const s = new THREE.Mesh(_jointGeoCache[key], pvcMat);
  s.position.copy(pos);
  return s;
}

// Build the 12-edge wireframe for one rectangular section.
// size   = [length, height, depth]  (X, Y, Z extents)
// origin = [x0, y0, z0]             (bottom-front-left corner)
//
// The sections sit on the floor (y0 = 0) and share the same front face
// (z0 = 0), so the only thing that varies between sections is:
//   - x0  (where the section starts along the length axis)
//   - size[0] (how long it is)
//   - size[1] (how tall it is)
//   - size[2] (depth, always == model width = 0.36 m)
function makeSectionFrame(size, origin, radius) {
  const [sl, sh, sd] = size;
  const [ox, oy, oz] = origin;
  const grp = new THREE.Group();

  // 8 corners: bottom-front-left to top-back-right
  // Index convention:
  //   0: (ox,    oy,    oz)      bottom-front-left
  //   1: (ox+sl, oy,    oz)      bottom-front-right
  //   2: (ox+sl, oy+sh, oz)      top-front-right
  //   3: (ox,    oy+sh, oz)      top-front-left
  //   4: (ox,    oy,    oz+sd)   bottom-back-left
  //   5: (ox+sl, oy,    oz+sd)   bottom-back-right
  //   6: (ox+sl, oy+sh, oz+sd)   top-back-right
  //   7: (ox,    oy+sh, oz+sd)   top-back-left
  const C = [
    new THREE.Vector3(ox,    oy,    oz),
    new THREE.Vector3(ox+sl, oy,    oz),
    new THREE.Vector3(ox+sl, oy+sh, oz),
    new THREE.Vector3(ox,    oy+sh, oz),
    new THREE.Vector3(ox,    oy,    oz+sd),
    new THREE.Vector3(ox+sl, oy,    oz+sd),
    new THREE.Vector3(ox+sl, oy+sh, oz+sd),
    new THREE.Vector3(ox,    oy+sh, oz+sd),
  ];

  // 12 edges
  const edges = [
    [0,1],[1,2],[2,3],[3,0],   // front face
    [4,5],[5,6],[6,7],[7,4],   // back face
    [0,4],[1,5],[2,6],[3,7],   // depth connectors
  ];
  for (const [a,b] of edges) {
    const seg = makePipeSeg(C[a], C[b], radius);
    if (seg) grp.add(seg);
  }

  // Joints at all 8 corners
  for (const corner of C) grp.add(makeJoint(corner, radius));

  return grp;
}

// Plate: a 10×10 cm square glued to a section face.
// face  = "+z" (front) | "-z" (back) | "+x" | "-x" | "+y" | "-y"
// u     = 0..1 along section length  (X axis)
// v     = 0..1 along section height  (Y axis)
//
// The plate stands slightly proud of the face (lift) so it doesn't
// z-fight with the pipe frame.
const plateMat = new THREE.MeshStandardMaterial({
  color: 0x9333ea,
  emissive: 0x9333ea,
  emissiveIntensity: 0.3,
  roughness: 0.35,
  metalness: 0.0,
  side: THREE.DoubleSide,
});

function readPlateColor() {
  const el = document.getElementById("plate-color");
  if (!el) return 0x9333ea;
  const hex = el.value.replace("#", "");
  return parseInt(hex, 16);
}

function makePlate(p, secOrigin, secSize) {
  const ps = p.side_m || 0.10;
  const geo = new THREE.PlaneGeometry(ps, ps);
  const mesh = new THREE.Mesh(geo, plateMat);

  const [sl, sh, sd] = secSize;
  const [ox, oy, oz] = secOrigin;
  const u = p.u ?? 0.5;
  const v = p.v ?? 0.5;
  const lift = Math.max(0.004, ps * 0.12);

  // Clamp so the plate stays fully on the face
  const hu = ps / (2 * Math.max(sl, 1e-6));
  const hv = ps / (2 * Math.max(sh, 1e-6));
  const cu = Math.max(hu, Math.min(1 - hu, u));
  const cv = Math.max(hv, Math.min(1 - hv, v));

  // Remap C++ model face names → viewer face names.
  // C++ model:  +X=length, +Y=width/depth, +Z=height
  // Viewer:     +X=length, +Y=height,      +Z=depth
  //   C++ "+y" (front face, camera-facing) → viewer "+z"
  //   C++ "-y" (back face)                 → viewer "-z"
  //   C++ "+z" (top)                       → viewer "+y"
  //   C++ "-z" (bottom)                    → viewer "-y"
  //   C++ "+x" / "-x" unchanged
  const faceRemap = { "+y": "+z", "-y": "-z", "+z": "+y", "-z": "-y" };
  const viewerFace = faceRemap[p.face] ?? p.face;

  switch (viewerFace) {
    case "+z": {
      // Front face (Z = oz + sd). u along X, v along Y.
      mesh.position.set(ox + cu*sl, oy + cv*sh, oz + sd + lift);
      // Default plane faces +Z — correct
      break;
    }
    case "-z": {
      // Back face (Z = oz). u along X, v along Y.
      mesh.position.set(ox + cu*sl, oy + cv*sh, oz - lift);
      mesh.rotation.y = Math.PI;
      break;
    }
    case "+x": {
      // Right face (X = ox + sl). u along Z(depth), v along Y.
      const huz = ps / (2 * Math.max(sd, 1e-6));
      const cuz = Math.max(huz, Math.min(1 - huz, u));
      mesh.position.set(ox + sl + lift, oy + cv*sh, oz + cuz*sd);
      mesh.rotation.y = Math.PI/2;
      break;
    }
    case "-x": {
      // Left face (X = ox). u along Z(depth), v along Y.
      const huz = ps / (2 * Math.max(sd, 1e-6));
      const cuz = Math.max(huz, Math.min(1 - huz, u));
      mesh.position.set(ox - lift, oy + cv*sh, oz + cuz*sd);
      mesh.rotation.y = -Math.PI/2;
      break;
    }
    case "+y": {
      // Top face. u along X, v along Z.
      const hvz = ps / (2 * Math.max(sd, 1e-6));
      const cvz = Math.max(hvz, Math.min(1 - hvz, v));
      mesh.position.set(ox + cu*sl, oy + sh + lift, oz + cvz*sd);
      mesh.rotation.x = -Math.PI/2;
      break;
    }
    case "-y": default: {
      // Bottom face. u along X, v along Z.
      const hvz = ps / (2 * Math.max(sd, 1e-6));
      const cvz = Math.max(hvz, Math.min(1 - hvz, v));
      mesh.position.set(ox + cu*sl, oy - lift, oz + cvz*sd);
      mesh.rotation.x = Math.PI/2;
      break;
    }
  }
  return mesh;
}

// Dimension annotation: a line from A to B with end-cap ticks and a
// centered billboard label. axis = "x" | "y" determines tick orientation.
function makeDimLine(a, b, labelText, axis, modelExt) {
  const grp = new THREE.Group();
  const DIM_COLOR = 0x38bdf8;  // sky-blue

  // Main line
  const pts = [a.clone(), b.clone()];
  const lineGeo = new THREE.BufferGeometry().setFromPoints(pts);
  const lineMat = new THREE.LineBasicMaterial({ color: DIM_COLOR, depthTest: false });
  const line = new THREE.Line(lineGeo, lineMat);
  line.renderOrder = 998;
  grp.add(line);

  // End-cap ticks (short perpendicular lines at A and B)
  const tickLen = Math.max(0.012, modelExt * 0.022);
  // Tick direction: perpendicular to the dimension line in the XY plane.
  // For a length (X) line the tick goes in Y; for a height (Y) line it goes in X.
  const tickDir = axis === "x"
    ? new THREE.Vector3(0, 1, 0)
    : new THREE.Vector3(1, 0, 0);

  for (const end of [a, b]) {
    const t0 = end.clone().addScaledVector(tickDir, -tickLen * 0.5);
    const t1 = end.clone().addScaledVector(tickDir,  tickLen * 0.5);
    const tGeo = new THREE.BufferGeometry().setFromPoints([t0, t1]);
    const tLine = new THREE.Line(tGeo, lineMat);
    tLine.renderOrder = 998;
    grp.add(tLine);
  }

  // Billboard label at the midpoint
  const mid = new THREE.Vector3().addVectors(a, b).multiplyScalar(0.5);
  const c = document.createElement("canvas");
  c.width = 320; c.height = 56;
  const ctx = c.getContext("2d");
  ctx.fillStyle = "rgba(10,14,26,0.0)"; ctx.fillRect(0,0,c.width,c.height);
  ctx.fillStyle = DIM_COLOR.toString(16).padStart(6,"0");
  // Use the hex color string directly for canvas
  ctx.fillStyle = "#38bdf8";
  ctx.font = "700 24px 'Space Mono', monospace";
  ctx.textAlign = "center"; ctx.textBaseline = "middle";
  ctx.fillText(labelText, c.width/2, c.height/2);
  const tex = new THREE.CanvasTexture(c); tex.minFilter = THREE.LinearFilter;
  const sp = new THREE.Sprite(new THREE.SpriteMaterial({ map:tex, transparent:true, depthTest:false }));
  const ext = Math.max(0.1, modelExt);
  const sw = ext * 0.20;
  sp.scale.set(sw, sw * 0.175, 1);
  // Offset label slightly away from the model
  const labelOffset = axis === "x"
    ? new THREE.Vector3(0, -ext * 0.045, 0)
    : new THREE.Vector3(-ext * 0.055, 0, 0);
  sp.position.copy(mid).add(labelOffset);
  sp.renderOrder = 999;
  grp.add(sp);

  return grp;
}

// Section label: small billboard floating above the section center.
function makeSectionLabel(text, cx, sh, cz, modelExt) {
  const c = document.createElement("canvas");
  c.width = 256; c.height = 56;
  const x = c.getContext("2d");
  x.fillStyle = "rgba(10,14,26,0.82)"; x.fillRect(0,0,c.width,c.height);
  x.strokeStyle = "rgba(0,191,255,0.5)"; x.lineWidth = 1.5;
  x.strokeRect(1,1,c.width-2,c.height-2);
  x.fillStyle = "#90cdf4";
  x.font = "600 22px 'Space Mono', monospace";
  x.textAlign = "center"; x.textBaseline = "middle";
  x.fillText(text, c.width/2, c.height/2);
  const tex = new THREE.CanvasTexture(c); tex.minFilter = THREE.LinearFilter;
  const sp = new THREE.Sprite(new THREE.SpriteMaterial({ map:tex, transparent:true, depthTest:false }));
  const ext = Math.max(0.1, modelExt);
  const w = ext * 0.15;
  sp.scale.set(w, w * 0.22, 1);
  sp.position.set(cx, sh * 1.12 + ext * 0.04, cz);
  sp.renderOrder = 999;
  return sp;
}

// ── setModel ───────────────────────────────────────────────────────────
function setModel(m, opts = {}) {
  _currentModel = m;
  if (!opts.keepFlip) _flipped = false;

  // Update plate color from input every time we (re)render.
  const pc = readPlateColor();
  plateMat.color.setHex(pc);
  plateMat.emissive.setHex(pc);

  clear(modelGroup); clear(dimGroup);

  console.log("[Task12Viewer] setModel", {
    sections: m?.sections?.length || 0,
    plates:   m?.plates?.length || 0,
    totals:   m?.totals,
  });

  if (!m || !Array.isArray(m.sections) || m.sections.length === 0) {
    console.warn("[Task12Viewer] no sections — nothing to render");
    return;
  }

  // Parse and validate each section.
  // C++ section.size   = [length_x, width_y, height_z]
  // C++ section.origin = center of bottom face → origin.x = x_center, origin.y = -width/2
  //
  // We convert to viewer coords:
  //   vSize   = [length, height, depth]  = [size[0], size[2], size[1]]
  //   vOrigin = [x_left, y_floor, z_back] = [origin[0] - size[0]/2,  0,  0]

  // When flipped, reverse section order and mirror X positions so the
  // model looks like you walked around to the other side.
  const sourceSections = _flipped ? [...m.sections].reverse() : m.sections;

  const parsed = sourceSections.map((s, i) => {
    const fixVec = (v, fb) => {
      const out = [0,0,0];
      for (let k = 0; k < 3; k++) {
        const x = Number(v?.[k]);
        out[k] = Number.isFinite(x) ? x : fb;
      }
      return out;
    };
    const rawSize   = fixVec(s.size,   0.1);   // [length_x, width_y, height_z]
    const rawOrigin = fixVec(s.origin, 0.0);   // [x_center, y_center, z_floor=0]

    // Clamp non-positive model dimensions
    for (let k = 0; k < 3; k++) if (rawSize[k] <= 0) rawSize[k] = 0.05;

    // Remap to viewer coordinate system
    const size   = [rawSize[0], rawSize[2], rawSize[1]]; // [length, height, depth]
    const origin = [rawOrigin[0] - rawSize[0] / 2, 0, 0]; // [x_left, y_floor, z_back]

    return { i, size, origin };
  });

  // Pipe radius: ~1.25 cm for a real-scale model.
  // Scale up proportionally if the model is larger than expected.
  const totalLen = parsed.reduce((mx, p) => Math.max(mx, p.origin[0] + p.size[0]), 0.1);
  const radius = Math.max(0.010, Math.min(0.020, totalLen * 0.009));

  // Render each section frame.
  for (const { size, origin } of parsed) {
    modelGroup.add(makeSectionFrame(size, origin, radius));
  }

  // Render plates.
  // We need viewer-coord size/origin (same transform as in parsed[] above).
  for (const p of (m.plates || [])) {
    const sec = m.sections[p.section_id ?? 0];
    if (sec) {
      const fixVec = (v, fb) => {
        const out = [0,0,0];
        for (let k = 0; k < 3; k++) {
          const x = Number(v?.[k]);
          out[k] = Number.isFinite(x) ? x : fb;
        }
        return out;
      };
      const rawSize   = fixVec(sec.size,   0.1);
      const rawOrigin = fixVec(sec.origin, 0.0);
      for (let k = 0; k < 3; k++) if (rawSize[k] <= 0) rawSize[k] = 0.05;
      const vSize   = [rawSize[0], rawSize[2], rawSize[1]];
      const vOrigin = [rawOrigin[0] - rawSize[0] / 2, 0, 0];
      modelGroup.add(makePlate(p, vOrigin, vSize));
    }
  }

  // Section labels
  const bbox0 = new THREE.Box3().setFromObject(modelGroup);
  const ext0  = bbox0.isEmpty() ? 1 :
    Math.max(...bbox0.getSize(new THREE.Vector3()).toArray(), 0.1);

  const secLabels = _flipped ? ["Right", "Middle", "Left"] : ["Left", "Middle", "Right"];
  for (const { i, size, origin } of parsed) {
    const cx = origin[0] + size[0] / 2;
    const cz = origin[2] + size[2] / 2;
    const label = secLabels[i] ?? `§${i+1}`;
    dimGroup.add(makeSectionLabel(label, cx, origin[1] + size[1], cz, ext0));
  }

  // Frame camera to model
  const bbox = new THREE.Box3().setFromObject(modelGroup);
  if (bbox.isEmpty()) { dimGroup.visible = dimsVisible; return; }

  const bsize = bbox.getSize(new THREE.Vector3());
  const ctr   = bbox.getCenter(new THREE.Vector3());
  const ext   = Math.max(bsize.x, bsize.y, bsize.z, 0.1);

  camera.near = Math.max(0.001, ext * 0.001);
  camera.far  = Math.max(100,   ext * 60);
  camera.updateProjectionMatrix();

  // Floor grid at Y=0
  scene.remove(grid); grid.geometry?.dispose?.(); grid.material?.dispose?.();
  const gs = Math.max(4, ext * 2.5);
  grid = new THREE.GridHelper(gs, Math.round(gs / 0.1) || 20, 0x1a2840, 0x1a2840);
  grid.position.set(ctr.x, bbox.min.y, ctr.z);
  scene.add(grid);

  // Dimension lines — use actual section geometry (not bbox which includes pipe radius).
  const totalLength = parsed.reduce((s, p) => s + p.size[0], 0);
  const totalHeight = parsed.reduce((mx, p) => Math.max(mx, p.size[1]), 0);
  // Anchor lines to the actual model edges (x_left of first section, floor Y=0)
  const modelLeftX  = parsed.length ? parsed[0].origin[0] : bbox.min.x;
  const modelRightX = modelLeftX + totalLength;
  const frontZ      = bbox.max.z;
  const floorY      = 0;
  const leftX       = modelLeftX;

  const dimOffset = Math.max(0.03, ext * 0.055);  // gap between model and line

  dimGroup.add(makeDimLine(
    new THREE.Vector3(modelLeftX,  floorY - dimOffset, frontZ),
    new THREE.Vector3(modelRightX, floorY - dimOffset, frontZ),
    `L = ${(totalLength * 100).toFixed(1)} cm`,
    "x", ext
  ));
  dimGroup.add(makeDimLine(
    new THREE.Vector3(leftX - dimOffset, floorY, frontZ),
    new THREE.Vector3(leftX - dimOffset, totalHeight, frontZ),
    `H = ${(totalHeight * 100).toFixed(1)} cm`,
    "y", ext
  ));

  // Camera: head-on, slightly elevated, tighter zoom
  const d = ext * 1.4;
  controls.target.copy(ctr);
  camera.position.set(ctr.x, ctr.y + d*0.25, ctr.z + d*1.0);
  camera.lookAt(ctr);
  controls.update();

  console.log("[Task12Viewer] framed", {
    ext, ctr: ctr.toArray(),
    cam: camera.position.toArray(),
  });

  dimGroup.visible = dimsVisible;
}

function resetView(altKey) {
  if (altKey) {
    // Option+reset: replace 3D model with canonical defaults but keep
    // detection data (plates, per-pair) from the current model intact.
    _flipped = false;
    const base = _currentModel || {};
    const merged = Object.assign({}, base, {
      sections: DEFAULT_MODEL.sections,
      totals:   DEFAULT_MODEL.totals,
      plates:   base.plates || [],
      // keep existing scale source/confidence — only geometry is being reset
    });
    setModel(merged);
    if (window._task12NotifyStats) window._task12NotifyStats(merged);
    return;
  }
  const bbox = new THREE.Box3().setFromObject(modelGroup);
  if (bbox.isEmpty()) return;
  const bsize = bbox.getSize(new THREE.Vector3());
  const ctr   = bbox.getCenter(new THREE.Vector3());
  const ext   = Math.max(bsize.x, bsize.y, bsize.z, 0.1);
  const d = ext * 1.4;
  controls.target.copy(ctr);
  camera.position.set(ctr.x, ctr.y + d*0.25, ctr.z + d*1.0);
  camera.lookAt(ctr);
  controls.update();
}

function flipSections() {
  if (!_currentModel || !Array.isArray(_currentModel.sections)) return;
  // Swap section[0] and section[2] sizes in-place on the current model,
  // then re-render and notify technicals. Only length+height swap; depth
  // (width_y / size[1]) stays fixed at 0.36 m across all sections.
  const secs = _currentModel.sections;
  if (secs.length < 3) return;

  const swapSize = (a, b) => {
    // size = [length_x, width_y, height_z] — swap [0] and [2] only
    [a[0], b[0]] = [b[0], a[0]];
    [a[2], b[2]] = [b[2], a[2]];
  };

  const s0 = Array.isArray(secs[0].size) ? secs[0].size : [secs[0].size?.x||0.3, secs[0].size?.y||0.36, secs[0].size?.z||0.25];
  const s2 = Array.isArray(secs[2].size) ? secs[2].size : [secs[2].size?.x||0.3, secs[2].size?.y||0.36, secs[2].size?.z||0.40];
  swapSize(s0, s2);
  secs[0].size = s0;
  secs[2].size = s2;

  // Recompute section origins so sections stay contiguous after the size swap
  let xCursor = 0;
  secs.forEach(sec => {
    const sz = Array.isArray(sec.size) ? sec.size : [sec.size?.x||0, sec.size?.y||0, sec.size?.z||0];
    const w = sz[0] ?? 0;
    if (Array.isArray(sec.origin)) sec.origin[0] = xCursor + w / 2;
    else if (sec.origin) sec.origin.x = xCursor + w / 2;
    xCursor += w;
  });

  // Recompute totals after swap
  if (_currentModel.totals) {
    _currentModel.totals.length = secs.reduce((s, sec) => {
      const sz = Array.isArray(sec.size) ? sec.size : [sec.size?.x||0];
      return s + (sz[0] ?? 0);
    }, 0);
    _currentModel.totals.height = Math.max(...secs.map(s => {
      const sz = Array.isArray(s.size) ? s.size : [0,0,s.size?.z||0];
      return sz[2] || 0;
    }));
  }

  setModel(_currentModel, { keepFlip: true });
  if (window._task12NotifyStats) window._task12NotifyStats(_currentModel);
}

function toggleDimensions() {
  dimsVisible = !dimsVisible;
  dimGroup.visible = dimsVisible;
}

function exportGLB() {
  if (!THREE.GLTFExporter) return alert("GLTFExporter not loaded.");
  new THREE.GLTFExporter().parse(
    modelGroup,
    (buf) => {
      const blob = new Blob([buf], { type: "model/gltf-binary" });
      const a = document.createElement("a");
      a.href = URL.createObjectURL(blob); a.download = "coral_garden.glb"; a.click();
    },
    (err) => { console.error("GLTFExporter error", err); alert("GLB export failed: " + err); },
    { binary: true }
  );
}

function exportOBJ() {
  if (!THREE.OBJExporter) return alert("OBJExporter not loaded.");
  const text = new THREE.OBJExporter().parse(modelGroup);
  const blob = new Blob([text], { type: "text/plain" });
  const a = document.createElement("a");
  a.href = URL.createObjectURL(blob); a.download = "coral_garden.obj"; a.click();
}

window.Task12Viewer = { setModel, resetView, toggleDimensions, exportGLB, exportOBJ, flipSections };
})();
