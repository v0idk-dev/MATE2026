// 1_2.js — Task 1.2 front-end controller.
(() => {
"use strict";

const $ = (id) => document.getElementById(id);
const API = "/api/task1_2";

// ── Image stack ───────────────────────────────────────────────────────────────
// Each entry: { id, urlL, urlR, fileL, fileR, label }
const _stack = [];
let _stackActive = null;  // id of currently loaded pair

function stackAdd(fileL, fileR, urlL, urlR, label) {
  const id = Date.now();
  _stack.push({ id, urlL, urlR, fileL, fileR, label });
  stackRender();
  stackActivate(id);
}

function stackRemove(id) {
  const idx = _stack.findIndex(e => e.id === id);
  if (idx === -1) return;
  _stack.splice(idx, 1);
  if (_stackActive === id) {
    const next = _stack[idx] || _stack[idx - 1];
    if (next) stackActivate(next.id);
    else _stackActive = null;
  }
  stackRender();
}

function stackActivate(id) {
  const entry = _stack.find(e => e.id === id);
  if (!entry) return;
  _stackActive = id;
  files.left  = entry.fileL;
  files.right = entry.fileR;
  $("left-preview").src  = entry.urlL;  $("left-upload").classList.add("has-image");
  $("right-preview").src = entry.urlR;  $("right-upload").classList.add("has-image");
  vizImage("uploaded", 0, entry.urlL, "Left Camera");
  vizImage("uploaded", 1, entry.urlR, "Right Camera");
  refreshAnalyzeEnabled();
  stackRender();
}

function stackRender() {
  const bar = $("pg-stack-bar");
  if (!bar) return;
  bar.style.display = _stack.length ? "flex" : "none";
  bar.innerHTML = "";
  _stack.forEach((e, i) => {
    const item = document.createElement("div");
    item.className = "pg-stack-item" + (e.id === _stackActive ? " pg-stack-active" : "");

    const img = document.createElement("img");
    img.src = e.urlL; img.alt = e.label;
    // Animate in new items
    img.style.opacity = "0";
    img.style.transform = "scale(0.85)";
    img.style.transition = "opacity 0.2s, transform 0.2s";
    requestAnimationFrame(() => {
      img.style.opacity = "1"; img.style.transform = "scale(1)";
    });

    const del = document.createElement("div");
    del.className = "pg-stack-del"; del.textContent = "×";
    del.addEventListener("click", (ev) => { ev.stopPropagation(); stackRemove(e.id); });

    const lbl = document.createElement("div");
    lbl.className = "pg-stack-label"; lbl.textContent = e.label || `Pair ${i+1}`;

    item.appendChild(img); item.appendChild(del); item.appendChild(lbl);
    item.addEventListener("click", () => stackActivate(e.id));
    bar.appendChild(item);
  });
}

// ── AI model override modal ───────────────────────────────────────────────────
let _aiOverrideResolve = null;

function openAIModal() {
  return new Promise(resolve => {
    _aiOverrideResolve = resolve;
    const sel = $("pg-ai-model-select");
    sel.innerHTML = "";
    const addOpt = (val, txt) => {
      const o = document.createElement("option"); o.value = val; o.textContent = txt;
      sel.appendChild(o);
    };
    if (_aiProviders.openai) {
      addOpt("openai:gpt-5.5-pro",  "OpenAI · gpt-5.5-pro");
      addOpt("openai:gpt-5.5",      "OpenAI · gpt-5.5");
      addOpt("openai:gpt-5.4",      "OpenAI · gpt-5.4");
      addOpt("openai:gpt-5.4-mini", "OpenAI · gpt-5.4-mini");
    }
    if (_aiProviders.anthropic) {
      addOpt("anthropic:claude-opus-4-7",   "Anthropic · claude-opus-4-7");
      addOpt("anthropic:claude-sonnet-4-6", "Anthropic · claude-sonnet-4-6");
      addOpt("anthropic:claude-haiku-4-5",  "Anthropic · claude-haiku-4-5");
    }
    if (_aiProviders.google) {
      addOpt("google:gemini-3.1-pro-preview",  "Google · gemini-3.1-pro");
      addOpt("google:gemini-3-flash-preview",  "Google · gemini-3-flash");
      addOpt("google:gemini-2.5-pro",          "Google · gemini-2.5-pro");
      addOpt("google:gemini-2.5-flash",        "Google · gemini-2.5-flash");
    }
    if (_aiProviders.appleIntelligence) addOpt("apple:on-device", "Apple Intelligence");
    if (!sel.options.length) { resolve(null); return; }
    // Pre-select the settings default if it's in the list
    const def = (window._settingsPhoto || {}).defaultAIModel || "";
    if (def && [...sel.options].some(o => o.value === def)) sel.value = def;
    $("pg-ai-modal").classList.remove("pg-modal-hidden");
  });
}

function closeAIModal(result) {
  $("pg-ai-modal").classList.add("pg-modal-hidden");
  if (_aiOverrideResolve) { _aiOverrideResolve(result); _aiOverrideResolve = null; }
}

// ── helpers ──────────────────────────────────────────────────────────────────
function hexToHsvHue(hex) {
  hex = (hex || "#8000ff").replace("#", "");
  const r = parseInt(hex.slice(0,2),16)/255;
  const g = parseInt(hex.slice(2,4),16)/255;
  const b = parseInt(hex.slice(4,6),16)/255;
  const max = Math.max(r,g,b), min = Math.min(r,g,b);
  let h = 0;
  if (max === min) h = 0;
  else if (max === r) h = ((g-b)/(max-min)) * 60;
  else if (max === g) h = ((b-r)/(max-min) + 2) * 60;
  else h = ((r-g)/(max-min) + 4) * 60;
  if (h < 0) h += 360;
  return Math.round(h / 2);
}

function fmt(x, n=3) { return (x==null||isNaN(x))?"—":Number(x).toFixed(n); }

// ── Load settings defaults on page load ──────────────────────────────────────
async function loadSettingsDefaults() {
  let photo = {};
  try {
    const r = await fetch("/api/settings/photo");
    if (r.ok) photo = await r.json();
  } catch (_) {}

  // Plate color
  const R = Math.round(photo.plateColorR ?? 128);
  const G = Math.round(photo.plateColorG ?? 0);
  const B = Math.round(photo.plateColorB ?? 255);
  $("plate-color").value = "#" +
    R.toString(16).padStart(2,"0") +
    G.toString(16).padStart(2,"0") +
    B.toString(16).padStart(2,"0");

  // Baseline
  const bl = parseFloat(photo.baseline);
  if (bl > 0) $("baseline-m").value = bl.toFixed(3);

  // Plate dimensions
  const pw = parseFloat(photo.plateW);
  if (pw > 0) $("plate-w").value = pw.toFixed(3);
  const ph = parseFloat(photo.plateH);
  if (ph > 0) $("plate-h").value = ph.toFixed(3);

  // Plate count — Swift writes it as expectedPlateCount
  const pc = parseInt(photo.expectedPlateCount ?? photo.plateCount);
  if (pc > 0) $("plate-count").value = pc;

  // Underwater toggle
  $("underwater").checked = !!(photo.underwater);

  // Cache for bestAIProvider
  window._settingsPhoto = photo;

  // Show calibration status inside the <details>
  await refreshCalibStatus();
}

async function refreshCalibStatus() {
  const el = $("pg-calib-status");
  if (!el) return;
  try {
    const r = await fetch("/api/task1_2/calibrations");
    if (!r.ok) return;
    const c = await r.json();
    const hasLeft   = !!c.photogrammetryLeftCalibId;
    const hasRight  = !!c.photogrammetryRightCalibId;
    const hasStereo = !!c.activeStereoExtrinsicsId;
    if (hasLeft && hasRight && hasStereo) {
      el.className = "pg-calib-status ok";
      el.textContent = "✓ Left + right calibrations and stereo extrinsics loaded from Settings.";
    } else if (hasLeft || hasRight || hasStereo) {
      el.className = "pg-calib-status warn";
      const missing = [!hasLeft && "left calib", !hasRight && "right calib", !hasStereo && "stereo extrinsics"].filter(Boolean).join(", ");
      el.textContent = `Partial: missing ${missing} — configure in Settings › Photogrammetry.`;
    } else {
      el.className = "pg-calib-status";
      el.textContent = "No calibrations in Settings — Manhattan auto-calib will be used.";
    }
  } catch (_) {}
}

// ── AI provider detection ─────────────────────────────────────────────────────
let _aiProviders = {};
async function loadAIProviders() {
  try {
    const r = await fetch("/api/task1_2/ai_providers");
    if (r.ok) _aiProviders = await r.json();
  } catch (_) {}
}

function bestAIProvider() {
  // Returns {provider, model} from settings defaultAIModel, or first available
  let photo = {};
  try {
    // We already loaded photo defaults; re-read from the inputs isn't needed.
    // Instead use the cached _settingsPhoto set at load time.
    photo = window._settingsPhoto || {};
  } catch (_) {}
  const def = photo.defaultAIModel || "";
  if (def) {
    const [prov, ...rest] = def.split(":");
    return { provider: prov, model: rest.join(":") };
  }
  // Fallback: pick first configured provider
  for (const [prov, has] of Object.entries(_aiProviders)) {
    if (has && prov !== "appleIntelligence") return { provider: prov, model: "" };
  }
  if (_aiProviders.appleIntelligence) return { provider: "apple", model: "on-device" };
  return null;
}

// ── Image uploads ─────────────────────────────────────────────────────────────
const files = { left: null, right: null };
function wireUpload(side) {
  const input = $(`${side}-image`);
  const card  = $(`${side}-upload`);
  const prev  = $(`${side}-preview`);
  input.addEventListener("change", (e) => {
    const f = e.target.files && e.target.files[0];
    if (!f) return;
    files[side] = f;
    const url = URL.createObjectURL(f);
    prev.src = url;
    card.classList.add("has-image");
    vizImage("uploaded", side === "left" ? 0 : 1, url, side === "left" ? "Left Camera" : "Right Camera");
    refreshAnalyzeEnabled();
  });
  prev.addEventListener("click", () => input.click());
}
wireUpload("left"); wireUpload("right");

function refreshAnalyzeEnabled() {
  $("analyze-btn").disabled = !(files.left && files.right);
}

// ── Analyze ───────────────────────────────────────────────────────────────────
const state = { jobId:null, model:null, exports:null, lastConfig:null };

async function analyze({ enhance=false, aiOverride=null } = {}) {
  if (!files.left || !files.right) return;

  const fd = new FormData();
  fd.append("lefts[]",  files.left);
  fd.append("rights[]", files.right);

  const cfg = {
    plate_color_hex:  $("plate-color").value,
    target_hue:       hexToHsvHue($("plate-color").value),
    hue_tol:          25,
    plate_side_m:     (+$("plate-w").value + +$("plate-h").value) / 2 || 0.10,
    expected_plates:  +$("plate-count").value || 8,
    baseline_m:       +$("baseline-m").value || 0.10,
    rig_baseline_m:   +$("baseline-m").value || 0.10,
    underwater:       $("underwater").checked,
    water_n:          1.34,
    use_plate_pnp:    true,
    use_bundle_adjust: true,
    use_apple_intelligence: enhance && !!_aiProviders.appleIntelligence,
    ai_enhance:       enhance,
  };

  if (enhance) {
    const ai = aiOverride ? { provider: aiOverride.split(":")[0], model: aiOverride.split(":").slice(1).join(":") }
                          : bestAIProvider();
    if (ai) {
      cfg.ai_provider = ai.provider;
      if (ai.model) cfg.ai_model = ai.model;
    }
  }

  state.lastConfig = cfg;
  fd.append("config", JSON.stringify(cfg));

  showLoading(true, enhance ? "Enhancing…" : "Running pipeline…");
  $("error-section").style.display = "none";
  $("analyze-btn").disabled = true;

  try {
    const r = await fetch(`${API}/analyze`, { method:"POST", body:fd });
    const j = await r.json();
    if (!r.ok) {
      const detail = j.stderr ? `${j.error || "Pipeline error"}\n\n${j.stderr}` : (j.error || `HTTP ${r.status}`);
      throw new Error(detail);
    }
    state.jobId   = j.job_id;
    state.model   = j.model;
    state.exports = j.exports;

    if (j.job_id && j.debug_files) populateVizTabs(j.job_id, j.debug_files);
    onModelLoaded(j);
  } catch (e) {
    showError(e.message);
  } finally {
    showLoading(false);
    refreshAnalyzeEnabled();
  }
}

function populateVizTabs(jobId, debugFiles) {
  const byTag = {};
  for (const name of debugFiles) {
    const m = name.match(/^pair(\d+)_(.+)\.jpg$/);
    if (m) { const tag = m[2]; (byTag[tag] = byTag[tag] || []).push(name); }
  }

  function renderPair(panel, lFiles, rFiles, emptyMsg) {
    panel.innerHTML = "";
    if (!lFiles.length && !rFiles.length) {
      panel.innerHTML = `<div class="pg-viz-empty">${emptyMsg}</div>`; return;
    }
    const grid = document.createElement("div");
    grid.className = "pg-viz-grid";
    const pairs = Math.max(lFiles.length, rFiles.length);
    for (let i = 0; i < pairs; i++) {
      for (const [flist, side] of [[lFiles, "Left"], [rFiles, "Right"]]) {
        if (!flist[i]) continue;
        const wrap = document.createElement("div");
        const lbl  = document.createElement("div");
        lbl.className = "pg-viz-label";
        lbl.textContent = `Pair ${i} — ${side}`;
        const img = document.createElement("img");
        img.src = `/api/task1_2/debug/${jobId}/${flist[i]}`;
        img.alt = side;
        wrap.appendChild(lbl); wrap.appendChild(img);
        grid.appendChild(wrap);
      }
    }
    panel.appendChild(grid);
  }

  renderPair($("panel-detections"),
    byTag["undistL"] || [], byTag["undistR"] || [],
    "No undistorted images — pipeline may not have run undistortion step.");

  renderPair($("panel-epipolar"),
    byTag["rectL"] || [], byTag["rectR"] || [],
    "No rectified images — check calibration in Settings › Photogrammetry.");

  // Merge unrectified + rectified plate/pipe overlays into one grid per tab
  const platesL = [...(byTag["platesL"] || []), ...(byTag["rectPlatesL"] || [])];
  const platesR = [...(byTag["platesR"] || []), ...(byTag["rectPlatesR"] || [])];
  renderPair($("panel-plates"), platesL, platesR,
    "No plate detection overlays — run Analyze first.");

  const pipesL = [...(byTag["pipesL"] || []), ...(byTag["rectPipesL"] || [])];
  const pipesR = [...(byTag["pipesR"] || []), ...(byTag["rectPipesR"] || [])];
  renderPair($("panel-pipes"), pipesL, pipesR,
    "No pipe detection overlays — run Analyze first.");
}

// ── vizImage: place an image in the uploaded tab ─────────────────────────────
function vizImage(tab, idx, url, label) {
  const p = $(`panel-${tab}`);
  let grid = p.querySelector(".pg-viz-grid");
  if (!grid) { grid = document.createElement("div"); grid.className = "pg-viz-grid"; p.appendChild(grid); }
  // find or create slot by data-idx
  let slot = grid.querySelector(`[data-idx="${idx}"]`);
  if (!slot) {
    slot = document.createElement("div");
    slot.dataset.idx = idx;
    grid.appendChild(slot);
  }
  slot.innerHTML = `<div class="pg-viz-label">${label}</div><img alt="${label}">`;
  slot.querySelector("img").src = url;
}

// ── onModelLoaded ─────────────────────────────────────────────────────────────
function onModelLoaded(j) {
  $("results-section").style.display = "block";

  const tail = (j.stderr || "").trim();
  $("raw-output").textContent =
    JSON.stringify(j.model, null, 2) +
    (tail ? "\n\n— binary stderr —\n" + tail : "");

  populateStats(j.model, j);
  if (window.Task12Viewer) window.Task12Viewer.setModel(j.model);
}

// ── populateStats ─────────────────────────────────────────────────────────────
function populateStats(m, j) {
  if (!m) return;
  const t  = m.totals  || {};
  const sc = m.scale   || {};
  const pp = Array.isArray(m._per_pair) ? m._per_pair : (Array.isArray(j?.per_pair) ? j.per_pair : []);
  // stash for scale operations
  m._per_pair = pp;

  const sumPL = pp.reduce((s, d) => s + (d.n_plates_left  || 0), 0);
  const sumPR = pp.reduce((s, d) => s + (d.n_plates_right || 0), 0);
  const detected = pp.length
    ? `L:${sumPL} R:${sumPR} plates (${pp.length} pair${pp.length>1?"s":""})`
    : "—";

  const sects = m.sections || [];
  const secNames = ["Left", "Middle", "Right"];
  const secRows = sects.map((s, i) => {
    const sz = Array.isArray(s.size) ? s.size : [s.size?.x, s.size?.y, s.size?.z];
    return [`  ${secNames[i] || "Sec "+i} (L × H)`,
            `${fmt((sz[0]||0)*100,1)} × ${fmt((sz[2]||0)*100,1)} cm`];
  });

  const rows = [
    ["Total length",       fmt((t.length||0),3) + " m  (" + fmt((t.length||0)*100,1) + " cm)"],
    ["Frame depth",        "36.0 cm (fixed)"],
    ["Total height",       fmt((t.height||0),3) + " m  (" + fmt((t.height||0)*100,1) + " cm)"],
    ...secRows,
    ["Pairs used",         m.n_pairs_used ?? "—"],
    ["Detected per-frame", detected],
    ["Plates in model",    (m.plates || []).length],
    ["Baseline (input)",   state.lastConfig ? fmt(state.lastConfig.baseline_m,3) + " m" : "—"],
    ["Scale source",       sc.source || "plate-prior PnP"],
    ["Scale confidence",   sc.confidence != null ? fmt(sc.confidence*100,0) + "%" : "—"],
  ];

  const host = $("pg-stats");
  host.innerHTML = "";
  rows.forEach(([k,v]) => {
    const r = document.createElement("div");
    r.className = "stat-row";
    r.innerHTML = `<span class="stat-label">${k}</span><span class="stat-value">${v}</span>`;
    host.appendChild(r);
  });

  // Collect all warnings from multiple sources into one deduplicated block
  const warns = [];
  const addWarn = (msg, kind) => {
    if (!msg) return;
    const trimmed = String(msg).trim();
    if (!trimmed) return;
    // Split on ·  separator used by old code
    for (const part of trimmed.split(/\s+·\s+/)) {
      const p = part.trim();
      if (p && !warns.some(w => w.text === p)) warns.push({ text: p, kind });
    }
  };

  if (m.warning) addWarn(m.warning, "warn");
  // Surface rectify failures from stderr
  if (j?.stderr) {
    const match = (j.stderr).match(/rectifyStereoPair:[^\n]+/);
    if (match) addWarn(match[0].trim(), "bad");
  }
  // Plate count zero → color hint
  if (pp.length && sumPL === 0 && sumPR === 0) {
    addWarn(
      `No plates detected. Plate Color (${$("plate-color").value}) may not match actual plate hue — adjust and re-run.`,
      "warn"
    );
  }

  if (warns.length) {
    const r = document.createElement("div");
    r.className = "stat-row " + (warns.some(w=>w.kind==="bad") ? "bad" : "warn");
    const combined = warns.map(w => w.text).join("\n");
    r.innerHTML = `<span class="stat-label">Warning</span><span class="stat-value">${combined}</span>`;
    host.appendChild(r);
  }
}

// ── Tabs ───────────────────────────────────────────────────────────────────────
document.querySelectorAll(".tab").forEach(t => t.addEventListener("click", () => {
  document.querySelectorAll(".tab").forEach(x => x.classList.remove("active"));
  document.querySelectorAll(".tab-panel").forEach(x => x.classList.remove("active"));
  t.classList.add("active");
  $(`panel-${t.dataset.tab}`).classList.add("active");
}));

// ── Loading / Error ───────────────────────────────────────────────────────────
function showLoading(yes, sub) {
  $("loading-section").style.display = yes ? "block" : "none";
  if (sub) $("loading-sub-text").textContent = sub;
}
function showError(msg) {
  $("error-section").style.display = "block";
  // Show the first line prominently; if there's stderr detail, put it in a <pre>
  const lines = (msg || "Unknown error").split("\n");
  const first = lines[0];
  const rest  = lines.slice(1).join("\n").trim();
  const el = $("error-message");
  if (rest) {
    el.innerHTML = "";
    const p = document.createElement("span"); p.textContent = first; el.appendChild(p);
    const pre = document.createElement("pre");
    pre.style.cssText = "margin-top:0.5rem;font-size:0.62rem;white-space:pre-wrap;opacity:0.75;max-height:200px;overflow:auto;-webkit-user-select:text;user-select:text";
    pre.textContent = rest; el.appendChild(pre);
  } else {
    el.textContent = first;
  }
}

// ── Scale modal ───────────────────────────────────────────────────────────────
let _scaleMode = "length";

function openScaleModal(mode) {
  _scaleMode = mode;
  const input = $("pg-scale-input");
  if (mode === "height") {
    $("pg-scale-modal-title").textContent = "Set Real Height";
    $("pg-scale-modal-label").textContent = "Total height (cm)";
    const cur = state.model?.totals?.height;
    input.value = cur ? (cur * 100).toFixed(1) : "55";
    $("pg-scale-modal-hint").textContent = "Squishes section heights — length/depth unchanged.";
  } else {
    $("pg-scale-modal-title").textContent = "Set Real Length";
    $("pg-scale-modal-label").textContent = "Total length (cm)";
    const cur = state.model?.totals?.length;
    input.value = cur ? (cur * 100).toFixed(1) : "100";
    $("pg-scale-modal-hint").textContent = "Uniform scale — all ratios preserved, depth stays 36 cm.";
  }
  $("pg-scale-modal").classList.remove("pg-modal-hidden");
  input.focus(); input.select();
}

function closeScaleModal() {
  $("pg-scale-modal").classList.add("pg-modal-hidden");
}

function applyScale() {
  if (!state.model) { closeScaleModal(); return; }
  const val = parseFloat($("pg-scale-input").value);
  if (!isFinite(val) || val <= 0) { $("pg-scale-input").focus(); return; }
  closeScaleModal();

  if (_scaleMode === "height") {
    const hM = val / 100;
    const curH = state.model.totals?.height;
    if (!curH || curH <= 1e-6) return;
    const k = hM / curH;
    (state.model.sections || []).forEach(s => { if (s.size) s.size[2] *= k; });
    state.model.totals.height = hM;
    state.model.scale = { source:"manual height", confidence:1.0 };
  } else {
    const lM = val / 100;
    const curL = state.model.totals?.length;
    if (!curL || curL <= 1e-6) return;
    const k = lM / curL;
    (state.model.sections || []).forEach(s => {
      if (s.size) { s.size[0] *= k; s.size[2] *= k; } // size[1] is width — not scaled
      if (s.origin) s.origin = s.origin.map(v => v * k);
    });
    (state.model.plates || []).forEach(p => {
      // side_m is a known physical constant — not scaled
      if (p.corners) p.corners = p.corners.map(c => c.map(v => v * k));
    });
    // totals.width is the known spec dimension — not scaled
    if (state.model.totals.length != null) state.model.totals.length = lM;
    if (state.model.totals.height != null) state.model.totals.height *= k;
    state.model.scale = { source:"manual length", confidence:1.0 };
  }
  onModelLoaded({ model: state.model });
}

$("pg-real-width").addEventListener("click", (e) => {
  if (!state.model) return;
  openScaleModal(e.altKey ? "height" : "length");
});
$("pg-scale-confirm").addEventListener("click", applyScale);
$("pg-scale-cancel").addEventListener("click", closeScaleModal);
$("pg-scale-modal").addEventListener("click", (e) => {
  if (e.target === $("pg-scale-modal")) closeScaleModal();
});
$("pg-scale-input").addEventListener("keydown", (e) => {
  if (e.key === "Enter") applyScale();
  if (e.key === "Escape") closeScaleModal();
});

// ── AI modal buttons ──────────────────────────────────────────────────────────
$("pg-ai-confirm").addEventListener("click", () => {
  closeAIModal($("pg-ai-model-select").value || null);
});
$("pg-ai-cancel").addEventListener("click", () => closeAIModal(null));
$("pg-ai-modal").addEventListener("click", (e) => {
  if (e.target === $("pg-ai-modal")) closeAIModal(null);
});

// ── Buttons ───────────────────────────────────────────────────────────────────
$("analyze-btn").addEventListener("click", () => analyze());

$("pg-enhance").addEventListener("click", async (e) => {
  if (!state.model) return;
  const override = e.altKey ? await openAIModal() : null;
  if (e.altKey && !override) return;
  const ai = override
    ? { provider: override.split(":")[0], model: override.split(":").slice(1).join(":") }
    : bestAIProvider();
  if (!ai) { showError("No enhancement model configured. Check Settings."); return; }
  await enhance(ai.provider, ai.model);
});

async function enhance(provider, model) {
  const btn = $("pg-enhance");
  btn.disabled = true;
  btn.dataset.origText = btn.textContent;
  btn.innerHTML = 'Enhancing...';

  try {
    // Collect images as data URLs (up to 3 pairs from stack + current)
    const images = [];
    const toDataUrl = (file) => new Promise((res, rej) => {
      const r = new FileReader();
      r.onload = e => res({ label: file.name, data_url: e.target.result });
      r.onerror = rej;
      r.readAsDataURL(file);
    });

    const pairs = _stack.slice(0, 3);
    if (pairs.length === 0 && files.left) pairs.push({ fileL: files.left, fileR: files.right, label: "current" });
    for (const p of pairs) {
      if (p.fileL) images.push(await toDataUrl(p.fileL));
      if (p.fileR) images.push(await toDataUrl(p.fileR));
    }

    const r = await fetch(`${API}/enhance`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ provider, model, model_json: state.model, images }),
    });
    const j = await r.json();
    if (!r.ok) throw new Error(j.error || `HTTP ${r.status}`);

    // j.sizes = [[Lx,Hz],[Lx,Hz],[Lx,Hz]] — update only width and height per section
    if (!Array.isArray(j.sizes) || j.sizes.length !== 3) throw new Error("Unexpected response structure");

    j.sizes.forEach(([lx, hz], i) => {
      if (state.model.sections[i]?.size) {
        state.model.sections[i].size[0] = lx;
        state.model.sections[i].size[2] = hz;
      }
    });
    // Recompute section origins so sections stay contiguous after size changes.
    // origin[0] = x_center of section = sum of previous widths + half this width.
    let xCursor = 0;
    state.model.sections.forEach(sec => {
      const w = sec.size?.[0] ?? 0;
      sec.origin[0] = xCursor + w / 2;
      xCursor += w;
    });
    // Recompute totals
    const secs = state.model.sections;
    if (state.model.totals) {
      state.model.totals.length = secs.reduce((s, sec) => s + (sec.size?.[0] ?? 0), 0);
      state.model.totals.height = Math.max(...secs.map(sec => sec.size?.[2] ?? 0));
    }
    state.model.scale = { source: "enhanced", confidence: 0.5 };

    if (window.Task12Viewer) window.Task12Viewer.setModel(state.model);
    populateStats(state.model, { per_pair: state.model._per_pair || [] });
  } catch(e) {
    console.error("Enhance failed:", e.message);
    btn.style.transition = "background 0.4s, color 0.4s";
    btn.style.background = "rgba(255, 0, 0, 0.3)";
    setTimeout(() => { btn.style.background = ""; btn.style.transition = ""; }, 900);
  } finally {
    btn.disabled = false;
    btn.textContent = btn.dataset.origText || "Enhance";
  }
}

$("pg-reset-view").addEventListener("click", (e) => {
  if (!window.Task12Viewer) return;
  if (e.altKey) {
    window.Task12Viewer.resetView(true);
  } else {
    window.Task12Viewer.resetView(false);
  }
});
$("pg-toggle-dimensions").addEventListener("click", (e) => {
  if (!window.Task12Viewer) return;
  if (e.altKey) {
    window.Task12Viewer.flipSections();
  } else {
    window.Task12Viewer.toggleDimensions();
  }
});

// Screenshot — adds to stack instead of replacing
$("pg-screenshot-btn").addEventListener("click", async () => {
  let channelL = "CH03", channelR = "CH04";
  try {
    const r = await fetch("/api/cameras_config");
    if (r.ok) {
      const cfg = await r.json();
      channelL = cfg.screenshots?.photogrammetryLeft?.channel  || channelL;
      channelR = cfg.screenshots?.photogrammetryRight?.channel || channelR;
    }
  } catch (_) {}
  try {
    const [resL, resR] = await Promise.all([
      fetch(`/api/camera/screenshot/${channelL}`),
      fetch(`/api/camera/screenshot/${channelR}`),
    ]);
    if (!resL.ok || !resR.ok) { showError("No camera frame available."); return; }
    const [blobL, blobR] = await Promise.all([resL.blob(), resR.blob()]);
    const ts = new Date().toLocaleTimeString("en-US", { hour12:false, hour:"2-digit", minute:"2-digit", second:"2-digit" });
    const label = `${ts}`;
    const fL = new File([blobL], `shot_${Date.now()}_L.jpg`, { type:"image/jpeg" });
    const fR = new File([blobR], `shot_${Date.now()}_R.jpg`, { type:"image/jpeg" });
    const urlL = URL.createObjectURL(fL);
    const urlR = URL.createObjectURL(fR);
    stackAdd(fL, fR, urlL, urlR, label);
  } catch (e) { showError("Screenshot failed: " + e.message); }
});

// Exports
$("pg-export-json").addEventListener("click", () => {
  if (!state.model) return;
  const blob = new Blob([JSON.stringify(state.model, null, 2)], { type:"application/json" });
  download(blob, "coral_garden.json");
});
$("pg-export-glb").addEventListener("click", () => {
  // Always export the Three.js wireframe model (not the C++ solid-box GLB).
  if (window.Task12Viewer) window.Task12Viewer.exportGLB();
});

// Upload JSON re-import
$("upload-json-btn").addEventListener("click", () => $("upload-json-input").click());
$("upload-json-input").addEventListener("change", async (e) => {
  const f = e.target.files[0]; if (!f) return;
  try {
    const txt = await f.text();
    const r = await fetch(`${API}/import`, {
      method:"POST", headers:{"Content-Type":"application/json"}, body:txt
    });
    const j = await r.json();
    if (!r.ok) throw new Error(j.error || `HTTP ${r.status}`);
    state.model = j.model; state.exports = null;
    onModelLoaded({ model: j.model });
  } catch (err) { showError("Import failed: " + err.message); }
});

// Raw output toggle
$("raw-header").addEventListener("click", () => {
  const out = $("raw-output"), tog = $("raw-toggle");
  const open = out.style.display !== "none";
  out.style.display = open ? "none" : "block";
  tog.textContent = open ? "+" : "−";
});

function download(blob, name) {
  const a = document.createElement("a");
  a.href = URL.createObjectURL(blob); a.download = name;
  document.body.appendChild(a); a.click();
  setTimeout(() => { URL.revokeObjectURL(a.href); a.remove(); }, 100);
}

// ── Viewer callbacks for flip / default-reset ─────────────────────────────────
// Only refreshes the stats panel — does not touch detection data or raw output.
window._task12NotifyStats = (model) => {
  populateStats(model, { per_pair: (state.model?._per_pair) || [] });
};

// ── Boot ──────────────────────────────────────────────────────────────────────
(async () => {
  await Promise.all([loadSettingsDefaults(), loadAIProviders()]);
  // Cache photo settings for bestAIProvider()
  try {
    const r = await fetch("/api/settings/photo");
    if (r.ok) window._settingsPhoto = await r.json();
  } catch (_) {}
  refreshAnalyzeEnabled();
})();

})();
