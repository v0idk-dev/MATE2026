const { app, BrowserWindow, Menu, ipcMain, nativeImage, nativeTheme } = require('electron');
const { spawn, spawnSync, exec }  = require('child_process');
const http  = require('http');
const https = require('https');
const path  = require('path');
const fs    = require('fs');

const _pkg = (() => {
    try { return JSON.parse(fs.readFileSync(path.join(__dirname, 'package.json'), 'utf8')); }
    catch { return {}; }
})();

// electron-builder strips the "build" key from the bundled package.json,
// so these values must be hardcoded here and kept in sync with package.json.
const _APP_PRODUCT_NAME = 'MATE 2026 Robot Controller';
const _APP_COPYRIGHT    = 'Copyright © 2026 Dogukan Koc';

app.name = _APP_PRODUCT_NAME;

// Wipe Chromium's persisted per-origin zoom levels before any window opens
try {
    const _prefPath = require('path').join(app.getPath('userData'), 'Preferences');
    const _prefData = JSON.parse(require('fs').readFileSync(_prefPath, 'utf8'));
    if (_prefData.partition && _prefData.partition.per_host_zoom_levels) {
        delete _prefData.partition.per_host_zoom_levels;
        require('fs').writeFileSync(_prefPath, JSON.stringify(_prefData));
    }
} catch (_) {}

let mainWindow;
let _devToolsWin = null;
let _taskDevToolsWin = null;
let _taskWin = null;
let flaskProcess;
let autoJoinInterval = null;
let _isFirstLaunch = true; // true only until the first task-session-load call

// Per-window zoom state. Zoom ≤ 1.0 (cannot scale up past default).
// Size is always base * factor — never drifts.
const _winZoom = new WeakMap(); // BrowserWindow → { factor, baseW, baseH, minW, minH, maxW, maxH, tlX?, tlY? }
// Persisted factors survive window close/reopen
let _mainZoomFactor = 1.0;
let _taskZoomFactor = 1.0;

app.commandLine.appendSwitch('disable-features', 'UseOzonePlatform,WidgetLayeriting,RecordWebAppDebugInfo');
app.commandLine.appendSwitch('disable-partial-raster');
app.commandLine.appendSwitch('disable-software-rasterizer');
app.commandLine.appendSwitch('force-device-scale-factor', '1');

// Spawn Flask BEFORE the AppKit launch sequence completes, then block this
// thread until it answers. The dock keeps bouncing the entire time because
// applicationDidFinishLaunching: hasn't returned yet.
startFlaskServer();
waitForServerSync();


// ── Native settings addon (SwiftUI window inside this process) ────────────────
let settingsAddon = null;
function loadSettingsAddon() {
    if (settingsAddon) return settingsAddon;
    try {
        const addonPath = app.isPackaged
            ? path.join(process.resourcesPath, 'app.asar.unpacked', 'native', 'settings', 'build', 'Release', 'settings.node')
            : path.join(__dirname, 'native', 'settings', 'build', 'Release', 'settings.node');
        settingsAddon = require(addonPath);
    } catch (e) {
        console.error('Failed to load settings addon:', e.message);
    }
    return settingsAddon;
}

function openSettings() {
    const addon = loadSettingsAddon();
    if (addon) {
        addon.open();
    } else {
        console.warn('Settings addon not available');
    }
}

// ── Zoom — applies to ALL registered windows simultaneously ──────────────────
// factor is always absolute (0.8–1.0). Size = base * factor — never drifts,
// never breaks at min/max. Traffic lights move but don't resize (native limit).

function _applyZoom(win, newFactor) {
    if (!win || win.isDestroyed()) return;
    const z = _winZoom.get(win);
    if (!z) return;
    const f = Math.max(0.8, Math.min(1.0, Math.round(newFactor * 10) / 10));
    // Re-derive base from current live size so zoom is always relative to what
    // the user actually sees right now (respects manual resizes).
    const [curW, curH] = win.getSize();
    z.baseW = Math.round(curW / z.factor);
    z.baseH = Math.round(curH / z.factor);
    z.factor = f;
    win.webContents.setZoomFactor(f);
    // Clear constraints before resize so setSize is never clamped
    win.setMinimumSize(0, 0);
    if (z.maxW) win.setMaximumSize(99999, 99999);
    win.setSize(Math.round(z.baseW * f), Math.round(z.baseH * f));
    win.setMinimumSize(Math.round(z.minW * f), Math.round(z.minH * f));
    if (z.maxW) win.setMaximumSize(Math.round(z.maxW * f), Math.round(z.maxH * f));
    const addon = loadSettingsAddon();
    if (addon && addon.scaleTrafficLights) {
        try { addon.scaleTrafficLights(win.getNativeWindowHandle(), f); } catch {}
    }
    win.webContents.send('zoom-change', f);
}

function _zoomAllDelta(delta) {
    // Compute new factor from main window (source of truth); task window mirrors it
    const curFactor = _mainZoomFactor;
    const newFactor = Math.max(0.8, Math.min(1.0, Math.round((curFactor + delta) * 10) / 10));
    _mainZoomFactor = newFactor;
    _taskZoomFactor = newFactor;
    for (const win of BrowserWindow.getAllWindows()) {
        if (!_winZoom.has(win) || win.isDestroyed()) continue;
        _applyZoom(win, newFactor);
    }
    buildMenu();
}

function _zoomAllReset() {
    _mainZoomFactor = 1.0;
    _taskZoomFactor = 1.0;
    for (const win of BrowserWindow.getAllWindows()) {
        if (!_winZoom.has(win) || win.isDestroyed()) continue;
        _applyZoom(win, 1.0);
    }
    buildMenu();
}

function openTaskCounter() {
    if (_taskWin && !_taskWin.isDestroyed()) {
        _taskWin.focus();
        return;
    }
    const tf = _taskZoomFactor;
    _taskWin = new BrowserWindow({
        width:    Math.round(400 * tf), height:    Math.round(580 * tf),
        minWidth: Math.round(320 * tf), minHeight: Math.round(420 * tf),
        maxWidth: Math.round(520 * tf), maxHeight: Math.round(720 * tf),
        alwaysOnTop: true,
        backgroundColor: '#0a0e1a',
        show: false,
        titleBarStyle: 'hidden',
        trafficLightPosition: { x: 10, y: 7 },
        webPreferences: { nodeIntegration: false, contextIsolation: true, preload: path.join(__dirname, 'preload.js') },
    });
    _winZoom.set(_taskWin, { factor: tf, baseW: 400, baseH: 580, minW: 320, minH: 420, maxW: 520, maxH: 720 });
    // Clear Electron's persisted zoom for this origin
    _taskWin.webContents.setZoomFactor(1.0);
    _taskWin.webContents.setZoomLevel(0);
    const addon = loadSettingsAddon();
    if (addon && addon.setManaged) addon.setManaged(_taskWin.getNativeWindowHandle());
    _taskWin.loadURL('http://localhost:5001/t/tasks');
    _taskWin.once('ready-to-show', () => {
        if (addon && addon.setManaged) addon.setManaged(_taskWin.getNativeWindowHandle());
        if (_taskZoomFactor !== 1.0) {
            _taskWin.webContents.setZoomFactor(_taskZoomFactor);
            if (addon && addon.scaleTrafficLights)
                try { addon.scaleTrafficLights(_taskWin.getNativeWindowHandle(), _taskZoomFactor); } catch {}
        }
        _taskWin.show();
    });
    _taskWin.webContents.on('before-input-event', (e, input) => {
        if (input.meta && input.key === 'w') {
            e.preventDefault();
            if (_taskWin && !_taskWin.isDestroyed()) _taskWin.close();
        }
        if (input.meta && !input.shift && (input.key === '=' || input.key === '+')) {
            e.preventDefault(); _zoomAllDelta(+0.1);
        }
        if (input.meta && input.key === '-') {
            e.preventDefault(); _zoomAllDelta(-0.1);
        }
        if (input.meta && input.key === '0') {
            e.preventDefault(); _zoomAllReset();
        }
    });
    _taskWin.on('closed', () => {
        if (_taskDevToolsWin && !_taskDevToolsWin.isDestroyed()) _taskDevToolsWin.close();
        _taskWin = null;
    });
}

function openTaskDevTools() {
    if (!_taskWin || _taskWin.isDestroyed()) return;
    if (_taskDevToolsWin && !_taskDevToolsWin.isDestroyed()) {
        _taskDevToolsWin.focus();
        return;
    }
    _taskDevToolsWin = new BrowserWindow({
        width: 900, height: 650,
        backgroundColor: '#1e1e1e',
        show: false,
    });
    const addon = loadSettingsAddon();
    if (addon && addon.setManaged) addon.setManaged(_taskDevToolsWin.getNativeWindowHandle());
    _taskWin.webContents.setDevToolsWebContents(_taskDevToolsWin.webContents);
    _taskWin.webContents.openDevTools({ mode: 'detach', activate: true });
    _taskDevToolsWin.once('ready-to-show', () => {
        if (addon && addon.setManaged) addon.setManaged(_taskDevToolsWin.getNativeWindowHandle());
        _taskDevToolsWin.setTitle('DevTools — Task Tracker');
        setTimeout(() => { if (_taskDevToolsWin && !_taskDevToolsWin.isDestroyed()) _taskDevToolsWin.show(); }, 80);
    });
    _taskDevToolsWin.webContents.on('did-finish-load', () => {
        if (_taskDevToolsWin && !_taskDevToolsWin.isDestroyed())
            _taskDevToolsWin.setTitle('DevTools — Task Tracker');
    });
    _taskDevToolsWin.webContents.on('before-input-event', (e, input) => {
        if (input.meta && input.key === 'w') {
            e.preventDefault();
            if (_taskDevToolsWin && !_taskDevToolsWin.isDestroyed()) _taskDevToolsWin.close();
        }
    });
    _taskDevToolsWin.on('closed', () => { _taskDevToolsWin = null; });
}

function openDevTools() {
    if (!mainWindow) return;
    if (_devToolsWin && !_devToolsWin.isDestroyed()) {
        _devToolsWin.focus();
        return;
    }
    _devToolsWin = new BrowserWindow({
        width: 900, height: 650,
        backgroundColor: '#1e1e1e',
        show: false,
    });
    const addon = loadSettingsAddon();
    if (addon && addon.setManaged) addon.setManaged(_devToolsWin.getNativeWindowHandle());
    mainWindow.webContents.setDevToolsWebContents(_devToolsWin.webContents);
    mainWindow.webContents.openDevTools({ mode: 'detach', activate: true });
    _devToolsWin.once('ready-to-show', () => {
        if (addon && addon.setManaged) addon.setManaged(_devToolsWin.getNativeWindowHandle());
        _devToolsWin.setTitle('DevTools — Main Window');
        setTimeout(() => { if (_devToolsWin && !_devToolsWin.isDestroyed()) _devToolsWin.show(); }, 80);
    });
    _devToolsWin.webContents.on('did-finish-load', () => {
        if (_devToolsWin && !_devToolsWin.isDestroyed()) _devToolsWin.setTitle('DevTools — Main Window');
    });
    _devToolsWin.webContents.on('before-input-event', (e, input) => {
        if (input.meta && input.key === 'w') {
            e.preventDefault();
            if (_devToolsWin && !_devToolsWin.isDestroyed()) _devToolsWin.close();
        }
    });
    _devToolsWin.on('closed', () => { _devToolsWin = null; });
}

// ── Persistence ───────────────────────────────────────────────────────────────
function settingsPath() {
    return path.join(app.getPath('userData'), 'settings.json');
}

function loadSettings() {
    try { return JSON.parse(fs.readFileSync(settingsPath(), 'utf8')); }
    catch { return {}; }
}

// Watch settings.json for changes written by the SwiftUI addon and react.
function watchSettings() {
    let debounce = null;
    try {
        fs.watch(settingsPath(), () => {
            clearTimeout(debounce);
            debounce = setTimeout(() => {
                const s = loadSettings();
                if (s.autoJoinEnabled && s.autoJoinSsid) {
                    const net = (s.networks || []).find(n => n.ssid === s.autoJoinSsid);
                    startAutoJoin(s.autoJoinSsid, net ? net.pass : '');
                } else {
                    stopAutoJoin();
                }
                fetch('http://localhost:5001/api/undistort/reload', { method: 'POST' })
                    .then(() => { if (mainWindow) mainWindow.webContents.send('undistort-changed'); })
                    .catch(() => {});
                if (mainWindow) mainWindow.webContents.send('settings-photo-changed');
            }, 200);
        });
    } catch {}
}

// ── WiFi helpers (macOS networksetup) ────────────────────────────────────────
function getWifiInterface(cb) {
    exec('networksetup -listallhardwareports', (err, stdout) => {
        if (err) return cb('en0');
        const lines = stdout.split('\n');
        for (let i = 0; i < lines.length - 1; i++) {
            if (/Wi-Fi|AirPort/.test(lines[i])) {
                const m = lines[i + 1].match(/Device:\s+(\S+)/);
                if (m) return cb(m[1]);
            }
        }
        cb('en0');
    });
}

function getCurrentSSID(cb) {
    exec("networksetup -getairportnetwork en0 2>/dev/null || airport -I 2>/dev/null", (err, stdout) => {
        const m = stdout.match(/Current Wi-Fi Network:\s*(.+)/) || stdout.match(/SSID:\s*(.+)/);
        cb(m ? m[1].trim() : null);
    });
}

let networkStatus = { connected: false, ssid: null };

function broadcastNetworkStatus(status) {
    networkStatus = status;
    if (mainWindow && !mainWindow.isDestroyed())
        mainWindow.webContents.send('network-status', status);
}

function startAutoJoin(ssid, pass) {
    stopAutoJoin();
    const tryJoin = () => {
        getCurrentSSID(current => {
            if (current === ssid) {
                broadcastNetworkStatus({ connected: true, ssid });
                return;
            }
            getWifiInterface(iface => {
                const cmd = pass
                    ? `networksetup -setairportnetwork ${iface} "${ssid}" "${pass}"`
                    : `networksetup -setairportnetwork ${iface} "${ssid}"`;
                exec(cmd, () => {
                    getCurrentSSID(now => {
                        broadcastNetworkStatus({ connected: now === ssid, ssid: now === ssid ? ssid : networkStatus.ssid });
                    });
                });
            });
        });
    };
    tryJoin();
    autoJoinInterval = setInterval(tryJoin, 10000);
}

function stopAutoJoin() {
    if (autoJoinInterval) { clearInterval(autoJoinInterval); autoJoinInterval = null; }
    broadcastNetworkStatus({ connected: false, ssid: null });
}

// ── IPC (registered inside whenReady so ipcMain is guaranteed available) ──────

// ── Dev mode state (persists across menu rebuilds) ───────────────────────────
let _devMode = false;

// ── Task session persistence (owned by main.js, not Flask) ───────────────────
function taskSessionPath() {
    return path.join(app.getPath('userData'), 'task_session.json');
}

function loadTaskSession() {
    try { return JSON.parse(fs.readFileSync(taskSessionPath(), 'utf8')); }
    catch { return null; }
}

function writeTaskSession(data) {
    try { fs.writeFileSync(taskSessionPath(), JSON.stringify(data, null, 2)); } catch {}
}

function deleteTaskSession() {
    try { fs.unlinkSync(taskSessionPath()); } catch {}
}

// ── saveTasksOnRelaunch flag (persisted in settings.json) ────────────────────
function loadSaveTasksFlag() {
    return !!(loadSettings().saveTasksOnRelaunch);
}
function setSaveTasksFlag(val) {
    let s = loadSettings();
    s.saveTasksOnRelaunch = val;
    try { fs.writeFileSync(settingsPath(), JSON.stringify(s, null, 2)); } catch {}
    buildMenu();
    if (mainWindow) mainWindow.webContents.send('settings-photo-changed');
}

// ── Simulate flags (persisted in settings.json) ───────────────────────────────
function loadSimFlags() {
    const s = loadSettings();
    return {
        connected: !!(s.simConnected),
        opmode:    !!(s.simOpmode),
        data:      !!(s.simData),
    };
}
function saveSimFlags(flags) {
    let s = loadSettings();
    s.simConnected = flags.connected;
    s.simOpmode    = flags.opmode;
    s.simData      = flags.data;
    try { fs.writeFileSync(settingsPath(), JSON.stringify(s, null, 2)); } catch {}
}
let _simFlags = loadSimFlags();
function setSimFlag(key, val) {
    _simFlags[key] = val;
    saveSimFlags(_simFlags);
    buildMenu();
    if (mainWindow) mainWindow.webContents.send('simulate-flags', _simFlags);
}

const _ICON_NAMES = {
    gear:     'gear',
    reload:   'arrow.clockwise',
    restart:  'restart',
    devtools: 'wrench.and.screwdriver',
    devoff:   'xmark',
    zoomIn:   'plus.magnifyingglass',
    zoomOut:  'minus.magnifyingglass',
    zoomReset:'square.arrowtriangle.4.outward',
    fullscreen:'arrow.up.left.and.arrow.down.right',
};

function _menuIcon(key) {
    const name = _ICON_NAMES[key];
    if (!name) return undefined;
    try {
        const img = nativeImage.createFromNamedImage(name);
        if (!img || img.isEmpty()) return undefined;
        const size = img.getSize();
        const targetHeight = 14;
        const scale = targetHeight / size.height;
        const sized = img.resize({
            width: Math.round(size.width * scale),
            height: targetHeight
        });
        sized.setTemplateImage(true);
        return sized;
    } catch { return undefined; }
}



// ── App menu ──────────────────────────────────────────────────────────────────
let _currentMenu = null;

function buildMenu() {
    // Single Reload item whose click handler checks for Option key.
    // The KeyboardEvent passed to click() includes modifiers, so no alternate item needed.
    const debugItems = [
        {
            label: 'Reload Current Window',
            accelerator: _devMode ? 'CmdOrCtrl+R' : undefined,
            icon: _menuIcon('reload'),
            click: (menuItem, win, event) => {
                if (!_devMode && event.altKey) {
                    _devMode = true;
                    buildMenu();
                } else {
                    const focused = require('electron').BrowserWindow.getFocusedWindow();
                    const target = focused || mainWindow;
                    if (target) target.webContents.reload();
                }
            },
        },
    ];

    debugItems.push({ type: 'separator' });
    debugItems.push({
        label: 'Restart Camera Feed',
        accelerator: 'CmdOrCtrl+Shift+R',
        icon: _menuIcon('restart'),
        click: () => {
            if (mainWindow) mainWindow.webContents.send('camera-restarting');
            if (flaskProcess) { try { flaskProcess.kill(); } catch {} flaskProcess = null; }
            startFlaskServer();
            // Poll until Flask responds, then notify renderer
            const deadline = Date.now() + 20000;
            function poll() {
                if (Date.now() > deadline) return;
                const r = spawnSync('/usr/bin/curl', ['-s', '-o', '/dev/null', '-w', '%{http_code}', '--max-time', '1', 'http://127.0.0.1:5001/'], { encoding: 'utf8' });
                const code = (r.stdout || '').trim();
                if (code && code !== '000') {
                    if (mainWindow) mainWindow.webContents.send('camera-ready');
                } else {
                    setTimeout(poll, 400);
                }
            }
            setTimeout(poll, 600);
        },
    });

    if (_devMode) {
        debugItems.push({ type: 'separator' });
        debugItems.push({
            label: 'Simulate Connected',
            type: 'checkbox',
            checked: _simFlags.connected,
            click: (item) => setSimFlag('connected', item.checked),
        });
        debugItems.push({
            label: 'Simulate OpMode',
            type: 'checkbox',
            checked: _simFlags.opmode,
            click: (item) => setSimFlag('opmode', item.checked),
        });
        debugItems.push({
            label: 'Simulate Data',
            type: 'checkbox',
            checked: _simFlags.data,
            click: (item) => setSimFlag('data', item.checked),
        });
        debugItems.push({ type: 'separator' });
        debugItems.push({
            label: 'Developer Tools',
            accelerator: 'CmdOrCtrl+Alt+I',
            icon: _menuIcon('devtools'),
            click: () => {
                const focused = require('electron').BrowserWindow.getFocusedWindow();
                if (focused && focused === _taskWin && !focused.isDestroyed()) {
                    openTaskDevTools();
                } else {
                    openDevTools();
                }
            },
        });
        debugItems.push({
            label: 'Exit Developer Mode',
            icon: _menuIcon('devoff'),
            click: () => { _devMode = false; buildMenu(); },
        });
    }

    const template = [
        {
            label: app.name,
            submenu: [
                { role: 'about' },
                { type: 'separator' },
                { label: 'Settings…', accelerator: 'Cmd+,', click: openSettings, icon: _menuIcon('gear') },
                { type: 'separator' },
                { role: 'hide' }, { role: 'hideOthers' }, { role: 'unhide' },
                { type: 'separator' },
                { role: 'quit' },
            ],
        },
        {
            label: 'Edit',
            role: 'editMenu'
        },
        {
            label: 'Window',
            role: 'windowMenu',
        },
        {
            label: 'View',
            submenu: [
                {
                    label: 'Zoom In',
                    accelerator: 'Cmd+=',
                    icon: _menuIcon('zoomIn'),
                    enabled: _mainZoomFactor < 1.0,
                    click: () => _zoomAllDelta(+0.1),
                },
                {
                    label: 'Zoom Out',
                    accelerator: 'Cmd+-',
                    icon: _menuIcon('zoomOut'),
                    enabled: _mainZoomFactor > 0.8,
                    click: () => _zoomAllDelta(-0.1),
                },
                {
                    label: 'Reset Zoom',
                    accelerator: 'Cmd+0',
                    icon: _menuIcon('zoomReset'),
                    enabled: _mainZoomFactor !== 1.0,
                    click: () => _zoomAllReset(),
                },
            ],
        },
        {
            label: 'Tasks',
            submenu: [
                {
                    label: 'Save Tasks on Relaunch',
                    type: 'checkbox',
                    checked: loadSaveTasksFlag(),
                    click: (item) => setSaveTasksFlag(item.checked),
                },
                { type: 'separator' },
                {
                    label: 'Launch Task Tracker',
                    accelerator: 'Cmd+T',
                    icon: (() => {
                        try {
                            const img = nativeImage.createFromNamedImage('arrow.up.right');
                            if (!img || img.isEmpty()) return undefined;
                            const sz = img.getSize();
                            const scale = 14 / sz.height;
                            const sized = img.resize({ width: Math.round(sz.width * scale), height: 14 });
                            sized.setTemplateImage(true);
                            return sized;
                        } catch { return undefined; }
                    })(),
                    click: () => openTaskCounter(),
                },
                {
                    label: 'Reset Tracker',
                    icon: _menuIcon('reload'),
                    click: () => {
                        deleteTaskSession();
                        if (_taskWin && !_taskWin.isDestroyed()) {
                            _taskWin.webContents.send('task-session-reset');
                        }
                    },
                },
            ],
        },
        { label: 'Debug', submenu: debugItems },
    ];

    _currentMenu = Menu.buildFromTemplate(template);
    Menu.setApplicationMenu(_currentMenu);
    const addon = loadSettingsAddon();
    if (addon) {
        // Override the Zoom In key-equivalent display to "+" while keeping
        // the actual binding on Cmd+= (the physical key on US keyboards).
        if (addon.setMenuKeyEquivalent) {
            try { addon.setMenuKeyEquivalent('Zoom In', '+', false); } catch {}
        }
        // Give the auto-injected native fullscreen item icon, label, and a
        // separator above it; also installs FS notification observers (once).
        if (addon.decorateFullScreenMenuItem) {
            try { addon.decorateFullScreenMenuItem(); } catch {}
        }
    }
}

// ── Flask server ──────────────────────────────────────────────────────────────
function getProjectRoot() {
    return app.isPackaged ? process.resourcesPath : __dirname;
}

function startFlaskServer() {
    const root   = getProjectRoot();
    const python = path.join(root, 'app/Resources/python-runtime/bin/python3');
    const script = path.join(root, 'web/app.py');
    console.log('Starting server...');
    flaskProcess = spawn(python, [script], {
        cwd: path.join(root, 'web'),
        env: { ...process.env, ELECTRON_IS_PACKAGED: app.isPackaged ? '1' : '' }
    });
    flaskProcess.stdout.on('data', d => { try { process.stdout.write(`${d}`); } catch {} });
    flaskProcess.stderr.on('data', d => { try { process.stderr.write(`${d}`); } catch {} });
    flaskProcess.on('error', err => { try { console.error('Server error:', err); } catch {} });
}

// Block the main thread until Flask answers on :5001. Called BEFORE app
// finishes launching so macOS keeps the dock icon bouncing the whole time.
function waitForServerSync(timeoutMs = 30000) {
    const deadline = Date.now() + timeoutMs;
    while (Date.now() < deadline) {
        const r = spawnSync('/usr/bin/curl', [
            '-s', '-o', '/dev/null', '-w', '%{http_code}',
            '--max-time', '1',
            'http://127.0.0.1:5001/'
        ], { encoding: 'utf8' });
        const code = (r.stdout || '').trim();
        if (code && code !== '000') return true;
        spawnSync('/bin/sleep', ['0.1']);
    }
    return false;
}

// ── Main window ───────────────────────────────────────────────────────────────
function createWindow() {
    mainWindow = new BrowserWindow({
        width: 1820, height: 1020, minHeight: 900, minWidth: 1600,
        webPreferences: { nodeIntegration: false, contextIsolation: true, preload: path.join(__dirname, 'preload.js'), backgroundThrottling: false },
        titleBarStyle: 'hidden',
        trafficLightPosition: { x: 12, y: 9 },
        backgroundColor: '#1a1a1a',
    });
    _winZoom.set(mainWindow, { factor: 1.0, baseW: 1820, baseH: 1020, minW: 1600, minH: 900, maxW: 0, maxH: 0 });
    // Clear Electron's persisted zoom for this origin so it always starts at 1.0
    mainWindow.webContents.setZoomFactor(1.0);
    mainWindow.webContents.setZoomLevel(0);
    var simQ = `?sc=${_simFlags.connected?1:0}&so=${_simFlags.opmode?1:0}&sd=${_simFlags.data?1:0}`;
    mainWindow.loadURL('http://localhost:5001' + simQ);
    mainWindow.on('enter-full-screen', () => mainWindow.webContents.send('traffic-lights-hidden'));
    mainWindow.on('leave-full-screen',  () => mainWindow.webContents.send('traffic-lights-visible'));
    mainWindow.webContents.on('did-finish-load', () => {
        var newSim = `?sc=${_simFlags.connected?1:0}&so=${_simFlags.opmode?1:0}&sd=${_simFlags.data?1:0}`;
        if (newSim !== simQ) {
            simQ = newSim;
            mainWindow.loadURL('http://localhost:5001' + newSim);
        }
        if (mainWindow.isFullScreen()) mainWindow.webContents.send('traffic-lights-hidden')
    });
    mainWindow.on('closed', () => {
        if (_devToolsWin && !_devToolsWin.isDestroyed()) _devToolsWin.close();
        mainWindow = null;
    });
}

// ── Lifecycle ─────────────────────────────────────────────────────────────────

app.whenReady().then(() => {
    ipcMain.on('open-settings', () => openSettings());
    ipcMain.on('open-task-counter', () => openTaskCounter());


    // Task session: renderer sends state on every change; main.js writes synchronously.
    // Always write to disk — read back at load time only if flag is on.
    ipcMain.on('task-session-save', (_, data) => {
        writeTaskSession(data);
    });
    ipcMain.handle('task-session-load', () => {
        const firstLaunch = _isFirstLaunch;
        _isFirstLaunch = false;
        // On relaunch: only restore if flag is on
        // On window reopen within same session: always restore
        if (firstLaunch && !loadSaveTasksFlag()) return null;
        return loadTaskSession();
    });
    ipcMain.on('task-session-reset', () => {
        deleteTaskSession();
        if (_taskWin && !_taskWin.isDestroyed()) {
            _taskWin.webContents.send('task-session-reset');
        }
    });

    ipcMain.handle('list-cameras', () => {
        const addon = loadSettingsAddon();
        if (!addon || !addon.listCameras) return '[]';
        try { return addon.listCameras(); }
        catch { return '[]'; }
    });

    // ── AI proxy helpers ──────────────────────────────────────────────
    // Forward a request to a cloud AI provider. Provider's API key is
    // pulled from the macOS Keychain via the native addon — keys never
    // leave the main process. Apple Intelligence is dispatched in step 9.
    function aiCall(provider, model, body, cb) {
        const addon = loadSettingsAddon();
        if (!addon || !addon.aiKeyGet) return cb(new Error('addon-unavailable'));
        let key = null;
        if (provider !== 'apple') {
            try { key = addon.aiKeyGet(provider); } catch { key = null; }
            if (!key) return cb(new Error('no-key-for-' + provider));
        }
        let host, p, headers;
        if (provider === 'openai') {
            host = 'api.openai.com';
            p = '/v1/chat/completions';
            headers = {
                'Content-Type': 'application/json',
                'Authorization': 'Bearer ' + key,
            };
        } else if (provider === 'anthropic') {
            host = 'api.anthropic.com';
            p = '/v1/messages';
            headers = {
                'Content-Type': 'application/json',
                'x-api-key': key,
                'anthropic-version': '2023-06-01',
            };
        } else if (provider === 'google') {
            host = 'generativelanguage.googleapis.com';
            p = `/v1beta/models/${encodeURIComponent(model)}:generateContent?key=${encodeURIComponent(key)}`;
            headers = { 'Content-Type': 'application/json' };
        } else if (provider === 'apple') {
            const addon = loadSettingsAddon();
            if (!addon || !addon.appleIntelligenceGenerate)
                return cb(new Error('apple-addon-missing'));
            const prompt = (body && body.messages)
                ? body.messages.map(m => {
                    if (typeof m.content === 'string') return m.content;
                    if (Array.isArray(m.content)) return m.content.filter(c => c.type === 'text').map(c => c.text).join('\n');
                    return '';
                  }).join('\n')
                : (body && body.prompt) ? body.prompt : JSON.stringify(body || {});
            // Run in a worker thread so the main-thread event loop stays live
            const { Worker, isMainThread, workerData, parentPort } = require('worker_threads');
            const workerSrc = `
                const { workerData, parentPort } = require('worker_threads');
                const addon = require(workerData.addonPath);
                try {
                    const out = addon.appleIntelligenceGenerate(workerData.prompt);
                    parentPort.postMessage({ ok: true, out });
                } catch(e) {
                    parentPort.postMessage({ ok: false, err: String(e.message || e) });
                }
            `;
            let addonPath;
            try { addonPath = require.resolve('./native/settings/build/Release/settings.node'); }
            catch { try { addonPath = require.resolve('./native/settings/build/Debug/settings.node'); } catch { return cb(new Error('apple-addon-path-unknown')); } }
            const w = new Worker(workerSrc, { eval: true, workerData: { addonPath, prompt } });
            w.once('message', msg => {
                if (msg.ok) cb(null, { status: 200, body: msg.out });
                else cb(new Error(msg.err));
            });
            w.once('error', e => cb(e));
            return;
        } else {
            return cb(new Error('unknown-provider-' + provider));
        }
        const payload = Buffer.from(JSON.stringify(body || {}));
        const req = https.request({
            host, port: 443, path: p, method: 'POST', headers: {
                ...headers, 'Content-Length': payload.length,
            },
        }, resp => {
            let chunks = [];
            resp.on('data', d => chunks.push(d));
            resp.on('end', () => {
                cb(null, {
                    status: resp.statusCode,
                    body: Buffer.concat(chunks).toString('utf8'),
                });
            });
        });
        req.on('error', e => cb(e));
        req.setTimeout(120000, () => { try { req.destroy(new Error('timeout')); } catch {} });
        req.write(payload);
        req.end();
    }

    // Tiny internal API server on 5002 so Flask can call native-only
    // facilities: camera enumeration, AI provider status, AI proxy.
    http.createServer((req, res) => {
        const send = (code, ctype, payload) => {
            res.writeHead(code, { 'Content-Type': ctype });
            res.end(payload);
        };
        if (req.method === 'GET' && req.url === '/cameras') {
            const addon = loadSettingsAddon();
            let json = '[]';
            if (addon && addon.listCameras) {
                try { json = addon.listCameras(); } catch {}
            }
            return send(200, 'application/json', json);
        }
        if (req.method === 'GET' && req.url === '/ai_providers') {
            const addon = loadSettingsAddon();
            let json = '{}';
            if (addon && addon.aiProvidersJson) {
                try { json = addon.aiProvidersJson(); } catch {}
            }
            return send(200, 'application/json', json);
        }
        if (req.method === 'POST' && req.url === '/ai_call') {
            let raw = '';
            req.on('data', d => { raw += d; if (raw.length > 32 * 1024 * 1024) req.destroy(); });
            req.on('end', () => {
                let parsed;
                try { parsed = JSON.parse(raw || '{}'); }
                catch { return send(400, 'application/json', '{"error":"bad-json"}'); }
                const { provider, model, body } = parsed;
                if (!provider || !model) {
                    return send(400, 'application/json',
                        '{"error":"missing-provider-or-model"}');
                }
                aiCall(provider, model, body, (err, out) => {
                    if (err) {
                        return send(502, 'application/json',
                            JSON.stringify({ error: String(err.message || err) }));
                    }
                    send(out.status || 200, 'application/json', out.body || '{}');
                });
            });
            return;
        }
        res.writeHead(404);
        res.end();
    }).listen(5002, '127.0.0.1');

    app.setAboutPanelOptions({
        applicationName:    app.name,
        applicationVersion: [
            `Version ${_pkg.version} (${_pkg.buildNumber ?? 1}) arm64`,
            '',
            `Chromium Engine Version: ${process.versions.chrome}`,
            `Electron Version: ${process.versions.electron}`,
            `Node.JS Version: ${process.versions.node}`,
            '',
            _pkg.description ?? '',
            '',
            `${_APP_COPYRIGHT}. All Rights Reserved.`
        ].join('\n'),
        version:   '',
        copyright: '',
        credits:   '',
        iconPath:  app.isPackaged
                       ? path.join(process.resourcesPath, '..', 'Resources', 'icon.icns')
                       : path.join(__dirname, 'build', 'icon.icns'),
    });
    buildMenu();
    nativeTheme.on('updated', () => buildMenu());
    loadSettingsAddon(); // load the .node early so SwiftUI initialises on the main thread
    // Flask was started synchronously at top-level and is already up.
    createWindow();
    watchSettings();
    const s = loadSettings();
    if (s.autoJoinEnabled && s.autoJoinSsid) {
        const net = (s.networks || []).find(n => n.ssid === s.autoJoinSsid);
        startAutoJoin(s.autoJoinSsid, net ? net.pass : '');
    }
});

app.on('window-all-closed', () => {
    const addon = loadSettingsAddon();
    if (addon) addon.close();
    if (flaskProcess) flaskProcess.kill();
    app.quit();
});
app.on('activate', () => { if (!mainWindow) createWindow(); });
app.on('will-quit', () => {
    if (flaskProcess) flaskProcess.kill();
});
