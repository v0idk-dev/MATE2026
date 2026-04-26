const { app, BrowserWindow, Menu, ipcMain, nativeImage, nativeTheme } = require('electron');
const { spawn, exec }  = require('child_process');
const http = require('http');
const path = require('path');
const fs   = require('fs');

const _pkg = (() => {
    try { return JSON.parse(fs.readFileSync(path.join(__dirname, 'package.json'), 'utf8')); }
    catch { return {}; }
})();

// electron-builder strips the "build" key from the bundled package.json,
// so these values must be hardcoded here and kept in sync with package.json.
const _APP_PRODUCT_NAME = 'MATE 2026 Robot Controller';
const _APP_COPYRIGHT    = 'Copyright © 2026 Dogukan Koc';

app.name = _APP_PRODUCT_NAME;

let mainWindow;
let _devToolsWin = null;
let flaskProcess;
let autoJoinInterval = null;

app.commandLine.appendSwitch('disable-features', 'UseOzonePlatform,WidgetLayeriting,RecordWebAppDebugInfo');
app.commandLine.appendSwitch('disable-partial-raster');
app.commandLine.appendSwitch('disable-software-rasterizer');
app.commandLine.appendSwitch('force-device-scale-factor', '1');


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
    // Set .managed before openDevTools so Electron never gets a chance to
    // override the collection behavior with .fullScreenAuxiliary.
    const addon = loadSettingsAddon();
    if (addon && addon.setManaged) {
        addon.setManaged(_devToolsWin.getNativeWindowHandle());
    }
    mainWindow.webContents.setDevToolsWebContents(_devToolsWin.webContents);
    mainWindow.webContents.openDevTools({ mode: 'detach', activate: true });
    // Re-apply after openDevTools in case Electron resets it, then show once painted.
    _devToolsWin.once('ready-to-show', () => {
        if (addon && addon.setManaged) {
            addon.setManaged(_devToolsWin.getNativeWindowHandle());
        }
        setTimeout(() => { if (_devToolsWin && !_devToolsWin.isDestroyed()) _devToolsWin.show(); }, 80);
    });
    // Cmd+W closes devtools window, not main window
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
    devtools: 'wrench.and.screwdriver',
    devoff:   'xmark',
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
            label: 'Reload App',
            accelerator: _devMode ? 'CmdOrCtrl+R' : undefined,
            icon: _menuIcon('reload'),
            click: (menuItem, win, event) => {
                if (!_devMode && event.altKey) {
                    _devMode = true;
                    buildMenu();
                } else {
                    if (mainWindow) mainWindow.webContents.reload();
                }
            },
        },
    ];

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
            click: () => openDevTools(),
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
            role: 'windowMenu'
        },
        { label: 'Debug', submenu: debugItems },
    ];

    _currentMenu = Menu.buildFromTemplate(template);
    Menu.setApplicationMenu(_currentMenu);
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
    flaskProcess.stdout.on('data', d => {
        try {
            const text = `${d}`;
            process.stdout.write(text);
            if (text.includes('MATE_SERVER_READY') && !mainWindow) {
                createWindow();
            }
        } catch {}
    });
    flaskProcess.stderr.on('data', d => { try { process.stderr.write(`${d}`); } catch {} });
    flaskProcess.on('error', err => { try { console.error('Server error:', err); } catch {} });
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
    var simQ = `?sc=${_simFlags.connected?1:0}&so=${_simFlags.opmode?1:0}&sd=${_simFlags.data?1:0}`;
    mainWindow.loadURL('http://localhost:5001' + simQ);
    mainWindow.on('enter-full-screen', () => mainWindow.webContents.send('traffic-lights-hidden'));
    mainWindow.on('leave-full-screen',  () => mainWindow.webContents.send('traffic-lights-visible'));
    // On every load (including reload), tell renderer current fullscreen state
    mainWindow.webContents.on('did-finish-load', () => {
        var newSim = `?sc=${_simFlags.connected?1:0}&so=${_simFlags.opmode?1:0}&sd=${_simFlags.data?1:0}`;
        if (newSim !== simQ) {
            simQ = newSim;
            mainWindow.loadURL('http://localhost:5001' + newSim);
        }
        if (mainWindow.isFullScreen()) mainWindow.webContents.send('traffic-lights-hidden')
    });
    mainWindow.on('closed', () => { mainWindow = null; });
}

// ── Lifecycle ─────────────────────────────────────────────────────────────────

app.whenReady().then(() => {
    ipcMain.on('open-settings', () => openSettings());

    ipcMain.handle('list-cameras', () => {
        const addon = loadSettingsAddon();
        if (!addon || !addon.listCameras) return '[]';
        try { return addon.listCameras(); }
        catch { return '[]'; }
    });

    // Tiny internal API server on 5002 so Flask can call listCameras via the addon.
    http.createServer((req, res) => {
        if (req.method === 'GET' && req.url === '/cameras') {
            const addon = loadSettingsAddon();
            let json = '[]';
            if (addon && addon.listCameras) {
                try { json = addon.listCameras(); } catch {}
            }
            res.writeHead(200, { 'Content-Type': 'application/json' });
            res.end(json);
        } else {
            res.writeHead(404);
            res.end();
        }
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
    startFlaskServer();
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
