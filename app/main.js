const { app, BrowserWindow } = require('electron');
const { spawn } = require('child_process');
const path = require('path');

let mainWindow;
let flaskProcess;

// Get the correct project root
function getProjectRoot() {
    if (app.isPackaged) {
        // When packaged, resources are in app.asar or Resources folder
        return process.resourcesPath;
    } else {
        // During development, go up from electron's location to project root
        // If you run `npm start` from project root, __dirname is project root
        return __dirname;
    }
}

// Start Flask backend
function startFlaskServer() {
    const projectRoot = getProjectRoot();
    const pythonPath = path.join(projectRoot, 'app/Resources/python-runtime/bin/python3');
    const flaskScript = path.join(projectRoot, 'web/app.py');
    
    console.log('Starting Flask server...');
    console.log('Python path:', pythonPath);
    console.log('Flask script:', flaskScript);
    
    flaskProcess = spawn(pythonPath, [flaskScript], {
        cwd: projectRoot,
        env: { ...process.env }
    });
    
    flaskProcess.stdout.on('data', (data) => {
        console.log(`${data}`);
    });
    
    flaskProcess.stderr.on('data', (data) => {
        console.log(`${data}`);
    });
    
    flaskProcess.on('error', (err) => {
        console.error('Failed to start Flask:', err);
    });
}

function createWindow() {
    mainWindow = new BrowserWindow({
        width: 1400,
        height: 900,
        webPreferences: {
            nodeIntegration: false,
            contextIsolation: true,
        },
        titleBarStyle: 'hidden',  // ← Change from 'hiddenInset' to 'hidden'
        trafficLightPosition: { x: 12, y: 8 },  // ← Add this line (positions red/yellow/green buttons)
        backgroundColor: '#1a1a1a'
    });

    // Load your Flask app (it runs on port 5001 based on your app.py)
    mainWindow.loadURL('http://localhost:5001');
    
    // Open DevTools in development
    if (!app.isPackaged) {
        mainWindow.webContents.openDevTools();
    }
    
    mainWindow.on('closed', () => {
        mainWindow = null;
    });
}

app.whenReady().then(() => {
    startFlaskServer();
    
    // Wait for Flask to start (2 seconds should be enough)
    setTimeout(createWindow, 2000);
});

app.on('window-all-closed', () => {
    if (flaskProcess) {
        console.log('Killing Flask process...');
        flaskProcess.kill();
    }
    app.quit();
});

app.on('activate', () => {
    if (mainWindow === null) {
        createWindow();
    }
});

// Cleanup on exit
app.on('will-quit', () => {
    if (flaskProcess) {
        flaskProcess.kill();
    }
});

