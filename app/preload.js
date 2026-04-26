const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('electronAPI', {
    runTask: (taskId, images) => ipcRenderer.invoke('run-task', taskId, images),
    getOutputs: (taskId) => ipcRenderer.invoke('get-outputs', taskId),
    deleteFile: (filepath) => ipcRenderer.invoke('delete-file', filepath),
    getCameraStream: (cameraId) => `http://localhost:8081/camera/${cameraId}`,
    onUndistortChanged: (cb) => ipcRenderer.on('undistort-changed', cb),
    onSettingsPhotoChanged: (cb) => ipcRenderer.on('settings-photo-changed', cb),
    onTrafficLightsHidden: (cb) => ipcRenderer.on('traffic-lights-hidden', cb),
    onTrafficLightsVisible: (cb) => ipcRenderer.on('traffic-lights-visible', cb),
    onWindowResizeStart: (cb) => ipcRenderer.on('window-resize-start', cb),
    onWindowResizeEnd: (cb) => ipcRenderer.on('window-resize-end', cb),
    openSettings: () => ipcRenderer.send('open-settings'),
});