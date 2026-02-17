const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('electronAPI', {
    runTask: (taskId, images) => ipcRenderer.invoke('run-task', taskId, images),
    getOutputs: (taskId) => ipcRenderer.invoke('get-outputs', taskId),
    deleteFile: (filepath) => ipcRenderer.invoke('delete-file', filepath),
    getCameraStream: (cameraId) => `http://localhost:8081/camera/${cameraId}`
});