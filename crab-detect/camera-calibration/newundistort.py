import cv2
import numpy as np

# ================= SETTINGS =================
# Replace these with your calibration files
K = np.loadtxt("fisheye_output/K.txt")
D = np.loadtxt("fisheye_output/D.txt").reshape(4, 1)  # ensure shape is (4,1)

BALANCE = 0.3       # 0 = crop, 1 = keep full FOV
CAMERA_INDEX = 0    # change if you have multiple cameras

# =================================================

# ================= OPEN CAMERA =================
cap = cv2.VideoCapture(CAMERA_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
assert cap.isOpened(), "Cannot open camera"

# Grab one frame to determine resolution
ret, frame = cap.read()
assert ret, "Failed to read from camera"
h, w = frame.shape[:2]
print(f"Camera frame resolution: {w}x{h}")

# ================= PRECOMPUTE UNDISTORT MAPS =================
# This must match the actual frame size
newK = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
    K, D, (w, h), np.eye(3), balance=BALANCE
)

map1, map2 = cv2.fisheye.initUndistortRectifyMap(
    K, D, np.eye(3), newK, (w, h), cv2.CV_16SC2
)

# ================= STREAM LOOP =================
print("Press 'q' to quit")
while True:
    ret, frame = cap.read()
    if not ret:
        break

    undistorted = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)

    # Display side by side
    combined = np.hstack((frame, undistorted))
    cv2.imshow("Original | Undistorted (Fisheye)", combined)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# ================= CLEANUP =================
cap.release()
cv2.destroyAllWindows()
