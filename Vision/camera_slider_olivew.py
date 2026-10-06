import cv2
import numpy as np
import time
from picamera2 import Picamera2

picam2 = Picamera2()

config = picam2.create_video_configuration(
    main={"format": "RGB888", "size": (1280, 720)},
    controls={"FrameRate": 30}
)

picam2.configure(config)

picam2.set_controls({
    "AeEnable": False,
    "AwbEnable": False,
    "ExposureTime": 20000,
    "AnalogueGain": 1.0,
    "ColourGains": (1.75, 2.7),
    "Saturation": 1.5,
    "ColourCorrectionMatrix": np.eye(3).flatten().tolist()
})

picam2.start()
time.sleep(2)

# --- Trackbar window ---
cv2.namedWindow("Controls")

def nothing(x):
    pass

# Create trackbars for thresholds (0–100 mapped to 0.0–1.0)
cv2.createTrackbar("Y_R", "Controls", 40, 100, nothing)
cv2.createTrackbar("Y_G", "Controls", 50, 100, nothing)
cv2.createTrackbar("Y_B", "Controls", 60, 100, nothing)

cv2.createTrackbar("B_R", "Controls", 45, 100, nothing)
cv2.createTrackbar("B_G", "Controls", 70, 100, nothing)
cv2.createTrackbar("B_B", "Controls", 35, 100, nothing)

cv2.createTrackbar("R_R", "Controls", 70, 100, nothing)
cv2.createTrackbar("R_G", "Controls", 50, 100, nothing)
cv2.createTrackbar("R_B", "Controls", 65, 100, nothing)

cv2.createTrackbar("G_R", "Controls", 50, 100, nothing)
cv2.createTrackbar("G_G", "Controls", 60, 100, nothing)
cv2.createTrackbar("G_B", "Controls", 55, 100, nothing)

try:
    while True:
        frame = picam2.capture_array().astype(np.float32)

        R = frame[:, :, 0]
        G = frame[:, :, 1]
        B = frame[:, :, 2]

        # Brightness-normalised RGB
        L = np.sqrt(R*R + G*G + B*B)
        L[L == 0] = 1e-6

        Rn = R / L
        Gn = G / L
        Bn = B / L

        # --- Read trackbars ---
        Y_R = cv2.getTrackbarPos("Y_R", "Controls") / 100.0
        Y_G = cv2.getTrackbarPos("Y_G", "Controls") / 100.0
        Y_B = cv2.getTrackbarPos("Y_B", "Controls") / 100.0

        B_R = cv2.getTrackbarPos("B_R", "Controls") / 100.0
        B_G = cv2.getTrackbarPos("B_G", "Controls") / 100.0
        B_B = cv2.getTrackbarPos("B_B", "Controls") / 100.0

        R_R = cv2.getTrackbarPos("R_R", "Controls") / 100.0
        R_G = cv2.getTrackbarPos("R_G", "Controls") / 100.0
        R_B = cv2.getTrackbarPos("R_B", "Controls") / 100.0

        G_R = cv2.getTrackbarPos("G_R", "Controls") / 100.0
        G_G = cv2.getTrackbarPos("G_G", "Controls") / 100.0
        G_B = cv2.getTrackbarPos("G_B", "Controls") / 100.0

        # --- Colour masks using trackbars ---
        yellow_mask = (Rn < Y_R) & (Gn > Y_G) & (Bn > Y_B)
        blue_mask   = (Rn > B_R) & (Gn < B_G) & (Bn < B_B)
        red_mask    = (Rn < R_R) & (Gn < R_G) & (Bn > R_B)
        green_mask  = (Rn > G_R) & (Gn > G_G) & (Bn < G_B)

        yellow_u8 = (yellow_mask * 255).astype(np.uint8)
        blue_u8   = (blue_mask   * 255).astype(np.uint8)
        red_u8    = (red_mask    * 255).astype(np.uint8)
        green_u8  = (green_mask  * 255).astype(np.uint8)

        frame_disp = frame.astype(np.uint8)

        # --- Bounding box helper ---
        def draw_boxes(mask_u8, colour_name, colour_bgr):
            contours, _ = cv2.findContours(mask_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                if cv2.contourArea(cnt) < 50:
                    continue
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(frame_disp, (x, y), (x+w, y+h), colour_bgr, 2)
                cv2.putText(frame_disp, colour_name, (x, y-5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour_bgr, 2)

        draw_boxes(yellow_u8, "Yellow", (0,255,255))
        draw_boxes(blue_u8,   "Blue",   (255,0,0))
        draw_boxes(red_u8,    "Red",    (0,0,255))
        draw_boxes(green_u8,  "Green",  (0,255,0))

        # --- Display windows ---
        cv2.imshow("Camera Feed + Bounding Boxes", frame_disp)
        cv2.imshow("Yellow", yellow_u8)
        cv2.imshow("Blue", blue_u8)
        cv2.imshow("Red", red_u8)
        cv2.imshow("Green", green_u8)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

finally:
    cv2.destroyAllWindows()
    picam2.stop()
    picam2.close()
