import time
import cv2
import numpy as np
from picamera2 import Picamera2

picam2 = Picamera2()

config = picam2.create_video_configuration(
    main={"format": "RGB888", "size": (2304, 1296)},
    controls={
        "FrameRate": 30
    }
)

picam2.configure(config)

# --- Manual camera controls (no auto exposure / AWB) ---
picam2.set_controls({
    "AeEnable": True,
    "AwbEnable": False,
    #"ExposureTime": 20000,      # 12 ms
    "AnalogueGain": 1.0,
    "ColourGains": (1.75, 2.7),
    "Saturation":1.5,
    "ColourCorrectionMatrix": np.eye(3).flatten().tolist()

    #"SensorMode": 2             # IMX708 HCG mode
})
def nothing(x):
    pass
picam2.start()
time.sleep(2)
cv2.namedWindow('App')
cv2.createTrackbar("Vmax","App",0,1,nothing)
try:
    while True:
        frame = picam2.capture_array().astype(np.float32)

        # Split channels
        R = frame[:, :, 0]
        G = frame[:, :, 1]
        B = frame[:, :, 2]

        # Brightness-normalised RGB (unit vector)
        L = np.sqrt(R*R + G*G + B*B)
        L[L == 0] = 1e-6

        Rn = R / L
        Gn = G / L
        Bn = B / L

        # --- Colour masks ---
        yellow_mask = (Rn < 0.4) & (Gn > 0.5) & (Bn > 0.66)
        blue_mask   = (Rn > 0.67) & (Gn < 0.6) & (Bn < 0.34)
        red_mask    = (Rn < 0.55) & (Gn < 0.52) & (Bn > 0.71)
        green_mask  = (Rn > 0.54) & (Gn > 0.61) & (Bn < 0.88)

        # Convert masks to uint8 for contour detection
        yellow_u8 = (yellow_mask * 255).astype(np.uint8)
        blue_u8   = (blue_mask   * 255).astype(np.uint8)
        red_u8    = (red_mask    * 255).astype(np.uint8)
        green_u8  = (green_mask  * 255).astype(np.uint8)

        # Work on the REAL frame (not normalised)
        frame_disp = frame.astype(np.uint8)

        # --- Bounding box helper ---
        def draw_boxes(mask_u8, colour_name, colour_bgr):
            contours, _ = cv2.findContours(mask_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 50:   # ignore tiny noise
                    continue
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(frame_disp, (x, y), (x+w, y+h), colour_bgr, 2)
                cv2.putText(frame_disp, colour_name, (x, y-5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour_bgr, 2)

        # --- Draw bounding boxes for each colour ---
        draw_boxes(yellow_u8, "Yellow", (0,255,255))
        draw_boxes(blue_u8,   "Blue",   (255,0,0))
        draw_boxes(red_u8,    "Red",    (0,0,255))
        draw_boxes(green_u8,  "Green",  (0,255,0))

        # --- Single display window ---
        cv2.imshow("Camera Feed + Bounding Boxes", frame_disp)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

finally:
    cv2.destroyAllWindows()
    picam2.stop()
    picam2.close()
