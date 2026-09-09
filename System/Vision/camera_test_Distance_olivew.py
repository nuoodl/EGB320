import time
import cv2
import numpy as np
from picamera2 import Picamera2

# ---------------------------------------------------------
#  Config / constants
# ---------------------------------------------------------
HEADLESS = False          # True: no GUI, text output only
MAX_FPS  = 10            # cap processing to 10 FPS

CROP_Y      = 734        # vanishing line (floor / wall intersection)
DARK_THRESH = 85         # brightness threshold for dark (maze bars)
MIN_AREA    = 400         # minimum contour area

H_CAM = 96.5             # camera height (mm or arbitrary units)
F_PX  = (12.3 / 36.0) * 2304  # focal length in pixels (approx)

# ---------------------------------------------------------
#  Camera setup
# ---------------------------------------------------------
picam2 = Picamera2()
config = picam2.create_video_configuration(
    main={"format": "RGB888", "size": (2304, 1296)},
    controls={"FrameRate": 60}
)
picam2.configure(config)

picam2.set_controls({
    "AeEnable": False,
    "AwbEnable": False,
    "AnalogueGain": 1.0,
    "ExposureTime": 10000,
    "ColourGains": (1.75, 2.7),
    "Saturation": 1.5,
    "ColourCorrectionMatrix": np.eye(3, dtype=np.float32).flatten().tolist()
})

picam2.start()
time.sleep(2.0)

# Pre-allocate / reuse kernel
kernel_h = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 40))

# ---------------------------------------------------------
#  Main loop
# ---------------------------------------------------------
try:
    last_time = time.time()

    while True:
        # FPS cap
        now = time.time()
        dt = now - last_time
        if dt < 1.0 / MAX_FPS:
            continue
        last_time = now

        # Capture frame (uint8)
        full_frame = picam2.capture_array()
        frame_disp = full_frame.copy() if not HEADLESS else None

        fps = 1.0 / dt

        # -----------------------------------------------------
        #  ROI below vanishing line
        # -----------------------------------------------------
        roi = full_frame[CROP_Y:, :, :]   # (H_roi, W, 3)
        R = roi[:, :, 0].astype(np.float32)
        G = roi[:, :, 1].astype(np.float32)
        B = roi[:, :, 2].astype(np.float32)

        # -----------------------------------------------------
        #  Brightness-normalised RGB
        # -----------------------------------------------------
        L = np.sqrt(R * R + G * G + B * B)
        L[L == 0] = 1e-6

        Rn = R / L
        Gn = G / L
        Bn = B / L

        # -----------------------------------------------------
        #  Colour masks (ROI)
        # -----------------------------------------------------
        yellow_mask = (Rn < 0.4) & (Gn > 0.5) & (Bn > 0.66)
        blue_mask   = (Rn > 0.67) & (Gn < 0.6) & (Bn < 0.34)
        red_mask    = (Rn < 0.55) & (Gn < 0.52) & (Bn > 0.71)
        green_mask  = (Rn > 0.54) & (Gn > 0.61) & (Bn < 0.88)

        yellow_u8 = (yellow_mask.astype(np.uint8) * 255)
        blue_u8   = (blue_mask.astype(np.uint8)   * 255)
        red_u8    = (red_mask.astype(np.uint8)    * 255)
        green_u8  = (green_mask.astype(np.uint8)  * 255)

        # -----------------------------------------------------
        #  Dark mask (maze vertical bars) on ROI
        # -----------------------------------------------------
        dark_mask = (L < DARK_THRESH).astype(np.uint8) * 255
        dark_mask = cv2.erode(dark_mask, kernel_h)

        H_roi, W = dark_mask.shape
        v_h = CROP_Y

        detections = []

        # -----------------------------------------------------
        #  Bounding box helper
        # -----------------------------------------------------
        def process_mask(mask_u8, label, colour_bgr):
            contours, _ = cv2.findContours(mask_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < MIN_AREA:
                    continue

                x, y, w, h = cv2.boundingRect(cnt)
                cx = x + w // 2
                cy_local = y + h // 2

                y_global = y + CROP_Y
                cy_global = cy_local + CROP_Y

                bearing = cx - (W // 2)

                feature_y = y_global + h
                dy = feature_y - v_h
                if dy > 1:
                    distance = 2.0 * (H_CAM * F_PX) / dy
                else:
                    distance = 0.0

                detections.append((label, bearing, distance))

                if not HEADLESS and frame_disp is not None:
                    cv2.rectangle(frame_disp,
                                  (x, y_global),
                                  (x + w, y_global + h),
                                  colour_bgr,
                                  2)
                    cv2.putText(frame_disp,
                                f"{label} b={bearing} d={distance:.1f}",
                                (x, max(0, y_global - 10)),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.6,
                                colour_bgr,
                                2)

        # -----------------------------------------------------
        #  Process all masks
        # -----------------------------------------------------
        process_mask(yellow_u8, "Yellow",   (0, 255, 255))
        process_mask(blue_u8,   "Blue",     (255, 0, 0))
        process_mask(red_u8,    "Red",      (0, 0, 255))
        process_mask(green_u8,  "Green",    (0, 255, 0))
        process_mask(dark_mask, "MazeEdge", (255, 255, 128))

        # -----------------------------------------------------
        #  Output (headless vs GUI)
        # -----------------------------------------------------
        if HEADLESS:
            # Shared-memory / IPC-friendly text output
            print(f"FPS: {fps:.2f}")
            for obj, bearing, dist in detections:
                print(f"{obj:10s}  bearing={bearing:5d}  dist={dist:8.2f}")
            print("-" * 40)
        else:
            # Draw vanishing line
            cv2.line(frame_disp, (0, v_h), (W, v_h), (255, 255, 0), 2)

            cv2.imshow("Camera Feed + Bounding Boxes", frame_disp)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

finally:
    if not HEADLESS:
        cv2.destroyAllWindows()
    picam2.stop()
    picam2.close()
