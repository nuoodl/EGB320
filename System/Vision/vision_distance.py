"""
EGB320 Vision - distance and bearing detection
==============================================
Olive's camera_test_Distance pipeline, made importable.

WHAT CHANGED FROM THE ORIGINAL, AND NOTHING ELSE:
    - the setup and the while-loop are wrapped in a class so another
      subsystem can call it instead of it running on import
    - the standalone GUI/headless demo now lives under
      `if __name__ == "__main__"`, so importing this file no longer
      starts a camera loop
    - detections are returned instead of only printed

All constants, colour masks, the dark-mask erode and the distance maths
are byte-for-byte Olive's. If she retunes anything, this file is what
gets updated - navigation does not keep its own copy.

OUTPUT FORMAT (unchanged from the original):
    (label, bearing_px, distance_mm)
      label       "Yellow" | "Blue" | "Red" | "Green" | "MazeEdge"
      bearing_px  pixels right of image centre, NEGATIVE = left
      distance_mm millimetres (H_CAM is in mm)

    Navigation converts these to radians and metres at its end.

RUN STANDALONE (unchanged behaviour):
    python3 vision_distance.py
"""

import time

import cv2
import numpy as np
from picamera2 import Picamera2

# ---------------------------------------------------------
#  Config / constants - Olive's, unchanged
# ---------------------------------------------------------
HEADLESS = False          # True: no GUI, text output only
MAX_FPS = 10              # cap processing to 10 FPS

CROP_Y = 734              # vanishing line (floor / wall intersection)
DARK_THRESH = 85          # brightness threshold for dark (maze bars)
MIN_AREA = 400            # minimum contour area

H_CAM = 96.5              # camera height (mm)
F_PX = (12.3 / 36.0) * 2304   # focal length in pixels (approx)

RESOLUTION = (2304, 1296)


class VisionDistance:
    """Wraps the camera and one frame of Olive's detection pipeline."""

    def __init__(self):
        self.picam2 = Picamera2()
        config = self.picam2.create_video_configuration(
            main={"format": "RGB888", "size": RESOLUTION},
            controls={"FrameRate": 60}
        )
        self.picam2.configure(config)

        self.picam2.set_controls({
            "AeEnable": False,
            "AwbEnable": False,
            "AnalogueGain": 1.0,
            "ExposureTime": 10000,
            "ColourGains": (1.75, 2.7),
            "Saturation": 1.5,
            "ColourCorrectionMatrix": np.eye(3, dtype=np.float32).flatten().tolist()
        })

        self.picam2.start()
        time.sleep(2.0)

        self.kernel_h = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 40))

    def get_detections(self, draw_on=None):
        """Process one frame.

        Returns a list of (label, bearing_px, distance_mm).
        If draw_on is a frame copy, bounding boxes are drawn onto it."""
        full_frame = self.picam2.capture_array()
        frame_disp = full_frame.copy() if draw_on is not None else None

        roi = full_frame[CROP_Y:, :, :]
        R = roi[:, :, 0].astype(np.float32)
        G = roi[:, :, 1].astype(np.float32)
        B = roi[:, :, 2].astype(np.float32)

        L = np.sqrt(R * R + G * G + B * B)
        L[L == 0] = 1e-6

        Rn = R / L
        Gn = G / L
        Bn = B / L

        yellow_mask = (Rn < 0.4) & (Gn > 0.5) & (Bn > 0.66)
        blue_mask = (Rn > 0.67) & (Gn < 0.6) & (Bn < 0.34)
        red_mask = (Rn < 0.55) & (Gn < 0.52) & (Bn > 0.71)
        green_mask = (Rn > 0.54) & (Gn > 0.61) & (Bn < 0.88)

        yellow_u8 = (yellow_mask.astype(np.uint8) * 255)
        blue_u8 = (blue_mask.astype(np.uint8) * 255)
        red_u8 = (red_mask.astype(np.uint8) * 255)
        green_u8 = (green_mask.astype(np.uint8) * 255)

        dark_mask = (L < DARK_THRESH).astype(np.uint8) * 255
        dark_mask = cv2.erode(dark_mask, self.kernel_h)

        H_roi, W = dark_mask.shape
        v_h = CROP_Y

        detections = []

        def process_mask(mask_u8, label, colour_bgr):
            contours, _ = cv2.findContours(mask_u8, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < MIN_AREA:
                    continue

                x, y, w, h = cv2.boundingRect(cnt)
                cx = x + w // 2
                y_global = y + CROP_Y

                bearing = cx - (W // 2)

                feature_y = y_global + h
                dy = feature_y - v_h
                if dy > 1:
                    distance = 2.0 * (H_CAM * F_PX) / dy
                else:
                    distance = 0.0

                detections.append((label, bearing, distance))

                if frame_disp is not None:
                    cv2.rectangle(frame_disp, (x, y_global),
                                  (x + w, y_global + h), colour_bgr, 2)
                    cv2.putText(frame_disp,
                                f"{label} b={bearing} d={distance:.1f}",
                                (x, max(0, y_global - 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour_bgr, 2)

        process_mask(yellow_u8, "Yellow", (0, 255, 255))
        process_mask(blue_u8, "Blue", (255, 0, 0))
        process_mask(red_u8, "Red", (0, 0, 255))
        process_mask(green_u8, "Green", (0, 255, 0))
        process_mask(dark_mask, "MazeEdge", (255, 255, 128))

        if frame_disp is not None:
            cv2.line(frame_disp, (0, v_h), (W, v_h), (255, 255, 0), 2)
            self.last_frame = frame_disp

        return detections

    def close(self):
        self.picam2.stop()
        self.picam2.close()


# ---------------------------------------------------------
#  Standalone demo - the original script's behaviour
# ---------------------------------------------------------
if __name__ == "__main__":
    vision = VisionDistance()
    try:
        last_time = time.time()
        while True:
            now = time.time()
            dt = now - last_time
            if dt < 1.0 / MAX_FPS:
                time.sleep(0.001)
                continue
            last_time = now
            fps = 1.0 / dt

            detections = vision.get_detections(
                draw_on=None if HEADLESS else True)

            if HEADLESS:
                print(f"FPS: {fps:.2f}")
                for obj, bearing, dist in detections:
                    print(f"{obj:10s}  bearing={bearing:5d}  dist={dist:8.2f}")
                print("-" * 40)
            else:
                cv2.imshow("Camera Feed + Bounding Boxes", vision.last_frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    finally:
        if not HEADLESS:
            cv2.destroyAllWindows()
        vision.close()
