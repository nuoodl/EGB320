import cv2
from picamera2 import Picamera2
import picamera2

cap = picamera2.Picamera2()

config = cap.create_video_configuration(main={"format": "RGB888", "size": (640, 480)})
cap.configure(config)
cap.set.controls({"ExposureTime": 100000, "AnalogueGain": 1.0, "ColourGains": (1.4, 1.5)})

cap.start()
frame = cap.capture_array()

cap.close()
