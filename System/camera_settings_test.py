import time
import cv2
from picamera2 import Picamera2


# Select ONE resolution
# resolution = (640, 480)
# resolution = (820, 616)
resolution = (2304, 1296)
#resolution = (1920, 1080)      # Full HD, more processing required

# Select ONE frame rate
frame_rate = 30
# frame_rate = 20
# frame_rate = 10


camera = Picamera2()

config = camera.create_video_configuration(
    main={"format": "XRGB8888", "size": resolution},
    controls={"FrameRate": frame_rate}
)

camera.configure(config)


# Automatic camera settings
camera.set_controls({
    "AeEnable": True,
    "AwbEnable": False
})


# Manual camera settings
# Comment out the automatic settings above before using these.
#
# camera.set_controls({
#     "AeEnable": False,
#     "AwbEnable": False,
#     "ExposureTime": 10000,
#     "AnalogueGain": 1.5,
#     "ColourGains": (1.4, 1.5)
# })


camera.start()
time.sleep(2)

try:
    while True:
        frame = camera.capture_array()
        frame_low = cv2.resize(frame, (640,480), interpolation=cv2.INTER_AREA)
        cv2.imshow("EGB320 Camera Test", frame_low)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

finally:
    cv2.destroyAllWindows()
    camera.stop()
    camera.close()