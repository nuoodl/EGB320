import time
from controller import Controller

motor_driver = Controller()


def test_motor_driving(speed_left, speed_right, duration):
    """
    Test the motors by running them at specified speeds for a given duration.
    
    :param speed_left: Speed for the left motor (-127 to 127)
    :param speed_right: Speed for the right motor (-127 to 127)
    :param duration: Duration to run the motors in seconds
    """
    try:
        # Read the initial encoder baseline
        start_left, start_right = motor_driver.get_encoder_ticks()

        print("Starting motor test")
        print("Press Ctrl+C to stop")

        # Command the motors
        motor_driver.set_raw_motor_speed(speed_left, speed_right)

        steps = max(1, int(duration * 10))
        for _ in range(steps):
            current_left, current_right = motor_driver.get_encoder_ticks()

            # Print relative ticks since start
            print(
                f"Left Ticks Delta: {current_left - start_left} | "
                f"Right Ticks Delta: {current_right - start_right}"
            )

            time.sleep(0.1)

    finally:
        motor_driver.set_raw_motor_speed(0, 0)
        print("Motors stopped")


def test_motor_turning(turn_direction, duration=0.45):
    """
    Test turning by driving motors in opposite directions.
    
    :param turn_direction: Turn indicator (<= 1 for Left turn, > 1 for Right turn)
    :param duration: How long to run the turn in seconds
    """
    try:
        start_left, start_right = motor_driver.get_encoder_ticks()

        print("Starting turn test")
        print("Press Ctrl+C to stop")

        # Determine direction based on parameter
        if turn_direction <= 1:
            speed_left = -120
            speed_right = 120
        else:
            speed_left = 120
            speed_right = -120

        # Command the motors
        motor_driver.set_raw_motor_speed(speed_left, speed_right)

        steps = max(1, int(duration * 10))
        for _ in range(steps):
            current_left, current_right = motor_driver.get_encoder_ticks()

            print(
                f"Left Ticks Delta: {current_left - start_left} | "
                f"Right Ticks Delta: {current_right - start_right}"
            )

            time.sleep(0.1)

    finally:
        motor_driver.set_raw_motor_speed(0, 0)
        print("Motors stopped")


if __name__ == "__main__":
    try:
        # Call with 1 argument (or pass duration as second parameter if needed)
        test_motor_turning(1, duration=0.45)
    finally:
        motor_driver.set_raw_motor_speed(0, 0)