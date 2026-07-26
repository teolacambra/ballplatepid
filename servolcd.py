import pigpio
import time
from RPLCD import CharLCD
from RPi import GPIO

GPIO.setwarnings(False)

# -------------------------------
# LCD Setup
# -------------------------------

lcd = CharLCD(
    pin_rs=25,
    pin_e=24,
    pins_data=[23, 17, 27, 22],
    numbering_mode=GPIO.BCM,
    cols=16,
    rows=2
)

# -------------------------------
# Servo Setup
# -------------------------------

SERVO_X = 18
SERVO_Y = 16

pi = pigpio.pi()

if not pi.connected:
    raise RuntimeError("Could not connect to pigpiod.")

# -------------------------------
# Functions
# -------------------------------

def angle_to_pulse(angle):
    angle = max(0, min(180, angle))
    return 1000 + (angle / 180.0) * 1000


def move_servo(pin, angle):
    pulse = angle_to_pulse(angle)
    pi.set_servo_pulsewidth(pin, pulse)


def update_display(x_angle, y_angle):

    lcd.clear()

    lcd.cursor_pos = (0, 0)
    lcd.write_string(f"Servo X: {x_angle:3d}")

    lcd.cursor_pos = (1, 0)
    lcd.write_string(f"Servo Y: {y_angle:3d}")


# -------------------------------
# Main Program
# -------------------------------

angles = [0, 45, 90, 135, 180]

try:

    while True:

        # Forward
        for angle in angles:

            move_servo(SERVO_X, angle)
            move_servo(SERVO_Y, angle)

            update_display(angle, angle)

            print(f"Servo X = {angle}°")
            print(f"Servo Y = {angle}°")

            time.sleep(2)

        # Reverse
        for angle in reversed(angles[:-1]):

            move_servo(SERVO_X, angle)
            move_servo(SERVO_Y, angle)

            update_display(angle, angle)

            print(f"Servo X = {angle}°")
            print(f"Servo Y = {angle}°")

            time.sleep(2)

except KeyboardInterrupt:
    print("Stopping...")

finally:

    pi.set_servo_pulsewidth(SERVO_X, 0)
    pi.set_servo_pulsewidth(SERVO_Y, 0)

    lcd.clear()
    lcd.write_string("Program Ended")

    pi.stop()

    GPIO.cleanup()