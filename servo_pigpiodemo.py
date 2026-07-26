import pigpio
import time

SERVO_PIN = 18

pi = pigpio.pi()

if not pi.connected:
    raise RuntimeError("Could not connect to pigpio daemon.")

try:
    while True:
        print("0°")
        pi.set_servo_pulsewidth(SERVO_PIN, 500)    # 0°
        time.sleep(3)

        print("180°")
        pi.set_servo_pulsewidth(SERVO_PIN, 2500)   # 180°
        time.sleep(3)

except KeyboardInterrupt:
    print("Stopping...")

finally:
    # Stop sending pulses
    pi.set_servo_pulsewidth(SERVO_PIN, 0)
    pi.stop()