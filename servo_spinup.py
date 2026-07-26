# python servo demo
from gpiozero import AngularServo
from time import sleep

servo = AngularServo(
    18,
    min_angle=0,
    max_angle=180,
    min_pulse_width=0.0005,   # 0.5 ms
    max_pulse_width=0.0025    # 2.5 ms
)

while True:
    servo.angle = 0
    sleep(3)

    servo.angle = 180
    sleep(3)