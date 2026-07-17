# arduino library importation
import serial 
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd 
serial_port = 'COM3'  # Replace with your Arduino's serial port
print(serial_port)
baud_rate = 9600  # Match the baud rate set in your Arduino sketch

servo1_pin = 10
servo2_pin = 11

# tilt servo 1 10 degrees
#tilt it back


# tilt servo 2 10 degrees
# tilt it back

#tilt servo 1 10 degrees
#tilt servo 2 10 degrees
# tilt them back
#optimization consideration; instead of x(full) and y(full), move them in small increments and record the data at each increment. This will allow for a more detailed analysis of the servo movements and their effects on the system.

class PID(object):
    def __init__(self, kp, ki, kd, desired_distance):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = 0
        self.error = 0
        self.integralerror = 0
        self.derivativeerror = 0
        self.prev_error = 0
        self.output = 0
    def compute(self, position):
        self.error = self.setpoint - position
        self.integralerror += self.error * dt
        self.derivativeerror = (self.error - self.prev_error) / dt if dt > 0 else 0
        self.output = (self.kp * self.error) + (self.ki * self.integralerror) + (self.kd * self.derivativeerror)
        self.prev_error = self.error
        return self.output

#PID_P = kp * distance_error (rotate the servo by this amount) (kp encapsulates angle-to-distance conversion, complex dynamics between the servo and the system, and any other factors that affect the relationship between the distance error and the required servo rotation angle.)
#PID_D = kd * (distance_error - distance_error_prev) / dt # ds/dt
#PID_I = ki * integral(distance_error) * elapsed_time # s * m/s = m

# servo1 angle, degrees
#adjust servo 1 angle by pid_pd
# servo2 angle, degrees
#adjust servo 2 angle by pid_pd

  # Time in seconds - increment by the arduino tick 
sr = 2000 # sample rate in Hz, 2000 samples per second
simlength = 5  # Simulation length in seconds
dt = 1/sr  # Time difference between samples in seconds 
xideal = 0
yideal = 0
desired_pos = [xideal, yideal]  # Desired distance in meters
kpx = 80  # Proportional gain
kdx = 0  # Derivative gain
kix = 0  # Integral gain
kpy = 0.1  # Proportional gain
kdy = 0  # Derivative gain
kiy = 0  # Integral gain
x_pid = PID(kpx, kix, kdx, xideal)
y_pid = PID(kpy, kiy, kdy, yideal)

x = 2.5
y = -0.8
vx = 0
vy = 0
servo1_angle = 0.0
servo2_angle = 0.0
servo1_angle = np.clip(servo1_angle, -30, 30)
servo2_angle = np.clip(servo2_angle, -30, 30)
total_time = 0  # Initialize total time in seconds

x_errors = []
y_errors = []
for i in range(simlength * sr):  # Loop for the duration of the simulation
    
    total_time += dt  # Increment total time by dt
    samples = total_time * sr  # Total number of samples
    # Simulate reading x and y values from the Arduino
    
    distance = np.array([x, y])  # Current distance in meters
    print(f"X: {x}, Y: {y}, Time: {total_time}")
    previous_time = total_time - dt  # Previous time in seconds
    print(f"Previous x error: {x_pid.prev_error}")
    print(f"Previous y error: {y_pid.prev_error}")
    x_adjust = x_pid.compute(x)
    y_adjust = y_pid.compute(y)
    servo1_angle = np.clip(x_adjust, -30, 30)
    servo2_angle = np.clip(y_adjust, -30, 30)
    print(f"X adjustment: {x_adjust}")
    print(f"Y adjustment: {y_adjust}")
    print(f"X error: {x_pid.error}")
    print(f"Y error: {y_pid.error}")
    x_errors.append(x_pid.error)
    y_errors.append(y_pid.error)
   
    # Move servos
    servo1_angle = x_adjust
    servo2_angle = y_adjust
    # Resultant dynamics of the system based on servo adjustments
    # For simplicity, let's assume the system responds linearly to servo adjustments
    ax = 9.81 * np.sin(np.radians(servo1_angle))
    ay = 9.81 * np.sin(np.radians(servo2_angle))

    vx += ax * dt
    vy += ay * dt

    vx *= 0.995
    vy *= 0.995

    x += vx * dt
    y += vy * dt
    print(f"x = {x:.6f}")
    print(f"error = {x_pid.error:.6f}")
    print(f"servo = {servo1_angle:.6f}")
    print(f"ax = {ax:.6f}")
    print(f"vx = {vx:.6f}")
plt.plot(x_errors)
plt.xlabel("Sample")
plt.ylabel("X Error")
plt.grid(True)
plt.show()


