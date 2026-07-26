"""
Ball-on-Plate Simulation
-------------------------------------------------------------------
Pure-Python simulation of 2-axis PID control balancing a ball on a
tilting plate, driven by two servos (X-axis, Y-axis).  Swapping in real
hardware later: replacie the "read position" / "command
servo" calls in the control loop with serial reads/writes.


"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ---------------------------------------------------------------
# Physical constants
# ---------------------------------------------------------------
G = 9.81                       # m/s^2
BALL_INERTIA_FACTOR = 5 / 7    # solid sphere, rolling without slipping
PLATE_HALF_SIZE = 0.30         # m -- plate is 20cm x 20cm
BALL_RADIUS = 0.01             # m -- 20mm ball

SERVO_MAX_ANGLE = 30.0         # deg -- mechanical clamp
SERVO_MAX_RATE = 300.0         # deg/s -- approx max slew rate, MG996R

# ---------------------------------------------------------------
# Simulation settings
# ---------------------------------------------------------------
SAMPLE_RATE = 200              # Hz -- control loop rate
SIM_LENGTH = 8                 # s
DT = 1.0 / SAMPLE_RATE


class PID:
    """Standard PID with output clamping + basic anti-windup."""

    def __init__(self, kp, ki, kd, setpoint=0.0, output_limits=(None, None)):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.setpoint = setpoint
        self.error = 0.0
        self.integral = 0.0
        self.prev_error = 0.0
        self.output_limits = output_limits

    def compute(self, position, dt):
        self.error = self.setpoint - position
        self.integral += self.error * dt
        derivative = (self.error - self.prev_error) / dt if dt > 0 else 0.0
        output = self.kp * self.error + self.ki * self.integral + self.kd * derivative

        lo, hi = self.output_limits
        if lo is not None or hi is not None:
            clipped = float(np.clip(output, lo, hi))
            # anti-windup: stop integrating further in the saturated direction
            if clipped != output and self.ki != 0:
                self.integral -= self.error * dt
            output = clipped

        self.prev_error = self.error
        return output


class Servo:
    """
    Angle-clamped, rate-limited actuator model.

    A real servo can't (a) rotate past its mechanical limit or (b) snap
    instantly to a new angle -- it moves at some max deg/s. Both matter
    for this sim: without the angle clamp, a big PID output would command
    an unrealistic angle and, once past 90 degrees, sin(angle) starts
    folding back on itself, which flips the sign of the restoring force
    and makes the system diverge. Without the rate limit, the ball would
    react to instantaneous tilt changes a physical servo could never
    actually produce.
    """

    def __init__(self, max_angle=SERVO_MAX_ANGLE, max_rate=SERVO_MAX_RATE):
        self.angle = 0.0          # current commanded angle, deg (starts level)
        self.max_angle = max_angle
        self.max_rate = max_rate

    def command(self, target_angle, dt):
        # Step 1: clamp the angle to what the servo can physically reach.
        target_angle = float(np.clip(target_angle, -self.max_angle, self.max_angle))

        # Step 2: limit how far the servo can move this timestep, based on its
        # max slew rate. E.g. at 300 deg/s and dt=1/200s, it can move at most
        # 1.5 deg per step, regardless of how far away target_angle is.
        max_step = self.max_rate * dt
        delta = float(np.clip(target_angle - self.angle, -max_step, max_step))

        # Step 3: apply that bounded step to the servo's current angle.
        self.angle += delta
        return self.angle


def run_simulation(kpx=120, kix=0.5, kdx=15, kpy=80, kiy=0.5, kdy=15,
                    start_x=0.09, start_y=-0.08):
    """
    Run one full simulation and return the time history.

    X and Y are controlled independently -- each axis gets its own PID
    and its own servo, since tilting the plate about the X axis only
    affects the ball's X motion (and same for Y). 

    start_x/start_y set how far off-center the ball starts (meters),
    so you can see how the controller recovers from a disturbance.
    """
    # Two independent PID loops, one per axis. 
    x_pid = PID(kpx, kix, kdx, setpoint=0.0,
                output_limits=(-SERVO_MAX_ANGLE, SERVO_MAX_ANGLE))
    y_pid = PID(kpy, kiy, kdy, setpoint=0.0,
                output_limits=(-SERVO_MAX_ANGLE, SERVO_MAX_ANGLE))

    # Two independent servo models, one per axis.
    servo_x = Servo()
    servo_y = Servo()

    # Ball's starting position (m) and velocity (m/s) on the plate.
    x, y = start_x, start_y
    vx, vy = 0.2, -0.2

    # Log everything so it can be plotted/animated after the loop.
    history = {"t": [], "x": [], "y": [], "servo_x": [], "servo_y": []}
    t = 0.0
    n_steps = int(SIM_LENGTH * SAMPLE_RATE)

    for _ in range(n_steps):
        # --- control loop (this is the part that talks to Arduino
        #     over serial once hardware exists) ---
        # Each PID looks at the ball's current position on its axis and
        # returns a desired tilt angle (deg) to drive position to 0.
        x_cmd = x_pid.compute(x, DT)
        y_cmd = y_pid.compute(y, DT)
        # Hand that desired angle to the servo model, which clamps/rate-limits
        # it to what the real actuator could do.
        servo_x.command(x_cmd, DT)
        servo_y.command(y_cmd, DT)

        # --- plate/ball physics ---
        ax = BALL_INERTIA_FACTOR * G * np.sin(np.radians(servo_x.angle))
        ay = BALL_INERTIA_FACTOR * G * np.sin(np.radians(servo_y.angle))

        # Basic Euler integration: acceleration -> velocity -> position.
        vx += ax * DT
        vy += ay * DT
        vx *= 0.999   # rolling friction -- small per-step velocity decay
        vy *= 0.999

        x += vx * DT
        y += vy * DT

        # Record this step for the plots/animation.
        t += DT
        history["t"].append(t)
        history["x"].append(x)
        history["y"].append(y)
        history["servo_x"].append(servo_x.angle)
        history["servo_y"].append(servo_y.angle)

        # Stop early if the ball rolls off the edge of the plate.
        if abs(x) > PLATE_HALF_SIZE or abs(y) > PLATE_HALF_SIZE:
            print(f"Ball fell off the plate at t={t:.3f}s")
            break

    return history


if __name__ == "__main__":
    history = run_simulation()
    # if you want to tune PID gains, call run_simulation() with different kpx/kix/kdx/kpy/kiy/kdy values
    # --- static response plots ---
    fig, axs = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    axs[0].plot(history["t"], history["x"], label="X position (m)")
    axs[0].plot(history["t"], history["y"], label="Y position (m)")
    axs[0].axhline(0, color="gray", linewidth=0.5)
    axs[0].set_ylabel("Position (m)")
    axs[0].legend()
    axs[0].grid(True)

    axs[1].plot(history["t"], history["servo_x"], label="Servo X angle (deg)")
    axs[1].plot(history["t"], history["servo_y"], label="Servo Y angle (deg)")
    axs[1].set_xlabel("Time (s)")
    axs[1].set_ylabel("Servo angle (deg)")
    axs[1].legend()
    axs[1].grid(True)

    plt.tight_layout()
    plt.savefig("ball_on_plate_response.png", dpi=150)
    print("Saved response plot to ball_on_plate_response.png")

    # --- animated top-down view ---
    fig2, ax2 = plt.subplots(figsize=(5, 5))
    lim = PLATE_HALF_SIZE * 1.2
    ax2.set_xlim(-lim, lim)
    ax2.set_ylim(-lim, lim)
    ax2.set_aspect("equal")
    ax2.set_title("Ball-on-Plate (top-down view)")

    plate_outline = plt.Rectangle(
        (-PLATE_HALF_SIZE, -PLATE_HALF_SIZE),
        2 * PLATE_HALF_SIZE, 2 * PLATE_HALF_SIZE,
        fill=False, edgecolor="black", linewidth=2,
    )
    ax2.add_patch(plate_outline)
    ax2.plot([0], [0], "+", color="gray")
    ball_dot, = ax2.plot([], [], "o", color="crimson", markersize=12)

    frame_stride = max(1, len(history["t"]) // 300)
    frame_indices = list(range(0, len(history["t"]), frame_stride))

    def update(frame_i):
        idx = frame_indices[frame_i]
        ball_dot.set_data([history["x"][idx]], [history["y"][idx]])
        return ball_dot,

    ani = animation.FuncAnimation(
        fig2, update, frames=len(frame_indices), interval=30, blit=True
    )

    try:
        ani.save("ball_on_plate_animation.gif", writer="pillow", fps=30)
        print("Saved animation to ball_on_plate_animation.gif")
    except Exception as e:
        print(f"Could not save animation gif ({e}); showing live instead.")

    plt.show()
