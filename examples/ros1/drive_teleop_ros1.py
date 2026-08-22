#!/usr/bin/env python3
"""Nano-sim ROS 1 keyboard teleop.

Publishes geometry_msgs/Twist on /cmd_vel at 20 Hz. rosbridge relays it to
Unity, which maps linear.x -> forward speed (m/s) and angular.z -> yaw rate
(rad/s, + = left). Keys change the latched command; the timer keeps publishing
it, which also satisfies Unity's 0.5 s watchdog (it stops the car on silence).

Make sure /cmd_vel is ENABLED in the Unity topic config and that Unity is
connected to this machine's rosbridge (ws://<host>:9090).

Run from a real terminal (needs a TTY):
    rosrun <your_pkg> nanosim_teleop_ros1.py

Controls
    w / UP      speed +             s / DOWN    speed -
    a / LEFT    yaw left            d / RIGHT   yaw right
    space       full stop           x           straighten
    q / Ctrl-C  quit

POSIX only (macOS / Linux), matching the shipped Nano-sim binaries.
"""
import math
import select
import shutil
import sys
import termios
import tty

import rospy
from geometry_msgs.msg import Twist

RATE_HZ = 20.0          # comfortably above the 0.5 s command timeout
MAX_SPEED = 7.0         # m/s  (linear.x ceiling)
SPEED_STEP = 0.5        # m/s per key press (14 taps to the ceiling)
STEER_STEP = 0.10       # per key press, normalised -1..1 (matches the Python API)

# /cmd_vel carries a yaw RATE, not a steering angle, so a normalised steering
# command has to be converted with the CURRENT speed (bicycle model).
MAX_STEER_DEG = 26.0    # measured full lock on the LaTrax Rally
WHEELBASE = 0.165       # m

ARROWS = {"[A": "UP", "[B": "DOWN", "[C": "RIGHT", "[D": "LEFT"}


def clamp(v, limit):
    return max(-limit, min(limit, v))


STATUS_LINES = 2        # sticky block pinned to the bottom of the scroll region

# Kept to 79 chars so it survives _fit() on an 80-column terminal — a clipped
# legend defeats the whole point of pinning it.
LEGEND = ("[w/s] speed  [a/d] steer  [space] stop  "
          "[x] centre  [h] help  [q] quit")


def _fit(s):
    """Truncate to the terminal width. A wrapped line would consume an extra
    screen row, which breaks the cursor-up arithmetic and lets the block drift."""
    return s[:max(1, shutil.get_terminal_size((100, 24)).columns - 1)]


def _erase_block():
    """Wipe the sticky block, leaving the cursor at column 0 of its first row."""
    sys.stdout.write("\r\x1b[K")
    for _ in range(STATUS_LINES - 1):
        sys.stdout.write("\n\x1b[K")
    if STATUS_LINES > 1:
        sys.stdout.write("\x1b[%dA" % (STATUS_LINES - 1))
    sys.stdout.write("\r")


def say(msg):
    """Emit one scrolling log line ABOVE the sticky block.

    cbreak mode does not translate \\n, so lines end with CR+LF.
    """
    _erase_block()
    sys.stdout.write(msg + "\r\n")
    sys.stdout.flush()


def draw_status(lines):
    """Redraw the sticky block in place; the cursor returns to its first row."""
    _erase_block()
    sys.stdout.write("\r\n".join(_fit(l) for l in lines))
    if len(lines) > 1:
        sys.stdout.write("\x1b[%dA" % (len(lines) - 1))
    sys.stdout.write("\r")
    sys.stdout.flush()


HELP = [
    ("w / UP", "linear.x   +%.2f m/s   (max %.2f)" % (SPEED_STEP, MAX_SPEED)),
    ("s / DOWN", "linear.x   -%.2f m/s   (negative = reverse)" % SPEED_STEP),
    ("a / LEFT", "steer left   %.2f   (-1..1, lock %.0f deg)" % (STEER_STEP, MAX_STEER_DEG)),
    ("d / RIGHT", "steer right  %.2f   (ROS: + = left)" % STEER_STEP),
    (None, None),
    ("space", "full stop (speed + steering -> 0)"),
    ("x", "centre steering only"),
    (None, None),
    ("h", "show these keys again"),
    ("q  /  ^C", "quit"),
]


def print_help():
    """Draw the key map as an aligned, boxed block — readable at a glance."""
    width = 60
    say("+" + "-" * width + "+")
    say("|" + " NANO-SIM  ROS 1  KEYBOARD TELEOP ".center(width) + "|")
    say("|" + ("/cmd_vel @ %.0f Hz" % RATE_HZ).center(width) + "|")
    say("|" + ("angular.z = v * tan(steer * %.0f deg) / %.3f m"
              % (MAX_STEER_DEG, WHEELBASE)).center(width) + "|")
    say("+" + "-" * width + "+")
    for key, desc in HELP:
        if key is None:
            say("|" + " " * width + "|")
        else:
            say("|  %-12s  %-*s|" % (key, width - 16, desc))
    say("+" + "-" * width + "+")


def yaw_rate(speed, steer):
    """Normalised steering (-1..1) -> angular.z in rad/s.

        delta = steer * MAX_STEER_DEG
        omega = v * tan(delta) / L

    Commanding a fixed yaw rate instead is what stopped full steer from
    reaching full lock: the angle a given omega produces is atan(omega*L/v),
    which shrinks as speed rises — 2 rad/s is 18 deg at 1 m/s but only 2.7 deg
    at 7 m/s. Converting here keeps +/-1 at full lock for every speed.

    At v = 0 the result is 0: an Ackermann chassis cannot rotate in place.
    """
    return speed * math.tan(math.radians(steer * MAX_STEER_DEG)) / WHEELBASE


def bar(value, limit, half=6):
    """Centre-anchored ASCII bar for a signed axis: '--|###---' style."""
    cells = ["-"] * (half * 2 + 1)
    cells[half] = "|"
    step = 1 if value > 0 else -1
    for i in range(1, int(round(abs(value) / limit * half)) + 1):
        cells[half + i * step] = "#"
    return "".join(cells)


class KeyReader:
    """Non-blocking single-key reader for POSIX terminals."""

    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.saved = termios.tcgetattr(self.fd)

    def __enter__(self):
        tty.setcbreak(self.fd)      # unbuffered, but ISIG stays on so Ctrl-C works
        return self

    def __exit__(self, *exc):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)

    def get(self, timeout=0.0):
        """Return one key ('w', 'UP', ...) or None if nothing arrived in time."""
        if not select.select([sys.stdin], [], [], timeout)[0]:
            return None
        ch = sys.stdin.read(1)
        if ch != "\x1b":
            return ch
        if not select.select([sys.stdin], [], [], 0.002)[0]:
            return "ESC"
        return ARROWS.get(sys.stdin.read(2))


def main():
    if not sys.stdin.isatty():
        sys.exit("keyboard teleop needs a real terminal (stdin is not a TTY)")

    rospy.init_node("nanosim_teleop", disable_signals=True)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rate = rospy.Rate(RATE_HZ)

    speed = 0.0
    steer = 0.0           # normalised -1..1

    with KeyReader() as kb:
        print_help()
        try:
            while not rospy.is_shutdown():
                # Drain everything buffered since the last publish.
                while True:
                    key = kb.get(0.0)
                    if key is None:
                        break
                    if key in ("q", "\x03"):
                        raise KeyboardInterrupt
                    elif key in ("w", "UP"):
                        speed = clamp(speed + SPEED_STEP, MAX_SPEED)
                    elif key in ("s", "DOWN"):
                        speed = clamp(speed - SPEED_STEP, MAX_SPEED)
                    elif key in ("a", "LEFT"):
                        steer = clamp(steer + STEER_STEP, 1.0)   # ROS: + = left
                    elif key in ("d", "RIGHT"):
                        steer = clamp(steer - STEER_STEP, 1.0)
                    elif key == " ":
                        speed = steer = 0.0
                    elif key == "x":
                        steer = 0.0
                    elif key == "h":
                        print_help()

                yaw = yaw_rate(speed, steer)
                cmd = Twist()
                cmd.linear.x = speed
                cmd.angular.z = yaw
                pub.publish(cmd)

                draw_status([
                    LEGEND,
                    f"SPD {bar(speed, MAX_SPEED)} {speed:+.2f} m/s  "
                    f"STR {bar(steer, 1.0)} {steer:+.2f}  "
                    f"yaw {yaw:+6.2f} rad/s",
                ])
                rate.sleep()
        except (KeyboardInterrupt, rospy.ROSInterruptException):
            pass

    # Leave the car stopped rather than waiting out the watchdog.
    try:
        pub.publish(Twist())
        rospy.sleep(0.1)
    except rospy.ROSException:
        pass
    sys.stdout.write("\n" * STATUS_LINES)   # step past the sticky block
    print("stopped.")


if __name__ == "__main__":
    main()
