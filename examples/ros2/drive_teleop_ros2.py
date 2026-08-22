#!/usr/bin/env python3
"""Nano-sim ROS 2 keyboard teleop.

Publishes geometry_msgs/Twist on /cmd_vel at 20 Hz. rosbridge relays it to
Unity, which maps linear.x -> forward speed (m/s) and angular.z -> yaw rate
(rad/s, + = left). Keys change the latched command; the timer keeps publishing
it, which also satisfies Unity's 0.5 s watchdog (it stops the car on silence).

Make sure /cmd_vel is ENABLED in the Unity topic config and that Unity is
connected to this machine's rosbridge (ws://<host>:9090).

Run from a real terminal (needs a TTY):
    ros2 run <your_pkg> nanosim_teleop_ros2

Controls
    w / UP      speed +             s / DOWN    speed -
    a / LEFT    yaw left            d / RIGHT   yaw right
    space       full stop           x           straighten
    q / Ctrl-C  quit

POSIX only (macOS / Linux), matching the shipped Nano-sim binaries.
"""
import select
import shutil
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

PERIOD = 0.05           # 20 Hz, comfortably above the 0.5 s command timeout
MAX_SPEED = 7.0         # m/s  (linear.x ceiling)
SPEED_STEP = 0.5        # m/s per key press (14 taps to the ceiling)
MAX_YAW = 2.0           # rad/s
YAW_STEP = 0.2          # rad/s per key press

ARROWS = {"[A": "UP", "[B": "DOWN", "[C": "RIGHT", "[D": "LEFT"}


def clamp(v, limit):
    return max(-limit, min(limit, v))


STATUS_LINES = 2        # sticky block pinned to the bottom of the scroll region

# Kept to 79 chars so it survives _fit() on an 80-column terminal — a clipped
# legend defeats the whole point of pinning it.
LEGEND = ("[w/s] speed  [a/d] yaw  [space] stop  "
          "[x] straight  [h] help  [q] quit")


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
    ("a / LEFT", "angular.z  +%.2f rad/s  (ROS: + = left)" % YAW_STEP),
    ("d / RIGHT", "angular.z  -%.2f rad/s  (ROS: - = right)" % YAW_STEP),
    (None, None),
    ("space", "full stop (linear + angular -> 0)"),
    ("x", "straighten (angular.z -> 0)"),
    (None, None),
    ("h", "show these keys again"),
    ("q  /  ^C", "quit"),
]


def print_help():
    """Draw the key map as an aligned, boxed block — readable at a glance."""
    width = 60
    say("+" + "-" * width + "+")
    say("|" + " NANO-SIM  ROS 2  KEYBOARD TELEOP ".center(width) + "|")
    say("|" + ("/cmd_vel @ %.0f Hz" % (1.0 / PERIOD)).center(width) + "|")
    say("+" + "-" * width + "+")
    for key, desc in HELP:
        if key is None:
            say("|" + " " * width + "|")
        else:
            say("|  %-12s  %-*s|" % (key, width - 16, desc))
    say("+" + "-" * width + "+")


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


class NanoSimTeleop(Node):
    def __init__(self, keys):
        super().__init__("nanosim_teleop")
        self.keys = keys
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(PERIOD, self.tick)
        self.speed = 0.0
        self.yaw = 0.0
        self.done = False

    def tick(self):
        # Drain everything buffered since the last publish.
        while True:
            key = self.keys.get(0.0)
            if key is None:
                break
            if key in ("q", "\x03"):
                self.done = True
                return
            elif key in ("w", "UP"):
                self.speed = clamp(self.speed + SPEED_STEP, MAX_SPEED)
            elif key in ("s", "DOWN"):
                self.speed = clamp(self.speed - SPEED_STEP, MAX_SPEED)
            elif key in ("a", "LEFT"):
                self.yaw = clamp(self.yaw + YAW_STEP, MAX_YAW)   # ROS: + = left
            elif key in ("d", "RIGHT"):
                self.yaw = clamp(self.yaw - YAW_STEP, MAX_YAW)
            elif key == " ":
                self.speed = self.yaw = 0.0
            elif key == "x":
                self.yaw = 0.0
            elif key == "h":
                print_help()

        cmd = Twist()
        cmd.linear.x = self.speed
        cmd.angular.z = self.yaw
        self.pub.publish(cmd)

        draw_status([
            LEGEND,
            f"SPD {bar(self.speed, MAX_SPEED)} {self.speed:+.2f} m/s  "
            f"YAW {bar(self.yaw, MAX_YAW)} {self.yaw:+.2f} rad/s",
        ])

    def stop(self):
        """Leave the car stopped rather than waiting out the watchdog."""
        self.pub.publish(Twist())


def main():
    if not sys.stdin.isatty():
        sys.exit("keyboard teleop needs a real terminal (stdin is not a TTY)")

    rclpy.init()
    with KeyReader() as keys:
        node = NanoSimTeleop(keys)
        print_help()
        try:
            while rclpy.ok() and not node.done:
                rclpy.spin_once(node, timeout_sec=0.1)
        except KeyboardInterrupt:
            pass
        finally:
            node.stop()
            node.destroy_node()
            rclpy.shutdown()
    sys.stdout.write("\n" * STATUS_LINES)   # step past the sticky block
    print("stopped.")


if __name__ == "__main__":
    main()
