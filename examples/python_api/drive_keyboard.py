#!/usr/bin/env python3
"""Nano-sim Python API — keyboard teleop (SimServer, TCP 7720).

Wire format, both directions:  [4-byte little-endian length][UTF-8 JSON]
  Request   {"id": N, "method": "...", "params_json": "<JSON string>"}
  Response  {"id": N, "error": null|str, "result": "<JSON string>"}

Drives the car by hand in sync mode: read keys -> set_control -> tick.
Use it to confirm the link is alive and that controls reach the vehicle.
No ROS, no Docker — just Unity in Play mode with NetworkManager active.

Run from a real terminal (needs a TTY; an IDE's redirected console will not work):
    python3 drive_keyboard.py

Commands SPEED in m/s, not raw throttle. set_control only accepts a normalised
throttle, so the target speed is divided by the vehicle's speed scale (Unity:
Vehicle Setting > Vehicle Speed). Keep --vehicle-speed equal to that slider.

Controls
    w / UP      speed +             s / DOWN    speed -
    a / LEFT    steer left          d / RIGHT   steer right
    space       full stop           x           centre steering
    r           reset episode       q / Ctrl-C  quit

POSIX only (macOS / Linux), matching the shipped Nano-sim binaries.
"""
import argparse
import json
import select
import shutil
import socket
import struct
import sys
import termios
import time
import tty

TICK_HZ = 50.0          # wall-clock pacing of the sync-mode tick loop
STEER_STEP = 0.10       # per key press, normalised -1..1
SPEED_STEP = 0.5        # m/s per key press

# Unity's Vehicle Setting > Vehicle Speed, i.e. the speed at throttle = 1.0.
# set_control carries a NORMALISED throttle, so a target speed goes out as
# throttle = speed / VEHICLE_SPEED. If this does not match the slider the
# commanded m/s is silently scaled wrong; override with --vehicle-speed.
VEHICLE_SPEED = 7.0     # m/s

ARROWS = {"[A": "UP", "[B": "DOWN", "[C": "RIGHT", "[D": "LEFT"}


def clamp(v, lo=-1.0, hi=1.0):
    return max(lo, min(hi, v))


STATUS_LINES = 2        # sticky block pinned to the bottom of the scroll region

# Kept to 79 chars so it survives _fit() on an 80-column terminal — a clipped
# legend defeats the whole point of pinning it.
LEGEND = ("[w/s] speed  [a/d] str  [space] stop  [x] centre  "
          "[r] reset  [h] help  [q] quit")


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


def print_help(vmax):
    """Draw the key map as an aligned, boxed block — readable at a glance."""
    rows = [
        ("w / UP", "speed  +%.2f m/s   (max %.2f)" % (SPEED_STEP, vmax)),
        ("s / DOWN", "speed  -%.2f m/s   (negative = reverse)" % SPEED_STEP),
        ("a / LEFT", "steer left   %.2f   (API: - = left)" % STEER_STEP),
        ("d / RIGHT", "steer right  %.2f   (API: + = right)" % STEER_STEP),
        (None, None),
        ("space", "full stop (speed + steering -> 0)"),
        ("x", "centre steering only"),
        ("r", "reset episode"),
        (None, None),
        ("h", "show these keys again"),
        ("q  /  ^C", "quit"),
    ]
    width = 58
    say("+" + "-" * width + "+")
    say("|" + " NANO-SIM  KEYBOARD TELEOP ".center(width) + "|")
    say("|" + ("throttle = speed / %.2f m/s" % vmax).center(width) + "|")
    say("+" + "-" * width + "+")
    for key, desc in rows:
        if key is None:
            say("|" + " " * width + "|")
        else:
            say("|  %-12s  %-*s|" % (key, width - 16, desc))
    say("+" + "-" * width + "+")


def bar(value, limit=1.0, half=5):
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


class NanoSimClient:
    def __init__(self, host="127.0.0.1", port=7720, timeout=60.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self._id = 0

    def _recv_exact(self, n):
        buf = bytearray()
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("SimServer closed the connection")
            buf.extend(chunk)
        return bytes(buf)

    def call(self, method, params=None):
        """Send one request, return the parsed result (raises on server error)."""
        self._id += 1
        req = {"id": self._id, "method": method,
               "params_json": json.dumps(params or {})}
        payload = json.dumps(req).encode("utf-8")
        self.sock.sendall(struct.pack("<I", len(payload)) + payload)

        length = struct.unpack("<I", self._recv_exact(4))[0]
        resp = json.loads(self._recv_exact(length).decode("utf-8"))
        if resp.get("error"):
            raise RuntimeError(f"{method} failed: {resp['error']}")
        result = resp.get("result")
        if isinstance(result, str) and result:      # result is double-encoded JSON
            try:
                return json.loads(result)
            except json.JSONDecodeError:
                return result
        return result

    def close(self):
        self.sock.close()


def teleop(c, kb, vmax):
    """Closed-loop teleop until the user quits. Returns on 'q'."""
    steering = 0.0
    speed = 0.0             # TARGET speed, m/s
    step = 0
    period = 1.0 / TICK_HZ
    next_tick = time.perf_counter()

    while True:
        # Drain every key buffered since the last tick; the latest press wins
        # for one-shot keys, and repeats accumulate on the axes.
        reset_requested = False
        while True:
            key = kb.get(0.0)
            if key is None:
                break
            if key in ("q", "\x03"):
                return
            elif key in ("w", "UP"):
                speed = clamp(speed + SPEED_STEP, -vmax, vmax)
            elif key in ("s", "DOWN"):
                speed = clamp(speed - SPEED_STEP, -vmax, vmax)
            elif key in ("a", "LEFT"):
                steering = clamp(steering - STEER_STEP)   # API: + = right
            elif key in ("d", "RIGHT"):
                steering = clamp(steering + STEER_STEP)
            elif key == " ":
                steering = speed = 0.0
            elif key == "x":
                steering = 0.0
            elif key == "r":
                reset_requested = True
            elif key == "h":
                print_help(vmax)

        if reset_requested:
            steering = speed = 0.0
            c.call("set_control", {"steering": 0.0, "throttle": 0.0})
            c.call("reset")
            step = 0
            say(">> reset (manual)")

        # The wire protocol carries a normalised throttle; convert here so the
        # entire UI above can stay in m/s.
        c.call("set_control", {"steering": steering,
                               "throttle": clamp(speed / vmax)})
        res = c.call("tick")
        step += 1

        ego = res["observation"]["ego_state"]
        route = res["observation"].get("route") or {}
        draw_status([
            LEGEND,
            f"SPD {bar(speed, vmax)} {speed:+.2f} -> {ego['speed']:5.2f} m/s  "
            f"STR {bar(steering)} {steering:+.2f}  "
            f"cte {route.get('cross_track_error', 0.0):+.3f} "
            f"step {step:4d}",
        ])

        if res.get("terminated") or res.get("truncated"):
            steering = speed = 0.0
            c.call("set_control", {"steering": 0.0, "throttle": 0.0})
            c.call("reset")
            step = 0
            say(">> episode ended -> reset")

        next_tick += period
        sleep = next_tick - time.perf_counter()
        if sleep > 0:
            time.sleep(sleep)
        else:
            next_tick = time.perf_counter()   # fell behind; do not spiral


def main():
    ap = argparse.ArgumentParser(description="Nano-sim keyboard teleop (speed control)")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=7720)
    ap.add_argument("--vehicle-speed", type=float, default=VEHICLE_SPEED,
                    help="Unity's Vehicle Setting > Vehicle Speed in m/s "
                         "(speed at throttle = 1.0; default %(default)s)")
    args = ap.parse_args()
    if args.vehicle_speed <= 0:
        sys.exit("--vehicle-speed must be > 0 (it is the divisor for throttle)")
    if not sys.stdin.isatty():
        sys.exit("keyboard teleop needs a real terminal (stdin is not a TTY)")

    c = NanoSimClient(args.host, args.port)
    try:
        say(f"connected: {c.call('connect')}")
        c.call("set_sync_mode", {"enabled": True})   # deterministic stepping
        c.call("reset")

        print_help(args.vehicle_speed)
        with KeyReader() as kb:
            teleop(c, kb, args.vehicle_speed)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            c.call("set_control", {"steering": 0.0, "throttle": 0.0})
            c.call("set_sync_mode", {"enabled": False})
            c.call("disconnect")
        finally:
            c.close()
        sys.stdout.write("\n" * STATUS_LINES)   # step past the sticky block
        print("stopped.")


if __name__ == "__main__":
    main()
