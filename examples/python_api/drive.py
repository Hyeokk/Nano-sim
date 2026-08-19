#!/usr/bin/env python3
"""Nano-sim Python API — vehicle-control example (SimServer, TCP 7720).

Wire format, both directions:  [4-byte little-endian length][UTF-8 JSON]
  Request   {"id": N, "method": "...", "params_json": "<JSON string>"}
  Response  {"id": N, "error": null|str, "result": "<JSON string>"}

Drives one closed-loop episode in sync mode: set_control -> tick.
No ROS, no Docker — just Unity in Play mode with NetworkManager active.
"""
import json
import math
import socket
import struct


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


def main():
    c = NanoSimClient("127.0.0.1", 7720)
    try:
        print("connected:", c.call("connect"))

        c.call("set_sync_mode", {"enabled": True})   # deterministic stepping
        c.call("reset")

        for step in range(300):
            # Toy open-loop controller: constant throttle, sinusoidal steering.
            steering = 0.4 * math.sin(step * 0.03)    # -1..1  (+ = right)
            throttle = 0.5                            # -1..1  (negative = reverse)
            c.call("set_control", {"steering": steering, "throttle": throttle})

            res = c.call("tick")                      # steps physics, returns obs
            if step % 30 == 0:
                ego = res["observation"]["ego_state"]
                route = res["observation"].get("route") or {}
                print(f"step {step:3d}  "
                      f"speed={ego['speed']:.2f} m/s  "
                      f"cte={route.get('cross_track_error', 0.0):+.3f} m")

            if res.get("terminated") or res.get("truncated"):
                print("episode ended -> reset")
                c.call("reset")
    finally:
        c.call("set_sync_mode", {"enabled": False})
        c.call("disconnect")
        c.close()


if __name__ == "__main__":
    main()
