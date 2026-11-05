"""Local leader packet protocol; independent of Isaac Sim and LeRobot."""
import json
import math
import socket
import time

JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')

class Receiver:
    def __init__(self, port, timeout=0.5):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('127.0.0.1', port))
        self.socket.setblocking(False)
        self.timeout = timeout
        self.latest = None
        self.sequence = -1
        self.session = None
        self.received = 0

    def poll(self):
        for _ in range(256):
            try:
                data, _ = self.socket.recvfrom(8192)
            except BlockingIOError:
                break
            try:
                p = json.loads(data)
                age = time.monotonic() - p['time']
                absolute = p['version'] == 2 and p.get('mode') == 'absolute'
                q = p['q_rad'] if absolute else p['delta_rad']
                if (p['version'] != 1 and not absolute) or p['joints'] != list(JOINTS):
                    continue
                if not 0 <= age < self.timeout or len(q) != 6:
                    continue
                if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in q):
                    continue
                if not isinstance(p['session'], str) or not isinstance(p['seq'], int):
                    continue
                if p['session'] == self.session and p['seq'] <= self.sequence:
                    continue
                self.session, self.sequence = p['session'], p['seq']
                self.latest = p
                self.received += 1
            except (KeyError, TypeError, ValueError):
                continue
        if self.latest and time.monotonic() - self.latest['time'] < self.timeout:
            return self.latest
        return None

    def close(self):
        self.socket.close()
