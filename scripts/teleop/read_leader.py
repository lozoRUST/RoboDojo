#!/usr/bin/env python3
"""Read a Feetech SO leader in the pinned LeRobot environment.

Relative mode: the pose at startup maps to the simulation's current pose.
No Goal_Position or torque-enable commands are sent to the physical arm.
"""
import argparse
import json
import math
from pathlib import Path
import socket
import time
import uuid

from bridge import JOINTS
from mapping import to_sim_radians, validate_calibration


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', required=True)
    p.add_argument('--udp-port', type=int, default=15051)
    p.add_argument('--hz', type=float, default=100, help='Requested read/send frequency (default 100 Hz).')
    p.add_argument('--calibration', type=Path)
    p.add_argument('--mode', choices=['relative', 'absolute'], default='relative')
    p.add_argument('--signs', default='1,1,1,1,1,1', help='Six direction signs in joint order.')
    p.add_argument('--sim-signs', default='1,1,1,1,1,1',
                   help='Absolute-mode direction correction about simulation zero, AFTER model conversion; gripper must be +1.')
    p.add_argument('--calibrated-signs', default=None,
                   help='Signs BEFORE model offsets; absolute default 1,-1,1,1,-1,1; relative default all +1; gripper must be +1.')
    p.add_argument('--free-leader', action='store_true', help='Disable motor torque to allow manual movement.')
    p.add_argument('--probe', action='store_true', help='Read raw positions once and exit.')
    args = p.parse_args()
    if args.mode == 'absolute' and args.calibration is None:
        p.error('--mode absolute requires --calibration from the pinned LeRobot version.')
    calibration = None
    if args.calibration:
        calibration = json.loads(args.calibration.read_text())
        validate_calibration(calibration, JOINTS)
    signs = [float(v) for v in args.signs.split(',')]
    if len(signs) != 6 or any(v not in (-1, 1) for v in signs) or not 0 < args.hz <= 100:
        p.error('Use six +1/-1 signs and 0 < hz <= 100.')
    if args.mode == 'absolute' and signs != [1] * 6:
        p.error('Absolute mode uses calibration drive_mode; do not override --signs.')
    try:
        sim_signs = [int(v) for v in args.sim_signs.split(',')]
        calibrated_signs_text = args.calibrated_signs
        if calibrated_signs_text is None:
            calibrated_signs_text = '1,-1,1,1,-1,1' if args.mode == 'absolute' else '1,1,1,1,1,1'
        calibrated_signs = [int(v) for v in calibrated_signs_text.split(',')]
        to_sim_radians([0] * 6, sim_signs, calibrated_signs=calibrated_signs)
    except ValueError as exc:
        p.error(str(exc))
    if args.mode != 'absolute' and sim_signs != [1] * 6:
        p.error('--sim-signs requires --mode absolute.')
    if args.mode != 'absolute' and calibrated_signs != [1] * 6:
        p.error('--calibrated-signs requires --mode absolute.')
    if sim_signs != [1] * 6 and calibrated_signs != [1] * 6:
        p.error('Do not combine --sim-signs with --calibrated-signs.')
    from lerobot.common.robot_devices.robots.configs import So101RobotConfig
    from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus
    bus_config = So101RobotConfig().leader_arms['main']
    bus_config.port = args.port
    if tuple(bus_config.motors) != JOINTS:
        raise RuntimeError('Installed LeRobot joint order differs from the bridge protocol.')
    bus = FeetechMotorsBus(bus_config)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        bus.connect()
        bus.set_bus_baudrate(1_000_000)
        raw = bus.read('Present_Position').tolist()
        print('Leader motor IDs 1..6:', dict(zip(JOINTS, raw)), flush=True)
        if args.probe and calibration is None:
            return
        if calibration is not None:
            bus.set_calibration(calibration)
        if args.probe:
            values = bus.read('Present_Position').tolist()
            print('Calibrated degrees / gripper percent:', values, flush=True)
            print('New-calibration simulation radians:', to_sim_radians(values, sim_signs, calibrated_signs=calibrated_signs), flush=True)
            return
        if args.free_leader:
            bus.write('Torque_Enable', 0)
        elif any(bus.read('Torque_Enable')):
            raise RuntimeError('Leader torque enabled. Support the arm and restart with --free-leader.')
        print(f'{args.mode} control: ' + (
            'using calibrated joint angles and old-to-new URDF offsets.' if args.mode == 'absolute'
            else 'startup leader pose is the reference; verify each joint direction.'), flush=True)
        if args.mode == 'absolute':
            print('Simulation direction signs:', dict(zip(JOINTS, sim_signs)), flush=True)
            print('Calibrated direction signs (before offsets):', dict(zip(JOINTS, calibrated_signs)), flush=True)
        session = str(uuid.uuid4())
        seq = 0
        previous = None
        delta = [0.0] * 6
        metric_started = time.monotonic()
        metric_samples = 0
        metric_read_seconds = 0.0
        while True:
            start = time.monotonic()
            values = bus.read('Present_Position').tolist()
            metric_read_seconds += time.monotonic() - start
            metric_samples += 1
            if args.mode == 'relative' and previous is not None:
                for i in range(6):
                    diff = values[i] - previous[i]
                    if args.calibration:
                        # Pinned LeRobot uses degrees for arm joints, percent for gripper.
                        step = math.radians(diff) if i < 5 else diff / 100 * math.radians(100)
                    else:
                        # STS3215 encoders: 4096 ticks/revolution; unwrap across zero.
                        step = ((diff + 2048) % 4096 - 2048) * 2 * math.pi / 4096
                    if abs(step) > 0.5:
                        raise RuntimeError(f'Unexpected encoder jump on {JOINTS[i]}; restart to re-anchor.')
                    delta[i] += signs[i] * step
            previous = values
            packet = dict(version=1, session=session, seq=seq, time=time.monotonic(),
                          joints=list(JOINTS), delta_rad=delta)
            if args.mode == 'absolute':
                packet = dict(version=2, mode='absolute', session=session, seq=seq,
                              time=time.monotonic(), joints=list(JOINTS), q_rad=to_sim_radians(values, sim_signs, calibrated_signs=calibrated_signs))
            sock.sendto(json.dumps(packet).encode(), ('127.0.0.1', args.udp_port))
            seq += 1
            if seq % int(max(1, args.hz)) == 0:
                elapsed = time.monotonic() - metric_started
                print(f'Reader: effective_hz={metric_samples / max(elapsed, 1e-6):.1f} '
                      f'mean_read_ms={1000 * metric_read_seconds / metric_samples:.1f}', flush=True)
                metric_started = time.monotonic()
                metric_samples = 0
                metric_read_seconds = 0.0
                if args.calibration:
                    print('Calibrated degrees / gripper percent:', [round(v, 2) for v in values], flush=True)
                print(f'{args.mode} radians:', [round(v, 3) for v in packet.get('q_rad', delta)], flush=True)
            time.sleep(max(0, 1 / args.hz - (time.monotonic() - start)))
    finally:
        sock.close()
        if bus.is_connected:
            bus.disconnect()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
