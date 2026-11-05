#!/usr/bin/env python3
"""Run the installed LeRobot SO101 manual leader calibration interactively."""
import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parents[2]
                        / '.cache/calibration/so101/main_leader.json')
    args = parser.parse_args()
    if args.output.exists():
        parser.error(f'Calibration already exists: {args.output}. Use --output with a new filename.')
    from lerobot.common.robot_devices.robots.configs import So101RobotConfig
    from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus
    from lerobot.common.robot_devices.robots.feetech_calibration import run_arm_manual_calibration

    config = So101RobotConfig().leader_arms['main']
    config.port = args.port
    bus = FeetechMotorsBus(config)
    try:
        bus.connect()
        bus.set_bus_baudrate(1_000_000)
        print('Motor positions:', bus.read('Present_Position').tolist(), flush=True)
        input('Support the leader arm. Press Enter to disable torque and begin official calibration: ')
        bus.write('Torque_Enable', 0)
        calibration = run_arm_manual_calibration(bus, 'so101', 'main', 'leader')
        # In this pinned LeRobot version, start_pos is the raw Zero reading.
        # A correct zero reference must map close to zero after applying the
        # drive sign and homing offset inferred from the Rotated reading.
        invalid = []
        for i, name in enumerate(calibration['motor_names']):
            if calibration['calib_mode'][i] != 'DEGREE':
                continue
            sign = 1 - 2 * calibration['drive_mode'][i]
            zero = (sign * calibration['start_pos'][i] + calibration['homing_offset'][i]) * 360 / 4096
            zero = (zero + 180) % 360 - 180
            print(f'Zero reference check: {name} = {zero:.2f} degrees')
            if abs(zero) > 15:
                invalid.append(name)
        if invalid:
            raise RuntimeError(
                'Zero/Rotated reference mismatch on ' + ', '.join(invalid)
                + '. No file saved. Recheck both reference poses and repeat calibration.'
            )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open('x') as stream:
            json.dump(calibration, stream, indent=2)
        print(f'Calibration saved: {args.output}', flush=True)
        print('Convention: old LeRobot calibration (horizontal zero). Simulation mapping still needs alignment.')
    finally:
        if bus.is_connected:
            bus.disconnect()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nCalibration interrupted; no completed calibration was saved.')
