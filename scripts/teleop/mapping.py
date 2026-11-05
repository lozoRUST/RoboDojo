"""Pinned LeRobot old calibration -> SO101 new-calibration URDF joint angles.

Offsets derived by matching common STL visual transforms in official old/new
URDFs after aligning the base mesh. These are model conventions, not measured
corrections for a particular physical arm. See README for validation evidence.
"""
import math

ARM_OFFSETS_RAD = (0.0, math.pi / 2, -math.pi / 2, 0.0, 0.0486795)


def to_sim_radians(values, sim_signs=(1, 1, 1, 1, 1, 1), *, calibrated_signs=(1, 1, 1, 1, 1, 1)):
    # Optional hardware-direction trial, applied AFTER model-coordinate
    # conversion. Reflect around simulation q=0; do not alter calibration or
    # the official old/new URDF offsets. Default retains the geometric mapping.
    if len(sim_signs) != 6 or any(s not in (-1, 1) for s in sim_signs) or sim_signs[5] != 1:
        raise ValueError('Use six +/-1 simulation signs; gripper sign must remain +1.')
    if len(calibrated_signs) != 6 or any(s not in (-1, 1) for s in calibrated_signs) or calibrated_signs[5] != 1:
        raise ValueError('Use six +/-1 calibrated signs; gripper sign must remain +1.')
    if len(values) != 6 or not all(math.isfinite(float(v)) for v in values):
        raise ValueError('Expected five finite joint angles in degrees and gripper percent.')
    if not -10 <= values[5] <= 110:
        raise ValueError('Calibrated gripper percentage outside LeRobot bounds.')
    # Reflect calibrated angles about legacy Zero BEFORE adding model offsets.
    # This preserves the nominal Zero pose when correcting a measured direction.
    arm = [s * math.radians(float(v)) + offset
           for v, s, offset in zip(values[:5], calibrated_signs[:5], ARM_OFFSETS_RAD)]
    # Legacy calibration can return e.g. -186 degrees for an elbow whose
    # physical old-model angle is +174 degrees. Select the equivalent new
    # model angle in [-pi, pi), before enforcing simulation joint limits.
    arm = [(q + math.pi) % (2 * math.pi) - math.pi for q in arm]
    # The follower URDF uses an angular jaw; nominal closed=0, open=100 degrees.
    target = arm + [math.radians(max(0, min(100, float(values[5]))))]
    return [q * s for q, s in zip(target, sim_signs)]


def validate_calibration(calibration, names):
    if calibration.get('motor_names') != list(names):
        raise ValueError('Calibration motor order does not match SO101.')
    expected_modes = ['DEGREE'] * 5 + ['LINEAR']
    if calibration.get('calib_mode') != expected_modes:
        raise ValueError('Expected the pinned LeRobot legacy calibration format.')
    for i, name in enumerate(names[:5]):
        zero = ((1 - 2 * calibration['drive_mode'][i]) * calibration['start_pos'][i]
                + calibration['homing_offset'][i]) * 360 / 4096
        zero = (zero + 180) % 360 - 180
        if abs(zero) > 15:
            raise ValueError(f'{name}: Zero reference error {zero:.1f} degrees; check calibration.')
    if abs(calibration['end_pos'][5] - calibration['start_pos'][5]) < 100:
        raise ValueError('Gripper calibration span is too small.')
