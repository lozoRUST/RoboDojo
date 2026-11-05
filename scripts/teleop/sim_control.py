"""Apply leader relative joint targets to the registered SO-ARM101 articulation."""
import time
import math
import torch
from teleop.bridge import JOINTS, Receiver

class LeaderControl:
    def __init__(self, env, port, max_speed=5.0):
        if not math.isfinite(max_speed) or not 0 < max_speed <= 10:
            raise ValueError('max_speed must be >0 and <=10 rad/s.')
        if len(env.robot_manager.robot_list) != 1 or env.robot_manager.robot_list[0].robot_name != 'soarm101':
            raise ValueError('Leader teleoperation requires a single SO-ARM101 robot.')
        self.receiver = Receiver(port)
        self.robot = env.robot_manager.robot_key[0]
        self.ids = [self.robot.joint_names.index(name) for name in JOINTS]
        self.target = self.robot.data.joint_pos[:, self.ids].clone()
        self.limits = self.robot.data.soft_joint_pos_limits[:, self.ids].clone()
        self.session = None
        self.active = False
        self.max_speed = max_speed
        self.previous_time = time.monotonic()
        self.last_diagnostic = 0.0
        self.metric_started = self.previous_time
        self.metric_steps = 0
        self.sim_step_dt = float(env.sim.step_dt)
        self.loop_hz = 0.0
        print(f'[teleop] Target slew limit: {max_speed} rad/s', flush=True)
        print('[teleop] Effective limits (deg):', dict(zip(JOINTS,
              torch.rad2deg(self.limits[0]).cpu().tolist())), flush=True)
        print(f'[teleop] Listening on 127.0.0.1:{port}; waiting for leader.', flush=True)

    def step(self):
        now = time.monotonic()
        dt = min(now - self.previous_time, 0.05)
        self.previous_time = now
        self.metric_steps += 1
        elapsed = now - self.metric_started
        if elapsed >= 1.0:
            self.loop_hz = self.metric_steps / elapsed
            self.metric_started = now
            self.metric_steps = 0
        p = self.receiver.poll()
        if p is None:
            if self.active:
                self.target = self.robot.data.joint_pos[:, self.ids].clone()
                print('[teleop] Leader stale/disconnected: holding current pose.', flush=True)
            self.active = False
        else:
            absolute = p['version'] == 2
            delta = torch.tensor([p['q_rad'] if absolute else p['delta_rad']], device=self.target.device, dtype=self.target.dtype)
            # These are bounded revolute joints, not continuous axes. A badly
            # mapped angle near +/-pi must not clamp to opposite end stops.
            # Allow small calibration residuals; reject grossly invalid poses.
            margin = torch.deg2rad(torch.tensor(5.0, device=self.target.device))
            invalid = (delta < self.limits[..., 0] - margin) | (delta > self.limits[..., 1] + margin)
            if absolute and bool(invalid.any()):
                self.target = self.robot.data.joint_pos[:, self.ids].clone()
                self.active = False
                if now - self.last_diagnostic >= 1.0:
                    self.last_diagnostic = now
                    bad = [JOINTS[i] for i in range(len(JOINTS)) if bool(invalid[0, i])]
                    angles = torch.rad2deg(delta[0]).cpu().tolist()
                    print(f'[teleop] INVALID_ABSOLUTE_TARGET {bad}: '
                          f'{[round(v, 2) for v in angles]} deg; holding measured pose. '
                          'Check calibration direction/zero; target exceeds limits by >5 deg.', flush=True)
                self.robot.set_joint_position_target(self.target, joint_ids=self.ids)
                return
            if not self.active or p['session'] != self.session:
                self.origin = self.robot.data.joint_pos[:, self.ids].clone()
                self.target = self.origin.clone()
                self.reference = delta.clone()
                self.session = p['session']
                print('[teleop] Leader connected; ' + (
                    'absolute calibrated targets, approaching at limited speed.' if absolute
                    else 'current poses anchored.'), flush=True)
            self.active = True
            desired = delta if absolute else self.origin + delta - self.reference
            requested = desired
            desired = torch.maximum(self.limits[..., 0], torch.minimum(self.limits[..., 1], desired))
            self.target += torch.clamp(desired - self.target, -self.max_speed * dt, self.max_speed * dt)
            if now - self.last_diagnostic >= 1.0:
                self.last_diagnostic = now
                actual = self.robot.data.joint_pos[:, self.ids]
                errors = torch.rad2deg(torch.stack(((desired - self.target).abs().max(),
                                                   (self.target - actual).abs().max()))).cpu().tolist()
                print(f'[teleop] latency: packet_age_ms={(now - p["time"]) * 1000:.1f} '
                      f'loop_hz={self.loop_hz:.1f} sim/wall={self.loop_hz * self.sim_step_dt:.2f} '
                      f'slew_gap_deg={errors[0]:.1f} tracking_gap_deg={errors[1]:.1f}', flush=True)
                for i in (1, 4):
                    numbers = torch.rad2deg(torch.stack((requested[0, i], desired[0, i],
                                                        self.target[0, i], actual[0, i]))).cpu().tolist()
                    clipped = abs(numbers[0] - numbers[1]) > 0.01
                    print(f'[teleop] {JOINTS[i]} deg requested/limited/command/actual: '
                          f'{[round(v, 2) for v in numbers]}' + (' LIMIT_CLIPPED' if clipped else ''), flush=True)
        self.robot.set_joint_position_target(self.target, joint_ids=self.ids)

    def close(self):
        print(f'[teleop] Valid packets received: {self.receiver.received}', flush=True)
        print(f'[teleop] Joint positions (rad): {self.robot.data.joint_pos[:, self.ids].tolist()}', flush=True)
        self.receiver.close()
