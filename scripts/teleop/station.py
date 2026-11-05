"""RoboDojo adapter for the local collection station (versioned file IPC).

Physics owns robot state; a bounded IO queue owns frame spooling and JPEG preview.
Only explicit operations are accepted. No browser-supplied code is executed.
"""
import json
import os
from pathlib import Path
import queue
import shutil
import threading
import time

import numpy as np
import cv2


def atomic(path, data):
    path = Path(path)
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(data, ensure_ascii=False))
    os.replace(tmp, path)


class RoboDojoBackend:
    # SO101.urdf: gripper_frame_joint is fixed to gripper_link at this offset.
    # The frame origin is a stable grasp reference; moving_jaw is not.
    GRIPPER_FRAME_OFFSET = np.array([-0.0079, -0.000218121, -0.0981274], dtype=np.float64)

    def __init__(self, env, controller):
        self.env, self.controller = env, controller

    def read_state(self):
        c = self.controller
        return {'observation.state': c.robot.data.joint_pos[0, c.ids].detach().cpu().numpy().copy(),
                'action': c.target[0].detach().cpu().numpy().copy()}

    def read_gripper_frame_position(self):
        """Measured fixed gripper-frame origin, expressed in env-local meters."""
        robot_cfg = self.env.robot_manager.robot_list[0]
        pose = self.env.robot_manager.get_link_pose(
            robot_cfg, 'gripper_link', env_idx_list=[0], is_relative=True,
        )[0]
        if pose is None or len(pose) != 7 or not np.isfinite(pose).all():
            raise RuntimeError('Measured gripper_link pose is unavailable')
        # Isaac pose quaternion is [w, x, y, z]. Rotate the fixed URDF offset by
        # the measured parent orientation; this is a rigid-frame transform, not
        # joint-command forward kinematics.
        q = pose[3:]
        qv = q[1:]
        offset = self.GRIPPER_FRAME_OFFSET
        rotated = offset + 2.0 * np.cross(qv, np.cross(qv, offset) + q[0] * offset)
        return (pose[:3] + rotated).astype(np.float64)

    def stop(self):
        c = self.controller
        c.target = c.robot.data.joint_pos[:, c.ids].clone()
        c.robot.set_joint_position_target(c.target, joint_ids=c.ids)
        c.active = False


class RoboDojoCameras:
    def __init__(self, env, views):
        self.env, self.views = env, views
        if env.camera_manager.camera_names[0] != ['cam_head', 'cam_side', 'cam_wrist']:
            raise ValueError('Station profile requires the SO101 three-camera configuration')

    def read_frames(self):
        data = self.env.capture_manager.step(env_ids=[0])
        images = {}
        for name, camera in zip(self.views, data):
            rgb = np.asarray(camera['rgb'][0]['data'])[:, :, :3]
            if rgb.shape != (480, 640, 3) or rgb.dtype != np.uint8:
                raise ValueError(f'Invalid camera output: {name}, {rgb.shape}, {rgb.dtype}')
            images[name] = rgb.copy()
        return images


class StationBridge:
    def __init__(self, directory, env, controller):
        self.dir = Path(directory)
        self.config = json.loads((self.dir / 'config.json').read_text())
        self.env, self.controller = env, controller
        self.robot = RoboDojoBackend(env, controller)
        self.cameras = RoboDojoCameras(env, self.config['views'])
        self.phase = 'CONNECTING'
        self.enabled = False
        self.closed = False
        self.error = None
        self.episode = None
        self.frames = 0
        self.trajectory_points = []
        self.trajectory_sample_count = 0
        self.trajectory_current = None
        self.trajectory_error = None
        self.trajectory_frame_interval = 5
        self.trajectory_limit = 300
        self.joint_history = []
        self.joint_history_limit = 300
        self.tick = 0
        self.sim_time = 0.0
        self.started = time.monotonic()
        self.last_poll = self.last_status = self.last_preview = 0.0
        self.io_error = None
        self.jobs = queue.Queue(maxsize=8)
        self.io = threading.Thread(target=self._io_loop, daemon=True)
        self.io.start()
        self._status()

    def _io_loop(self):
        while True:
            job = self.jobs.get()
            try:
                if job is None:
                    return
                images, sample, path, preview = job
                if path is not None:
                    with path.with_suffix('.tmp').open('wb') as f:
                        np.savez(f, **sample, **images)
                    os.replace(path.with_suffix('.tmp'), path)
                if preview:
                    for name, rgb in images.items():
                        ok, jpeg = cv2.imencode('.jpg', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 75])
                        if not ok:
                            raise RuntimeError('JPEG encoding failed')
                        dest = self.dir / f'{name}.jpg'
                        dest.with_suffix('.tmp').write_bytes(jpeg.tobytes())
                        os.replace(dest.with_suffix('.tmp'), dest)
            except Exception as exc:
                self.io_error = str(exc)
            finally:
                self.jobs.task_done()

    def _status(self):
        sample = self.robot.read_state()
        try:
            self.trajectory_current = self.robot.read_gripper_frame_position().tolist()
            self.trajectory_error = None
        except Exception as exc:
            self.trajectory_current = None
            self.trajectory_error = str(exc)
        p = self.controller.receiver.latest
        atomic(self.dir / 'status.json', {
            'protocol': 1, 'phase': self.phase, 'enabled': self.enabled,
            'heartbeat': time.monotonic(), 'frames': self.frames,
            'inputValid': self.controller.active,
            'episode': self.episode.name if self.episode else None,
            'simulationTime': self.sim_time, 'error': self.error,
            'state': sample['observation.state'].tolist(), 'action': sample['action'].tolist(),
            'inputAgeMs': (time.monotonic() - p['time']) * 1000 if p else None,
            'simWallRatio': self.controller.loop_hz * self.controller.sim_step_dt,
            'previewTime': self.last_preview,
            'trajectory': {
                'episode': self.episode.name if self.episode else None,
                'frame': self.frames,
                'frameInterval': self.trajectory_frame_interval,
                'sampleCount': self.trajectory_sample_count,
                'truncated': self.trajectory_sample_count > len(self.trajectory_points),
                'link': 'gripper_frame_link (gripper_link + SO101.urdf 固定偏移)',
                'coordinateFrame': 'environment_local',
                'current': self.trajectory_current,
                'error': self.trajectory_error,
                'points': self.trajectory_points,
            },
            'jointHistory': self.joint_history,
        })

    def _sample_joint_history(self, frame, sim_time, sample):
        if frame != 1 and frame % self.trajectory_frame_interval != 0:
            return
        state = np.asarray(sample['observation.state'], dtype=np.float64).tolist()
        action = np.asarray(sample['action'], dtype=np.float64).tolist()
        self.joint_history.append({'frame': frame, 'time': float(sim_time), 'state': state, 'action': action})
        if len(self.joint_history) > self.joint_history_limit:
            del self.joint_history[:len(self.joint_history) - self.joint_history_limit]

    def _sample_trajectory(self, frame, sim_time):
        """Append a measured point at the declared cadence; telemetry cannot fail capture."""
        if frame != 1 and frame % self.trajectory_frame_interval != 0:
            return
        try:
            self.trajectory_current = self.robot.read_gripper_frame_position().tolist()
            self.trajectory_error = None
            self.trajectory_points.append({
                'frame': frame,
                'time': sim_time,
                'position': self.trajectory_current,
            })
            self.trajectory_sample_count += 1
            if len(self.trajectory_points) > self.trajectory_limit:
                del self.trajectory_points[:len(self.trajectory_points) - self.trajectory_limit]
        except Exception as exc:
            # Telemetry is supplementary: a pose read failure must not invalidate
            # an otherwise complete image/joint frame.
            self.trajectory_current = None
            self.trajectory_error = str(exc)

    def _finish(self, error=None):
        self.jobs.join()
        self.error = error or self.io_error
        atomic(self.episode / 'episode.json', {'frames': self.frames,
               'complete': self.error is None, 'error': self.error,
               'clock': 'simulation', 'fps': self.config['fps']})
        self.phase = 'ERROR' if self.error else 'REVIEW'

    def before_step(self):
        now = time.monotonic()
        if now - self.last_poll < .05:
            return
        self.last_poll = now
        lease = self.dir / 'lease'
        if not lease.exists() or time.time() - lease.stat().st_mtime > 15:
            if self.phase == 'RECORDING':
                self._finish('Browser/runtime heartbeat expired')
            self.enabled = False
            self.robot.stop()
            # Release GPU/device locks even when the parent API died abruptly.
            self.closed = True
            return
        if self.io_error and self.phase == 'RECORDING':
            self._finish(self.io_error)
        for path in sorted((self.dir / 'commands').glob('*.json')):
            command = json.loads(path.read_text())
            ack = {'id': command['id'], 'ok': True}
            try:
                op = command['op']
                if op == 'enable':
                    self.enabled = True
                elif op == 'disable':
                    if self.phase == 'RECORDING':
                        self._finish('Teleoperation stopped during recording')
                    self.enabled = False
                    self.robot.stop()
                elif op == 'reset':
                    if self.phase != 'READY' or self.enabled:
                        raise ValueError('Stop teleoperation and finish the current episode before resetting objects')
                    self.robot.stop()
                    self.env.scene_manager.apply_saved_poses(env_idx_list=[0])
                    self.sim_time += 40 * float(self.env.sim.step_dt)
                elif op == 'begin':
                    if self.phase != 'READY' or not self.enabled or not self.controller.active:
                        raise ValueError('Enable live teleoperation and wait for valid input before recording')
                    self.episode = self.dir / 'episodes' / command['id']
                    self.episode.mkdir(parents=True)
                    self.frames = 0
                    self.trajectory_points = []
                    self.trajectory_sample_count = 0
                    self.trajectory_current = None
                    self.trajectory_error = None
                    self.joint_history = []
                    self.error = self.io_error = None
                    self.phase = 'RECORDING'
                elif op == 'stop':
                    if self.phase != 'RECORDING':
                        raise ValueError('Not recording')
                    self._finish()
                elif op == 'enqueue':
                    # The API has durably accepted this sealed spool. Release only
                    # this slot; the export worker owns its files from now on.
                    if self.episode and self.episode.name == command.get('episode'):
                        if self.phase != 'REVIEW':
                            raise ValueError('Only a complete stopped episode can be queued')
                        self.episode = None
                        self.frames = 0
                        self.trajectory_points = []
                        self.trajectory_sample_count = 0
                        self.trajectory_current = None
                        self.trajectory_error = None
                        self.joint_history = []
                        self.error = self.io_error = None
                        self.phase = 'READY'
                elif op in ('discard', 'saved'):
                    if self.phase not in ('REVIEW', 'ERROR'):
                        raise ValueError('Stop the episode before clearing it')
                    if self.episode:
                        shutil.rmtree(self.episode)
                    self.episode = None
                    self.frames = 0
                    self.trajectory_points = []
                    self.trajectory_sample_count = 0
                    self.trajectory_current = None
                    self.trajectory_error = None
                    self.joint_history = []
                    self.error = self.io_error = None
                    self.phase = 'READY'
                elif op == 'close':
                    self.closed = True
                else:
                    raise ValueError(f'Unknown operation: {op}')
            except Exception as exc:
                ack = {'id': command['id'], 'ok': False, 'error': str(exc)}
            self._status()
            atomic(self.dir / 'acks' / path.name, ack)
            path.unlink()

    def after_step(self):
        self.tick += 1
        self.sim_time += float(self.env.sim.step_dt)
        now = time.monotonic()
        if self.phase == 'RECORDING' and (not self.controller.active or not self.enabled):
            self._finish('Leader stream stale or command rejected; episode requires inspection')
        # Same cadence as configured rendering: 10 * 0.004 = 1/25 simulation seconds.
        if self.tick % 10 == 0:
            self.env.sim.sim.render()
            preview = now - self.last_preview >= .1
            recording = self.phase == 'RECORDING'
            if (recording or preview) and now - self.started > 3:
                try:
                    images = self.cameras.read_frames()
                    sample = self.robot.read_state()
                    sample['capture_time'] = np.array([now], dtype=np.float64)
                    sample['simulation_time'] = np.array([self.sim_time], dtype=np.float64)
                    path = self.episode / f'{self.frames:06d}.npz' if recording else None
                    self.jobs.put_nowait((images, sample, path, preview))
                    if recording:
                        self.frames += 1
                        self._sample_joint_history(self.frames, self.sim_time, sample)
                        self._sample_trajectory(self.frames, self.sim_time)
                        if self.frames >= 7500 or (self.frames % 25 == 0 and shutil.disk_usage(self.dir).free < 2 * 1024**3):
                            self._finish('Episode limit or low disk space; inspect staged data')
                    if preview:
                        self.last_preview = now
                        if self.phase == 'CONNECTING':
                            self.phase = 'READY'
                            self.error = None
                except queue.Full:
                    if recording:
                        self._finish('Frame queue full; no frames silently dropped')
                except Exception as exc:
                    if recording:
                        self._finish(str(exc))
                    else:
                        self.error = str(exc)
        if now - self.last_status > .2:
            self.last_status = now
            self._status()

    def close(self):
        if self.phase == 'RECORDING':
            self._finish('Session closed during recording')
        self.robot.stop()
        self.jobs.put(None)
        self.io.join(timeout=5)
        self.phase = 'CLOSED'
        self._status()
