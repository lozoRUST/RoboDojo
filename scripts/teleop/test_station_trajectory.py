"""Hardware-free unit coverage for the collection station's measured pose telemetry."""
import json
import math
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace

import numpy as np

from station import RoboDojoBackend, StationBridge


class TrajectoryTelemetryTests(unittest.TestCase):
    def test_gripper_frame_offset_uses_measured_parent_quaternion(self):
        backend = RoboDojoBackend.__new__(RoboDojoBackend)
        parent = SimpleNamespace()
        backend.env = SimpleNamespace(robot_manager=SimpleNamespace(
            robot_list=[parent],
            get_link_pose=lambda *_args, **_kwargs: [np.array(
                [1.0, 2.0, 3.0, math.sqrt(.5), 0.0, 0.0, math.sqrt(.5)],
            )],
        ))
        actual = backend.read_gripper_frame_position()
        expected = np.array([1.0, 2.0, 3.0]) + np.array([
            .000218121, -.0079, -.0981274,
        ])
        np.testing.assert_allclose(actual, expected, atol=1e-8)

    def make_bridge(self, directory, robot):
        bridge = StationBridge.__new__(StationBridge)
        bridge.dir = Path(directory)
        bridge.robot = robot
        bridge.trajectory_frame_interval = 5
        bridge.trajectory_limit = 300
        bridge.trajectory_points = []
        bridge.trajectory_sample_count = 0
        bridge.trajectory_current = None
        bridge.trajectory_error = None
        return bridge

    def test_sampling_cadence_is_five_frames_and_retained_buffer_is_bounded(self):
        class Robot:
            def __init__(self):
                self.calls = 0

            def read_gripper_frame_position(self):
                self.calls += 1
                return np.array([self.calls, 0.0, 0.0])

        robot = Robot()
        bridge = self.make_bridge(tempfile.mkdtemp(), robot)
        for frame in range(1, 2001):
            bridge._sample_trajectory(frame, frame / 25)
        self.assertEqual(robot.calls, 401)  # frame 1, then each fifth frame
        self.assertEqual(bridge.trajectory_sample_count, 401)
        self.assertEqual(len(bridge.trajectory_points), 300)
        self.assertEqual(bridge.trajectory_points[0]['frame'], 505)
        self.assertEqual(bridge.trajectory_points[-1]['frame'], 2000)

    def test_pose_failure_is_telemetry_only_and_next_begin_clears_previous_episode(self):
        class BrokenRobot:
            def read_gripper_frame_position(self):
                raise RuntimeError('pose unavailable')

        with tempfile.TemporaryDirectory() as directory:
            bridge = self.make_bridge(directory, BrokenRobot())
            bridge._sample_trajectory(1, .04)
            self.assertEqual(bridge.trajectory_error, 'pose unavailable')
            self.assertEqual(bridge.trajectory_sample_count, 0)
            self.assertEqual(bridge.trajectory_points, [])

            root = Path(directory)
            (root / 'lease').touch()
            (root / 'commands').mkdir()
            (root / 'acks').mkdir()
            (root / 'episodes').mkdir()
            command = {'id': 'new-episode', 'op': 'begin'}
            (root / 'commands' / '001.json').write_text(json.dumps(command))
            bridge.phase = 'READY'
            bridge.enabled = True
            bridge.closed = False
            bridge.error = 'previous error'
            bridge.io_error = 'previous io error'
            bridge.episode = None
            bridge.frames = 17
            bridge.trajectory_points = [{'frame': 1, 'time': .04, 'position': [1, 2, 3]}]
            bridge.trajectory_sample_count = 1
            bridge.trajectory_current = [1, 2, 3]
            bridge.trajectory_error = 'stale telemetry'
            bridge.last_poll = 0
            bridge._status = lambda: None
            bridge.controller = SimpleNamespace(active=True)

            bridge.before_step()

            self.assertEqual(bridge.phase, 'RECORDING')
            self.assertEqual(bridge.frames, 0)
            self.assertEqual(bridge.trajectory_points, [])
            self.assertEqual(bridge.trajectory_sample_count, 0)
            self.assertIsNone(bridge.trajectory_current)
            self.assertIsNone(bridge.trajectory_error)
            self.assertIsNone(bridge.error)

    def test_joint_history_uses_same_cadence_and_bound(self):
        bridge = self.make_bridge(tempfile.mkdtemp(), SimpleNamespace())
        bridge.joint_history = []
        bridge.joint_history_limit = 300
        for frame in range(1, 2001):
            bridge._sample_joint_history(frame, frame / 25, {
                'observation.state': np.zeros(6), 'action': np.ones(6) * frame,
            })
        self.assertEqual(len(bridge.joint_history), 300)
        self.assertEqual(bridge.joint_history[0]['frame'], 505)
        self.assertEqual(bridge.joint_history[-1]['frame'], 2000)
        self.assertEqual(bridge.joint_history[-1]['action'][0], 2000)


if __name__ == '__main__':
    unittest.main()
