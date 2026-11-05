# SO-ARM101 three-camera preview

Both `soarm101` and its `soarm101_pickup` alias select
`env_cfg/camera/soarm101.yml`. The ordered RGB streams are:

| Name | View | Mount |
| --- | --- | --- |
| cam_head | External front | Environment |
| cam_side | External side | Environment |
| cam_wrist | Wrist | robot0/gripper_link |

All three use the existing RoboDojo camera/capture managers and a 640x480
pinhole preset. They are available to capture code as three separate streams.
The current teleop script does not yet write a LeRobot dataset.

In desktop mode `scripts/visualize_task.py` creates the **SO101 Cameras** panel
above the main viewport. Its three live viewports are independent of the main
navigation camera. `--no-camera-panel` hides this panel if rendering cost is an
issue; the camera sensors remain configured. Camera poses can be edited in the
YAML; changing the main navigation view does not change these sensor poses.

```bash
cd /home/descfly/workspace/RoboDojo
source /home/descfly/miniconda3/etc/profile.d/conda.sh
conda activate RoboDojo
PYTHONPATH="$PWD" python -u scripts/visualize_task.py \
  --task stack_blocks --env-cfg soarm101 --device cuda:0 \
  --enable_cameras --leader-port 15051
```

Add `--camera-snapshot-dir eval_result/soarm101_three_views` to save the three
RGB PNGs after warmup and a Kit application screenshot (`isaac_ui.png`) in GUI
mode. Headless snapshot/record requests automatically enable camera rendering.
`--record` still records the first camera only; use snapshots to inspect all
three. The UI uses additional render viewports; motion latency should be
measured with this panel visible before choosing recording rates.

## Model provenance and optical frame

Camera meshes and mount transforms come from official SO-ARM100 PR 176,
commit `919efc8d6d1396b19fe6d21e5642553e3cd51bbc`:
https://github.com/TheRobotStudio/SO-ARM100/pull/176

Files are in `Assets/Robots/soarm101/camera_source/`. The reproducible overlay
builder is `scripts/teleop/build_camera_asset.py` (requires the Isaac USD Python
libraries and scipy). It writes `SO101_camera.usda`, which references the
existing `SO101.usd` and adds the two visual meshes with official fixed poses.
The registered SOArm101 articulation loads this overlay through RobotManager.
The arm joint definitions, control mapping and original USD are preserved.
The camera payload is currently visual only: added mass and camera collisions
are not simulated.

Optical reference (CAD estimate, not physical calibration):
https://github.com/adityakamath/so_arm_ros2/blob/main/so_arm_description/urdf/so101/so101.common.xacro
The fetched reference is stored with the assets. The ROS optical axes are
converted to USD camera axes. The eye is advanced 14 mm along the optical axis
to clear the opaque camera lens mesh. Actual hardware intrinsics and extrinsics
must replace this provisional preset when matching a physical wrist camera.

Source SHA256:

```
so101.common.xacro: 755c31f3e46dc4ec5f5a4ff9ddaf5ec6497f714f1b52b0b85442ea343f05ae02
so101_new_calib_camera.urdf: 4bc858e08883abfd6d5febeb07d5c4b7f682a4cd34c0ec35f821c5feac166060
wrist_camera_mount_so101_v1.stl: e17a626158951ac8cdf8d960962c1a3c8bf23b64635a02f88b62016fe895cef8
wrist_camera_so101_v1.stl: 1147857bb136876c81f98d18b533342bae2383f0e23bec216064c9f093bd3577
```

## Verification

Desktop GPU run completed with three nonblack 640x480 RGB captures and a Kit
swapchain screenshot showing the panel above the main viewport. A second run
received 843 synthetic UDP packets (no physical serial access), moved wrist
flex/roll/gripper to 0.7/0.4/0.5 rad, and exited successfully. The wrist view
changed with the arm while the external camera viewpoints remained fixed.
Results: `eval_result/soarm101_three_views/` and
`eval_result/soarm101_three_views_motion/`. The latter contains the final UI
layout with the main viewport framed on the tabletop.
