# SO-ARM leader → Isaac Sim

The leader reader runs in `data_collection`, using the installed LeRobot commit
`0cf864870cf29f4738d3ade893e6fd13fbd7cdb5`, its `So101RobotConfig` motor map and
`FeetechMotorsBus.read("Present_Position")`. Isaac Sim runs separately in `RoboDojo`.
Only the local UDP bridge and simulation actuator adapter are new.

Current defaults for this leader: 100 Hz requested reader rate, 5 rad/s
simulation target slew limit, and absolute-mode calibrated signs
`1,-1,1,1,-1,1`. These can be overridden with `--hz`, `--leader-max-speed`,
and `--calibrated-signs`. Relative mode retains all-positive direction defaults.
Absolute mode still requires `--mode absolute --calibration PATH`.

## Start

SO-ARM101 now uses two external RGB cameras and one wrist RGB camera, with a
top-docked live panel in desktop mode. See [CAMERAS.md](CAMERAS.md) for setup,
asset provenance, camera pose settings and snapshot commands.

Terminal 1, desktop:

```bash
cd /home/descfly/workspace/RoboDojo
source /home/descfly/miniconda3/etc/profile.d/conda.sh
conda activate RoboDojo
PYTHONPATH="$PWD" python -u scripts/visualize_task.py --task stack_blocks --env-cfg soarm101 --device cuda:0 --leader-port 15051
```

Terminal 2, after the simulator prints `Listening`:

```bash
cd /home/descfly/workspace/RoboDojo
/home/descfly/miniconda3/envs/data_collection/bin/python scripts/teleop/read_leader.py --port /dev/serial/by-id/usb-1a86_USB_Single_Serial_5A46081874-if00 --probe
```

When all six positions are readable, support the leader, then start:

```bash
/home/descfly/miniconda3/envs/data_collection/bin/python scripts/teleop/read_leader.py --port /dev/serial/by-id/usb-1a86_USB_Single_Serial_5A46081874-if00 --mode absolute --calibration /home/descfly/workspace/RoboDojo/.cache/calibration/so101/main_leader_v2.json
```

The connected leader was verified torque-disabled. If torque is enabled, support
the leader and add `--free-leader` to disable it, like LeRobot's leader setup.
No Goal_Position commands are sent to physical motors. Ctrl+C stops the reader.

## Mapping and limits

### Absolute mode (after calibration)

`--mode absolute --calibration PATH` uses calibrated positions, not startup-pose
deltas. `mapping.py` maps the pinned LeRobot old-calibration arm degrees to the
installed `so101_new_calib` model with radian offsets:

```
shoulder_pan:   0
shoulder_lift: +pi/2
elbow_flex:    -pi/2
wrist_flex:    0
wrist_roll:   +0.0486795
```

Equivalent angles are wrapped into [-pi, pi) before the simulator clamps joint
limits. Gripper 0–100 percent maps to 0–100 degrees of the follower jaw. Physical
trigger and follower jaw need not have identical angles. Non-default `--signs`
are rejected in absolute mode because calibration already supplies drive signs.
The leader pose remains the absolute target after reconnect; the simulated robot
approaches it at the configured slew limit.

Do not use `--sim-signs=1,-1,1,1,-1,1` as a hardware-direction fix: it
negates the offsets as well as the angles and changes the old Zero pose's
mapped configuration. This earlier trial worsened the user's pose agreement.
Keep the default all +1 while checking reference poses. The old Zero pose
should read approximately zero calibrated arm degrees and maps to new-model
degrees `[0, 90, -90, 0, 2.789]`. First check that reference, then a second
known pose to establish direction. A direction correction must preserve a
verified reference: `q_sim = q_sim_ref + sign * (q_leader - q_leader_ref)`.
The calibration file is unchanged by the simulation-sign option.

For the user's reported reversed wrist-roll direction, use absolute-mode
`--calibrated-signs=1,1,1,1,-1,1` (do not combine with `--sim-signs`). This
reflects joint 5 about calibrated Zero BEFORE adding the 2.789-degree model
offset. It preserves nominal Zero; physical tracking still needs verification.
Joint 2 remains unchanged pending measurements. The reader prints calibrated
values once per second. After restarting the simulator, its log prints actual
effective limits and, once per second, joints 2/5 requested, limited, slew-limited
command and measured angles in degrees. `LIMIT_CLIPPED` identifies target
saturation. Shoulder URDF limits are about +/-100 degrees; the reported Zero
reading of +5.625 degrees already maps to +95.625 degrees before clipping.

Follow-up: the user reports joint 2 crossing from about -2.99 to +3.1 rad
near Middle. With a calibrated shoulder angle near +90 degrees, adding
+90 degrees places the target at the wrap boundary. For this hardware check,
use `--calibrated-signs=1,-1,1,1,-1,1`: shoulder target becomes
`-calibrated_angle + 90 degrees`, wrist-roll becomes
`-calibrated_angle + 2.789 degrees`. Both preserve nominal legacy Zero.
This interpretation still requires physical pose verification; it is not a
change to the official model-to-model conversion. The simulator now holds
measured pose for absolute packets exceeding any effective joint limit by
more than 5 degrees and logs `INVALID_ABSOLUTE_TARGET`. This prevents wrapped,
invalid targets from alternately commanding opposite end stops. Restart the
simulator to load this guard. Small excesses retain the existing limit clamp.

The offsets were derived by aligning the common base STL and comparing visual
mesh transforms across the official old/new URDFs. Forty random configurations
matched with maximum transform-element difference below 1.5e-5. Source:
https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/so101_old_calib.urdf

Model SHA256 values used for this comparison:
- old: `a8151e4f7e5818d066680671d372ce49e7d30c4d068edb1666ab1bab2d5a8afc`
- installed new: `3a65d2d35e68a8d2f0c2cc176d19b884506543c93ba72980145b80abe276022c`

This is a model-coordinate check; physical calibration and assembly accuracy
still determine how closely real and simulated poses match. The initial
`main_leader.json` failed the wrist-roll zero check; use the validated v2 file.

### Relative mode (legacy default)

Omitting `--mode absolute` selects relative joint teleoperation.
On connection/reconnection the simulator anchors its current pose to the current
leader pose. All six joints use angular changes; without `--calibration`, the
reader unwraps STS3215 raw encoder ticks (4096 per revolution). With a calibration
JSON from this LeRobot version, it uses arm degrees and gripper percentage.
The latter maps a full gripper range to 100 degrees in the simulation.

The six `--signs` values (default `1,1,1,1,1,1`) correspond to shoulder_pan,
shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper. Check directions one
joint at a time on the physical leader before task use. Signs and absolute pose
correspondence have not been verified on the connected hardware.

### Common limits

The simulation applies joint targets through the registered articulation,
clamped to USD soft limits and a default 5 rad/s target slew limit. After 0.5 seconds
without fresh packets it holds measured joint positions. Only relative mode
reanchors on reconnect.
This script does not record a LeRobot dataset or run CuRobo.

## Validation status

### Tracking latency tuning

After the user confirmed directions work with
`--calibrated-signs=1,-1,1,1,-1,1`, these became the absolute-mode defaults.
The requested reader rate now defaults to 100 Hz and target slew to 5 rad/s.
This also speeds up initial pose convergence; limits and invalid
target holding still apply. No physical motor gains or calibration are changed.

Reader logs effective rate and mean serial read time. Simulation logs packet
age (since the post-read timestamp, excluding serial acquisition), loop rate,
simulation-time/wall-time ratio, maximum slew gap and actuator tracking gap in
degrees. Loop rate is zero until the first one-second measurement window.
Large slew gap indicates target limiting; large tracking gap indicates dynamics,
contact or actuator lag. A sim/wall ratio below 1 means simulation advances
slower than real time. Packet age is not end-to-end visual latency. The GUI is
rendered every 10 physics steps (25 simulated frames/s at dt=0.004).
CPU checks verified the configurable limiter and preserved invalid-target hold;
the faster settings have not yet been measured with moving hardware and GPU.

With `main_leader_v2.json`, the eight-second absolute-mode check received 218
real leader packets. The simulated joint positions converged to approximately
`[0.039883, -1.294668, 1.468009, 0.983288, 0.060951, 0.004295]` rad, matching the
mapped live leader target to within 0.001 rad at that stationary pose.

Local packet validation/stale rejection passed. A synthetic UDP stream sent 75
packets to an actual GPU task scene; shoulder_pan reached approximately 0.1 rad,
then held on timeout. This validates the simulation bridge, not hardware tracking.
After connecting motor power, all six physical motors responded consistently and
reported torque disabled. A five-second real-leader-to-GPU-simulation check
received 128 packets and exited normally. The leader remained stationary, so
physical joint directions and grasping are still unverified.
Physical serial access has been granted for the current USB connection. USB replug may require:

```bash
sudo setfacl -m u:"$(id -un)":rw /dev/ttyACM0
```
