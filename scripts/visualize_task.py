#!/usr/bin/env python3
"""Open a RoboDojo task environment in the Isaac Sim viewport.

Example:
    python scripts/visualize_task.py --task general_pickup --env-cfg soarm101
"""

import argparse
from pathlib import Path
import time

from isaaclab.app import AppLauncher

ROOT = Path(__file__).resolve().parents[1]

parser = argparse.ArgumentParser(description="Visualize a RoboDojo task environment.")
parser.add_argument("--task", default="general_pickup")
parser.add_argument("--env-cfg", default="soarm101")
parser.add_argument("--seed", type=int, default=0)
parser.add_argument("--layout-id", type=int, default=0, help="Layout index within the selected seed set.")
parser.add_argument("--check-only", action="store_true", help="Validate scene placement, print prim paths, then exit.")
parser.add_argument("--record", type=Path, default=None, help="Write the head camera to an MP4 file.")
parser.add_argument("--seconds", type=float, default=10.0, help="Recording duration in seconds.")
parser.add_argument("--fps", type=int, default=25)
parser.add_argument("--leader-port", type=int, help="Listen for local leader UDP packets (e.g. 15051).")
parser.add_argument("--station-dir", type=Path, help="Managed collection session IPC directory.")
parser.add_argument("--camera-snapshot-dir", type=Path, help="Save one RGB PNG per configured camera after warmup.")
parser.add_argument("--no-camera-panel", action="store_true", help="Hide the top camera panel to reduce GUI rendering load.")
parser.add_argument("--leader-max-speed", type=float, default=5.0,
                    help="Leader target slew limit in rad/s (default 5).")
parser.add_argument("--run-seconds", type=float, default=None, help="Stop after this many seconds; useful for smoke checks.")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
if args.headless and (args.camera_snapshot_dir or args.record):
    args.enable_cameras = True
if not 0 < args.leader_max_speed <= 10:
    parser.error('--leader-max-speed must be >0 and <=10 rad/s.')

# Launch with a window unless the caller explicitly supplies --headless.
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

from omegaconf import OmegaConf

from task.RoboDojo.task_registry import load_task_class, task_config_path
from env.seed_manager.seed_manager import SeedManager


def load_environment_config():
    env_cfg_path = ROOT / "env_cfg" / f"{args.env_cfg}.yml"
    if not env_cfg_path.is_file():
        raise FileNotFoundError(f"Environment config not found: {env_cfg_path}")
    eval_cfg = OmegaConf.load(env_cfg_path)
    task_name, task_class = load_task_class(args.task)
    sim = OmegaConf.load(ROOT / "env_cfg" / "sim" / f"{eval_cfg.config.sim}.yml")
    sim.scene.num_envs = 1
    sim.device = args.device
    sim.seed = [args.seed]
    sim.physx = {}
    camera = OmegaConf.load(ROOT / "env_cfg" / "camera" / f"{eval_cfg.config.camera}.yml")
    camera.default_frequency = eval_cfg.get("observation", {}).get("collect_freq", 25)
    # CameraView uses CUDA annotators in Isaac Sim; keep its preallocated Warp
    # buffers on the same device to avoid native device mismatch failures.
    for capture_cfg in camera.annotator.values():
        for annotator_name, annotator_cfg in capture_cfg.items():
            if annotator_name != "enabled" and "device" in annotator_cfg:
                annotator_cfg.device = "cuda"
    result = OmegaConf.create(
        {
            "sim": sim,
            "scene": OmegaConf.load(ROOT / "env_cfg" / "scene" / f"{eval_cfg.config.scene}.yml"),
            "robot": OmegaConf.load(ROOT / "env_cfg" / "robot" / f"{eval_cfg.config.robot}.yml"),
            "camera": camera,
            "task_env": OmegaConf.load(task_config_path(ROOT / "task" / "RoboDojo" / "config", task_name)),
            "eval_cfg": {
                "config_name": eval_cfg.config_name,
                "task_name": task_name,
                "seed": args.seed,
                "num_envs": 1,
                "observation": eval_cfg.get("observation", {}),
            },
        }
    )
    return task_class, result


def main():
    task_class, config = load_environment_config()
    env = task_class(config, simulation_app)
    # Direct task construction does not go through EvalEnv, so load one of the
    # saved RoboDojo layouts explicitly. This supplies the table, room, ground,
    # and task objects in addition to the robot.
    seed_manager = SeedManager(config.eval_cfg)
    seed_manager.init_eval()
    layout = seed_manager.get_seed_scene_info(args.layout_id)
    env.scene_manager.layout_manager.set_saved_layout(0, layout)
    print(f"[visualize] Loaded layout: {seed_manager.seed_info[args.layout_id]['scene_layout']}")
    print(f"[visualize] task={args.task}, env_cfg={args.env_cfg}, device={args.device}")
    print("[visualize] Resetting environment...")
    env.reset(seed=[args.seed])
    # EvalEnv.setup_scene() restores tables and objects after TaskEnv.reset()
    # temporarily moves them offscreen. Follow that same placement lifecycle.
    env.scene_manager.apply_saved_poses(env_idx_list=[0])
    env.robot_manager.set_origin_endpose()
    env.robot_manager.set_robot_init_state()
    if config.eval_cfg.config_name == 'soarm101':
        env.sim.sim.set_camera_view(eye=[0.85, -1.1, 1.5], target=[0.2, -0.1, 0.85])
    # Evaluation bookkeeping and reward baselines belong to EvalEnv; this
    # preview only initializes the scene and robot control state.
    from isaacsim.core.utils.stage import get_current_stage
    from pxr import UsdGeom
    stage = get_current_stage()
    scene_config = env.scene_manager.config["env0"]
    paths = [scene_config["Table"]["prim_path"]] if "Table" in layout else []
    for category in ("Rigid", "Geometry", "Articulation"):
        for instances in scene_config.get(category, {}).values():
            paths.extend(instance["prim_path"] for instance in instances)
    for path in paths:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            raise RuntimeError(f"Scene prim missing: {path}")
        pos = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0).ExtractTranslation()
        if max(abs(float(v)) for v in pos) > 10:
            raise RuntimeError(f"Scene prim remains offscreen: {path}: {pos}")
        print(f"[visualize] scene prim: {path}, position={pos}", flush=True)
    if args.check_only:
        print("[visualize] SCENE_CHECK_OK", flush=True)
        env.close()
        return
    print("[visualize] Ready. Close the Isaac Sim window to exit.")
    writer = None
    if args.record is not None:
        import cv2

        args.record.parent.mkdir(parents=True, exist_ok=True)
        # The first camera in the RoboDojo camera config is the head camera.
        sample = env.capture_manager.step(env_ids=[0], cam_ids=[0])[0]["rgb"][0]["data"]
        height, width = sample.shape[:2]
        writer = cv2.VideoWriter(
            str(args.record), cv2.VideoWriter_fourcc(*"mp4v"), args.fps, (width, height)
        )
        if not writer.isOpened():
            raise RuntimeError(f"Could not open video writer: {args.record}")
        print(f"[visualize] Recording {width}x{height} to {args.record}")
        record_deadline = time.monotonic() + args.seconds
    else:
        record_deadline = None
    leader = None
    camera_panel = None
    station = None
    try:
        if config.eval_cfg.config_name == 'soarm101' and not args.headless and not args.no_camera_panel:
            from teleop.camera_panel import CameraPanel
            camera_panel = CameraPanel(env.camera_manager)
        if args.leader_port is not None:
            from teleop.sim_control import LeaderControl
            leader = LeaderControl(env, args.leader_port, max_speed=args.leader_max_speed)
        if args.station_dir:
            if leader is None:
                raise ValueError('--station-dir requires --leader-port')
            from teleop.station import StationBridge
            station = StationBridge(args.station_dir, env, leader)
        run_started = time.monotonic()
        snapshots_saved = False
        while simulation_app.is_running():
            if station is not None:
                station.before_step()
                if station.closed:
                    break
            if leader is not None and (station is None or station.enabled):
                leader.step()
            env.sim_step(render=station is None)
            if station is not None:
                station.after_step()
            if args.camera_snapshot_dir and not snapshots_saved and time.monotonic() - run_started > 3:
                import cv2
                args.camera_snapshot_dir.mkdir(parents=True, exist_ok=True)
                frames = env.capture_manager.step(env_ids=[0])
                for name, frame in zip(env.camera_manager.camera_names[0], frames):
                    output = args.camera_snapshot_dir / f'{name}.png'
                    rgb = frame['rgb'][0]['data']
                    if not cv2.imwrite(str(output), cv2.cvtColor(rgb[:, :, :3], cv2.COLOR_RGB2BGR)):
                        raise RuntimeError(f'Could not save {output}')
                    print(f'[cameras] Saved {name}: {output}, shape={rgb.shape}', flush=True)
                snapshots_saved = True
                if not args.headless:
                    import omni.kit.renderer.capture
                    import omni.appwindow
                    capture = omni.kit.renderer.capture.acquire_renderer_capture_interface()
                    capture.capture_next_frame_swapchain(
                        str(args.camera_snapshot_dir / 'isaac_ui.png'),
                        omni.appwindow.get_default_app_window())
            if args.run_seconds is not None and time.monotonic() - run_started >= args.run_seconds:
                break
            if writer is not None:
                import cv2

                frame = env.capture_manager.step(env_ids=[0], cam_ids=[0])[0]["rgb"][0]["data"]
                writer.write(cv2.cvtColor(frame[:, :, :3], cv2.COLOR_RGB2BGR))
                if time.monotonic() >= record_deadline:
                    break
    finally:
        if station is not None:
            station.close()
        if camera_panel is not None:
            camera_panel.close()
        if leader is not None:
            leader.close()
        if writer is not None:
            writer.release()
            print(f"[visualize] Video written: {args.record}")
        env.close()


if __name__ == "__main__":
    try:
        main()
    except BaseException:
        # Kit's close may terminate the process; report errors before closing.
        import traceback
        traceback.print_exc()
        raise
    finally:
        simulation_app.close()
