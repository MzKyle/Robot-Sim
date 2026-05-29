#!/usr/bin/env python3
import argparse
import os
from pathlib import Path
import subprocess
import sys
import time


SOURCE_ROOT = Path(__file__).resolve().parents[1]
if (SOURCE_ROOT / "robot_sim_scenarios").exists():
    sys.path.insert(0, str(SOURCE_ROOT))

from robot_sim_scenarios import build_world, load_scene


def build_parser():
    parser = argparse.ArgumentParser(description="Load a robot_sim scene and display it in Gazebo.")
    parser.add_argument(
        "--scene",
        default="debug_empty",
        help="Scene name from robot_sim_scenarios/scenes or an explicit YAML path.",
    )
    parser.add_argument(
        "--output-dir",
        default="",
        help="Optional directory for the generated world file.",
    )
    parser.add_argument(
        "--enable-fuel",
        action="store_true",
        help="Include optional Gazebo Fuel visual assets when generating the world.",
    )
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.enable_fuel:
        os.environ["ROBOT_SIM_ENABLE_FUEL_INCLUDES"] = "1"

    scene = load_scene(args.scene)
    output_dir = args.output_dir or None
    world_path = build_world(scene, output_dir=output_dir)
    print(f"Generated world: {world_path}")

    env = os.environ.copy()
    resource_paths = [
        str(SOURCE_ROOT),
        str(SOURCE_ROOT / "assets"),
        env.get("GZ_SIM_RESOURCE_PATH", ""),
    ]
    env["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join(path for path in resource_paths if path)

    startup_commands = scene.raw.get("startup_commands", [])
    if not startup_commands:
        return subprocess.run(["gz", "sim", "-r", str(world_path)], check=False, env=env).returncode

    process = subprocess.Popen(["gz", "sim", "-r", str(world_path)], env=env)
    try:
        for command in startup_commands:
            time.sleep(float(command.get("delay_sec", 0.0)))
            argv = [str(command["command"]), *(str(arg) for arg in command["args"])]
            print(f"Startup command: {' '.join(argv)}")
            subprocess.run(argv, check=False, env=env)
        return process.wait()
    except KeyboardInterrupt:
        process.terminate()
        return process.wait()


if __name__ == "__main__":
    sys.exit(main())
