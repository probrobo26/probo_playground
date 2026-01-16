"""
Main file for running the simulator.
"""

from environment import Environment
from robot import Robot
from utils import Pose, Position, Bounds, Landmark
from viz import Visualizer
import pandas as pd
from pathlib import Path
import csv, argparse, pickle, yaml

if __name__ == "__main__":
    # set up pathing and argparse
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-y",
        "--config",
        type=Path,
        default=Path("./input/config_example.yaml"),
        help="Path to config.yaml file.",
    )
    parser.add_argument(
        "-c",
        "--cmds",
        type=Path,
        default=Path("./input/vel_cmd_example.csv"),
        help="Path to motor commands file.",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=Path("./output/"),
        help="Indicate the desired output folder.",
    )
    parser.add_argument(
        "-v",
        "--viz_only",
        help="Indicate this to plot data that is already present in the output/ directory.",
        action="store_true",
    )
    parser.add_argument(
        "-a",
        "--animate",
        help="Indicate this to create an animated GIF of the trajectory.",
        action="store_true",
    )
    args = parser.parse_args()
    CONFIG_PATH: Path = args.config
    assert CONFIG_PATH.exists()
    CMD_PATH: Path = args.cmds
    assert CMD_PATH.exists()
    OUTPUT_PATH: Path = args.output
    assert OUTPUT_PATH.exists()
    VIZ_ONLY: bool = args.viz_only
    ANIMATE: bool = args.animate

    # if not viz only, run the simulator and log the results
    if not VIZ_ONLY:
        # pull config info
        with open(CONFIG_PATH, "r") as f:
            config = yaml.safe_load(f)
            env_info = config["environment"]
            robot_info = config["robot"]
            sensor_info = config["sensors"]
        # set up the sim
        obstacles = []
        for dims in env_info["obstacles"]:
            obstacles.append(
                Bounds(
                    dims[0],
                    dims[1],
                    dims[2],
                    dims[3],
                )
            )
        landmarks = []
        for lm in env_info["landmarks"]:
            landmarks.append(
                Landmark(
                    Position(
                        lm[0],
                        lm[1],
                    ),
                    len(landmarks),
                )
            )
        env = Environment(
            dimensions=Bounds(
                0,
                env_info["width"],
                0,
                env_info["height"],
            ),
            agent_pose=Pose(
                Position(
                    env_info["robot_start"][0],
                    env_info["robot_start"][1],
                ),
                env_info["robot_start"][2],
            ),
            obstacles=obstacles,
            landmarks=landmarks,
            timestep=env_info["timestep"],
            lm_range=env_info["pinger_range"],
        )
        robot = Robot(
            env,
            robot_info,
            sensor_info,
        )

        # set up timekeeping
        total_seconds = env_info["runtime"]
        total_timesteps = total_seconds / env.DT
        terminal = False

        # set up logging
        ground_truth_history = pd.DataFrame()
        sensor_data_history = pd.DataFrame()

        # open up the instructions, pop the first
        with open(CMD_PATH, "r") as cmd:
            # start popping velocity commands
            vel_cmds = csv.reader(cmd)
            next_cmd = next(vel_cmds)  # skip column names
            next_cmd = next(vel_cmds)
            current_lin_vel = float(next_cmd[1])
            current_ang_vel = float(next_cmd[2])

            # hit it!
            for step in range(int(total_timesteps) + 1):
                # first, sample the environment
                # print(f"\n***TIMESTEP T{env.time}***")
                ground_truth_history = pd.concat(
                    [
                        ground_truth_history,
                        robot.take_gt_snapshot(),
                    ],
                    ignore_index=True,
                )
                sensor_data_history = pd.concat(
                    [
                        sensor_data_history,
                        robot.take_sensor_measurements(),
                    ],
                    ignore_index=True,
                )

                # # retrieve new command if available or passed
                if round(float(next_cmd[0]), 3) <= env.DT * step and not terminal:
                    current_lin_vel = float(next_cmd[1])
                    current_ang_vel = float(next_cmd[2])
                    # out of commands
                    try:
                        next_cmd = next(vel_cmds)
                    except StopIteration:
                        terminal = True

                # move the robot with current commands
                robot.agent_step_differential(current_lin_vel, current_ang_vel)

            # log the results
            pickle.dump(
                ground_truth_history, open(OUTPUT_PATH / "groundtruth_log.pkl", "wb")
            )
            pickle.dump(sensor_data_history, open(OUTPUT_PATH / "sensor_log.pkl", "wb"))
            pickle.dump(env.info(), open(OUTPUT_PATH / "env_info.pkl", "wb"))
            pickle.dump(robot.info(), open(OUTPUT_PATH / "sensor_info.pkl", "wb"))
            print("Done running simulator")

    # plot the results
    viz = Visualizer(
        OUTPUT_PATH,
    )
    viz.draw_all()

    if ANIMATE:
        viz.animate_trajectories()
