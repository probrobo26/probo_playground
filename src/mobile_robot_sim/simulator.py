"""
Main file for running the simulator.
"""

from .environment import Environment
from .robot import Robot
from .utils import Pose, Position, Bounds, Landmark
import csv
import pandas as pd

if __name__ == "__main__":
    # set up the sim
    env = Environment(
        dimensions=Bounds(0, 30, 0, 30),
        agent_pose=Pose(Position(0, 0), 0),
        obstacles=[
            Bounds(0, 5, 25, 30),
            Bounds(5, 10, 10, 15),
            Bounds(20, 25, 5, 15),
        ],
        landmarks=[
            Landmark(Position(5, 5), 0),
            Landmark(Position(20, 5), 1),
            Landmark(Position(10, 15), 2),
            Landmark(Position(15, 30), 3),
            # Landmark(Position(25, 25), 4),
        ],
        timestep=0.1,
    )
    robot = Robot(env)

    # set up timekeeping
    total_seconds = 120.0
    total_timesteps = total_seconds / env.DT
    terminal = False

    # set up logging
    ground_truth_history = pd.DataFrame()
    sensor_data_history = pd.DataFrame()

    # open up the instructions, pop the first
    with open("./cmds/vel_cmd_example.csv", "r") as cmd:
        # start popping velocity commands
        vel_cmds = csv.reader(cmd)
        next_cmd = next(vel_cmds)  # skip column names
        next_cmd = next(vel_cmds)
        current_lin_vel = float(next_cmd[1])
        current_ang_vel = float(next_cmd[2])

        # hit it!
        for step in range(int(total_timesteps) + 1):
            # first, sample the environment
            print(f"\n***TIMESTEP T{env.time}***")
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
        ground_truth_history.to_csv("./logs/groundtruth_log.csv")
        sensor_data_history.to_csv("./logs/sensor_log.csv")
