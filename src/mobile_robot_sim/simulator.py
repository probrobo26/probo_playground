"""
Main file for running the simulator.
"""

from environment import Environment
from robot import Robot
from utils import Pose, Position, Bounds, Landmark
from viz import Visualizer
import csv
import pandas as pd
import pickle

if __name__ == "__main__":
    # set up the sim
    env = Environment(
        dimensions=Bounds(0, 50, 0, 40),
        agent_pose=Pose(Position(0, 0), 0),
        obstacles=[
            Bounds(15, 25, 8, 12),
            Bounds(40, 45, 10, 15),
            Bounds(10, 20, 20, 25),
            Bounds(15, 20, 30, 35),
            Bounds(30, 35, 20, 30),
        ],
        landmarks=[
            Landmark(Position(10, 10), 0),
            Landmark(Position(10, 40), 1),
            Landmark(Position(20, 0), 2),
            Landmark(Position(20, 20), 3),
            Landmark(Position(30, 10), 4),
            Landmark(Position(30, 20), 5),
            Landmark(Position(20, 30), 6),
            Landmark(Position(40, 30), 7),
            Landmark(Position(50, 10), 8),
            Landmark(Position(50, 20), 9),
        ],
        timestep=0.1,
    )
    robot = Robot(env)

    # set up timekeeping
    total_seconds = 170
    total_timesteps = total_seconds / env.DT
    terminal = False

    # set up logging
    ground_truth_history = pd.DataFrame()
    sensor_data_history = pd.DataFrame()
    VIZ = True

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
        # ground_truth_history.to_csv("./logs/groundtruth_log.csv")
        # sensor_data_history.to_csv("./logs/sensor_log.csv")
        pickle.dump(ground_truth_history, open("./logs/groundtruth_log.pkl", "wb"))
        pickle.dump(sensor_data_history, open("./logs/sensor_log.pkl", "wb"))
        pickle.dump(env.info(), open("./logs/env_info.pkl", "wb"))
        pickle.dump(robot.info(), open("./logs/sensor_info.pkl", "wb"))
        # robot.info().to_csv("./logs/sensor_info.csv")
        # print(robot.info())

    if VIZ:
        # grab all the data
        DATA_PATH = "./logs/"
        gt_log_path = DATA_PATH + "groundtruth_log.pkl"
        sensor_log_path = DATA_PATH + "sensor_log.pkl"
        env_info_path = DATA_PATH + "env_info.pkl"
        sensor_info_path = DATA_PATH + "sensor_info.pkl"
        with open(sensor_log_path, "rb") as f:
            sensor_log = pickle.load(f)
        with open(gt_log_path, "rb") as f:
            gt_log = pickle.load(f)
        # unpack into dataframes
        with open(env_info_path, "rb") as f:
            env_info = pickle.load(f)
        with open(sensor_info_path, "rb") as f:
            sensor_info = pickle.load(f)

        # make visuals
        viz = Visualizer(
            env_info,
            gt_log,
            sensor_log,
        )
        viz.draw_all()
