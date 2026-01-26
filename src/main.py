"""
Main file for running the simulator.
"""

from environment import Environment
from robot import Robot
from utils import Position, Pose, Landmark, Bounds

if __name__ == "__main__":
    # set up the environment
    # TODO: choose values for each input parameter, using the expected datatype
    dimensions = None
    dt = None
    obstacles = []
    landmarks = []
    initial_robot_pose = None

    env = Environment(
        dimensions,
        dt,
        obstacles,
        landmarks,
        initial_robot_pose,
    )

    # set up the robot
    robot = Robot(env)

    # set up timekeeping
    # TODO: set the total_seconds variable to however long you want the simulator to run (not real-time!)
    total_seconds = None
    total_timesteps = total_seconds / env.DT

    # set up logging
    ground_truth_history = []
    sensor_data_history = []

    # set up input filepath and output filepaths
    input_commands_filepath = ""
    output_ground_truth = ""
    output_sensor_data = ""

    # open up the instructions, pop the first
    with open(input_commands_filepath, "r") as cmd:
        # iterate through each timestep
        for step in range(int(total_timesteps) + 1):
            # TODO: take a ground truth snapshot and add it to the history

            # TODO: take sensor measurements and add it to the history

            # TODO: retrieve the next motor command from the input file

            # TODO: execute the motor command

        # TODO: write both history lists to their respective output files
