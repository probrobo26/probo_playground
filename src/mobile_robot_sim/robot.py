""" """

from .utils import NEAR_ZERO, floating_mod_zero
from .environment import Environment
from .sensors import SensorInterface, LandmarkPinger, GPS, Odometry

import math
import random
import pandas as pd


class Robot:
    """
    The Robot class represents a mobile robotic agent. It can execute linear and angular velocity commands to move in the world. It can also noisily sense its environment with a variety of sensors.

    Attributes:
        env (Environment): the environment this robot is operating in
        last_lin_vel (float): most recent linear velocity command
        last_ang_vel (float): most recent angular velocity command
        sensors (list[SensorInterface]): list of all robot sensors
    """

    def __init__(self, env: Environment):
        """ """
        self.env = env
        # commands from controller
        self.cmd_lin_vel = 0.0  # m/s
        self.cmd_ang_vel = 0.0  # rad/s
        # noisy execution of controller commands
        self.actual_lin_vel = 0.0  # m/s
        self.actual_ang_vel = 0.0  # rad/s

        # physical properties
        self.EXECUTION_NOISE_LINEAR = 0.03  # 3% error
        self.EXECUTION_NOISE_ANGULAR = 0.05  # 5% error
        self.sensors: dict[str, SensorInterface] = {
            "LandmarkPinger": LandmarkPinger(self),
            "GPS": GPS(self),
            "Odometry": Odometry(self),
        }

    # --- Controller Methods ---
    def agent_step_differential(self, lin_vel: float, ang_vel: float):
        """
        Differential-drive mode. Given linear and angular velocities, update the position and heading of the agent in the environment.

        Returns:
            the new position and heading of the agent
        """
        # pocket the cmds
        self.cmd_lin_vel = lin_vel
        self.cmd_ang_vel = ang_vel

        # noisify the execution proportionally
        lin_vel = lin_vel * (1 + random.gauss(0, self.EXECUTION_NOISE_LINEAR))
        ang_vel = ang_vel * (1 + random.gauss(0, self.EXECUTION_NOISE_ANGULAR))

        # pocket the actual
        self.actual_lin_vel = lin_vel
        self.actual_ang_vel = ang_vel

        # linear only; drive in a straight line
        if abs(ang_vel) < NEAR_ZERO:
            dx = lin_vel * self.env.DT * math.cos(self.env.agent_pose.theta)
            dy = lin_vel * self.env.DT * math.sin(self.env.agent_pose.theta)
            dtheta = 0.0
        # linear and angular; drive in an arc
        else:
            r = lin_vel / ang_vel
            dtheta = ang_vel * self.env.DT
            dx = r * (
                math.sin(self.env.agent_pose.theta + dtheta)
                - math.sin(self.env.agent_pose.theta)
            )
            dy = -r * (
                math.cos(self.env.agent_pose.theta + dtheta)
                - math.cos(self.env.agent_pose.theta)
            )

        # pass deltas to the env
        self.env.robot_step(dx, dy, dtheta)

    def agent_step_translational(self, x_vel: float, y_vel: float):
        """
        Swerve-drive mode. Given x and y velocities, update the position and heading of the agent in the environment.

        Returns:
            the new position and heading of the agent
        """
        # build the new pose before validating it
        dx = x_vel * self.env.DT
        dy = y_vel * self.env.DT

        # pass deltas to env
        self.env.robot_step(dx, dy, 0.0)

    # --- Sensing Methods ---
    def take_sensor_measurements(self) -> pd.DataFrame:
        # start with env time
        measurements = pd.DataFrame({"Time": [self.env.time]})

        # query all sensors -- won't have all columns every time
        for s in self.sensors.values():
            if floating_mod_zero(self.env.time, s.interval):
                print(f"--> Collecting from {s.name}")
                measurements = pd.merge(
                    measurements,
                    s.sample(),
                    left_index=True,
                    right_index=True,
                )

        # also grab the command velocities
        measurements["CMD_LinearVelocity"] = [self.cmd_lin_vel]
        measurements["CMD_AngularVelocity"] = [self.cmd_ang_vel]

        # return
        return measurements

    def take_gt_snapshot(self) -> pd.DataFrame:
        """
        Return timestep-specific GT data for CSV logging.
        """
        # grab env data: time, robot pose, gt to landmarks
        env_data = self.env.take_gt_snapshot()

        # add in actual, imperfect velocity commands
        env_data["Actual_LinearVelocity"] = self.actual_lin_vel
        env_data["Actual_AngularVelocity"] = self.actual_ang_vel

        # print("--> Ground Truth Data")
        # print(env_data.columns)
        # print(env_data.values)
        return env_data
