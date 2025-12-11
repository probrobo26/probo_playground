"""
Contains the abstract base class for sensor classes to inherit from, plus all implemented sensor classes.
"""

# from robot import Robot
from utils import BearingRange, Position
import pandas as pd
from abc import ABC, abstractmethod
import math
import random


class SensorInterface(ABC):
    """
    Interface to standardize all sensors.

    Attributes:
        name (str): string identifier
        robot (Robot): reference robot. required for observing the environment
        interval (float): period between measurements
        last_meas_t (float): time since last measurement
    """

    def __init__(self, name: str, robot, interval: float):
        """
        Initialize a sensor class instace.

        Args:
            name (str): reference identifier
            robot (Robot): reference robot
            interval (float): period between measurements
            last_meas_t (float): time of last measurement
        """
        self._name = name
        self.robot = robot
        self._interval = interval
        self._last_meas_t = -math.inf

    @property
    def name(self) -> str:
        """
        Getter for the name property.
        """
        return self._name

    @property
    def interval(self) -> float:
        """
        Getter for the interval property.
        """
        return self._interval

    @property
    def last_meas_t(self) -> float:
        """
        Getter for the time of last measurement property.
        """
        return self._last_meas_t

    @last_meas_t.setter
    def last_meas_t(self, value: float):
        """
        Setter for the time of last measurement property.
        """
        self._last_meas_t = value

    @abstractmethod
    def sample(self) -> pd.DataFrame:
        """
        Sample the environment and return the noisy measurement(s).
        """
        pass


# --- Exteroceptive Sensors ---
# these measure the robot's physical relationship to the world and landmarks
class GPS(SensorInterface):
    """
    This class represents a GPS sensor that measures the position of the robot in 2D space.

    Attributes:
        name (str): string identifier
        robot (Robot): reference robot
        interval (float): period between measurements
        last_meas_t (float): time of last measurement
        GPS_NOISE (float): absolute noise for stdev
    """

    def __init__(
        self,
        robot,
        name: str = "GPS",
        interval: float = 2.0,
        noise: float = 0.5,
    ):
        """
        Initialize an instance of the GPS class.

        Args:
            name (str): reference identifier
            robot (Robot): reference robot
            interval (float): period between measurements
            noise (float): absolute noise for stdev
        """
        super().__init__(name, robot, interval)
        self.GPS_NOISE = noise

    def sample(self) -> pd.DataFrame:
        """
        Take a noisy GPS measurement of robot position.
        """
        gt_pose = self.robot.env.get_gt_robot_pose()
        x_noisy = random.gauss(gt_pose.pos.x, self.GPS_NOISE)
        y_noisy = random.gauss(gt_pose.pos.y, self.GPS_NOISE)

        return pd.DataFrame({self.name: [Position(x_noisy, y_noisy)]})


class LandmarkPinger(SensorInterface):
    """
    This class represents a sensor that measures the range and bearing between the robot and the floating-point landmarks on the map. In practice, this sensor could be a ToF sensor, a node in a network of beacons, or even a camera.

    Attributes:
        name (str): reference identifier
        robot (Robot): reference robot
        interval (float): period between measurements
        MAX_RANGE (int): maximum distance from a beacon for it to be visible
        RANGE_NOISE (float): absolute noise for range stdev
        RANGE_NOISE_RATIO (float): porportional noise for range stdev
        BEARING_NOISE (float): absolute noise for bearing stdev
    """

    def __init__(
        self,
        robot,
        name: str = "LandmarkPinger",
        interval: float = 0.5,
    ):
        """
        Initialize an instance of the LandmarkPinger class.

        Args:
            name (str): reference identifier
            robot (Robot): reference robot
            interval (float): period between measurements
        """
        super().__init__(name, robot, interval)
        self.MAX_RANGE = 10.0  # meters
        self.RANGE_NOISE = 0.2  # meters
        self.RANGE_PROP_NOISE = 0.02  # meters
        self.BEARING_NOISE = 0.0872665  # radians

    def sample(self) -> pd.DataFrame:
        """
        Noisily measure the bearing and range between the robot and all nearby landmarks.

        Returns:
            A dictionary mapping every known landmark to the noisy measurement relating it to the robot.
        """
        # setup
        to_landmarks_noisy = pd.DataFrame()
        gt_landmark_dists: pd.DataFrame = self.robot.env.get_gt_to_landmarks()
        # go through each landmark measurement -- each has its own column
        for lm in gt_landmark_dists.columns:
            gt: BearingRange = gt_landmark_dists[lm].values[0]  # gt value for lm
            if gt.range <= self.MAX_RANGE:
                br_noisy = BearingRange(
                    random.gauss(gt.bearing, self.BEARING_NOISE),
                    random.gauss(
                        gt.range,
                        self.RANGE_NOISE + self.RANGE_PROP_NOISE * gt.range,
                    ),  # noise is porportional to distance
                )
            else:
                br_noisy = BearingRange(math.inf, math.inf)
            to_landmarks_noisy[f"{self.name}_{lm}"] = [br_noisy]
        return to_landmarks_noisy


# --- Proprioceptive Sensors ---
# these measure the robot's physical relationship to its prior states
class Odometry(SensorInterface):
    """
    This class represents an odometry sensor that measures the robot's velocity based on wheel encoders.
    Reports noisy estimates of linear and angular velocities.

    Attributes:
        name (str): string identifier
        robot (Robot): reference robot
        interval (float): period between measurements
        last_meas_t (float): time of last measurement
        LINEAR_NOISE_RATIO (float): proportional noise for linear velocity stdev
        ANGULAR_NOISE_RATIO (float): proportional noise for angular velocity stdev
    """

    def __init__(
        self,
        robot,
        name: str = "Odometry",
        interval: float = 0.1,
        linear_noise_ratio: float = 0.05,
        angular_noise_ratio: float = 0.10,
    ):
        """
        Initialize an instance of the Odometry class.

        Args:
            robot (Robot): reference robot
            name (str): reference identifier
            interval (float): period between measurements
            linear_noise_ratio (float): proportional noise for linear velocity
            angular_noise_ratio (float): proportional noise for angular
        """
        super().__init__(name, robot, interval)
        self.LINEAR_NOISE_RATIO = linear_noise_ratio
        self.ANGULAR_NOISE_RATIO = angular_noise_ratio

    def sample(self) -> pd.DataFrame:
        """
        Take a noisy odometry measurement of the robot's velocities.

        Returns:
            A dictionary containing noisy linear and angular velocity estimates.
        """
        # Get the commanded velocities from the robot
        true_lin_vel = self.robot.actual_lin_vel
        true_ang_vel = self.robot.actual_ang_vel

        # Add proportional noise
        noisy_lin_vel = random.gauss(
            true_lin_vel, abs(true_lin_vel) * self.LINEAR_NOISE_RATIO
        )
        noisy_ang_vel = random.gauss(
            true_ang_vel, abs(true_ang_vel) * self.ANGULAR_NOISE_RATIO
        )

        return pd.DataFrame(
            {
                f"{self.name}_LinearVelocity": [noisy_lin_vel],
                f"{self.name}_AngularVelocity": [noisy_ang_vel],
            }
        )


# NOTE: the IMU and VIO classes were both generated by Claude, I haven't checked them or rewritten it myself yet
# class IMU(SensorInterface):
#     """
#     This class represents an IMU (Inertial Measurement Unit) that measures linear and angular acceleration.
#     Note: IMUs suffer from bias drift over time, making long-term integration challenging.

#     Attributes:
#         name (str): string identifier
#         robot (Robot): reference robot
#         interval (float): period between measurements
#         last_meas_t (float): time of last measurement
#         LINEAR_ACCEL_NOISE (float): absolute noise for linear acceleration stdev (m/s²)
#         ANGULAR_ACCEL_NOISE (float): absolute noise for angular acceleration stdev (rad/s²)
#         LINEAR_BIAS_DRIFT (float): bias drift rate for linear acceleration (m/s² per second)
#         ANGULAR_BIAS_DRIFT (float): bias drift rate for angular acceleration (rad/s² per second)
#     """

#     def __init__(
#         self,
#         robot: Robot,
#         name: str = "IMU",
#         interval: float = 0.01,  # IMUs are very fast, 100 Hz
#         linear_accel_noise: float = 0.05,
#         angular_accel_noise: float = 0.05,
#         linear_bias_drift: float = 0.001,
#         angular_bias_drift: float = 0.001,
#     ):
#         """
#         Initialize an instance of the IMU class.

#         Args:
#             robot (Robot): reference robot
#             name (str): reference identifier
#             interval (float): period between measurements
#             linear_accel_noise (float): absolute noise for linear acceleration (m/s²)
#             angular_accel_noise (float): absolute noise for angular acceleration (rad/s²)
#             linear_bias_drift (float): bias drift rate for linear acceleration (m/s² per second)
#             angular_bias_drift (float): bias drift rate for angular acceleration (rad/s² per second)
#         """
#         super().__init__(name, robot, interval)
#         self.LINEAR_ACCEL_NOISE = linear_accel_noise
#         self.ANGULAR_ACCEL_NOISE = angular_accel_noise
#         self.LINEAR_BIAS_DRIFT = linear_bias_drift
#         self.ANGULAR_BIAS_DRIFT = angular_bias_drift

#         # IMU bias (slowly drifting offset)
#         self.linear_bias = 0.0
#         self.angular_bias = 0.0

#         # Track previous velocities to compute acceleration
#         self.prev_lin_vel = 0.0
#         self.prev_ang_vel = 0.0
#         self.prev_time = 0.0

#     def sample(self) -> dict[str, float]:
#         """
#         Take a noisy IMU measurement of linear and angular acceleration.

#         Returns:
#             A dictionary containing noisy linear and angular acceleration estimates.
#         """
#         current_time = self.robot.env.time

#         # Compute true accelerations from velocity changes
#         if self.prev_time > 0:
#             dt = current_time - self.prev_time
#             if dt > 0:
#                 true_linear_accel = (self.robot.last_lin_vel - self.prev_lin_vel) / dt
#                 true_angular_accel = (self.robot.last_ang_vel - self.prev_ang_vel) / dt
#             else:
#                 true_linear_accel = 0.0
#                 true_angular_accel = 0.0
#         else:
#             true_linear_accel = 0.0
#             true_angular_accel = 0.0

#         # Update bias (random walk)
#         time_delta = current_time - self.prev_time if self.prev_time > 0 else 0.0
#         self.linear_bias += random.gauss(0, self.LINEAR_BIAS_DRIFT * time_delta)
#         self.angular_bias += random.gauss(0, self.ANGULAR_BIAS_DRIFT * time_delta)

#         # Add noise and bias
#         noisy_linear_accel = (
#             true_linear_accel
#             + self.linear_bias
#             + random.gauss(0, self.LINEAR_ACCEL_NOISE)
#         )
#         noisy_angular_accel = (
#             true_angular_accel
#             + self.angular_bias
#             + random.gauss(0, self.ANGULAR_ACCEL_NOISE)
#         )

#         # Update previous values for next sample
#         self.prev_lin_vel = self.robot.last_lin_vel
#         self.prev_ang_vel = self.robot.last_ang_vel
#         self.prev_time = current_time

#         return {
#             "linear_acceleration": noisy_linear_accel,
#             "angular_acceleration": noisy_angular_accel
#         }

# class VisualOdometry(SensorInterface):
#     """
#     Estimates motion from visual feature tracking.
#     """
#     def __init__(
#         self,
#         robot: Robot,
#         name: str = "VisualOdometry",
#         interval: float = 0.1,
#         translation_noise: float = 0.03,
#         rotation_noise: float = 0.08,
#         failure_rate: float = 0.05,  # 5% chance of tracking loss
#     ):
#         super().__init__(name, robot, interval)
#         self.TRANSLATION_NOISE = translation_noise
#         self.ROTATION_NOISE = rotation_noise
#         self.FAILURE_RATE = failure_rate
#         self.prev_pose = None

#     def sample(self):
#         """
#         Estimate motion since last measurement.
#         """
#         current_pose = self.robot.env.get_gt_robot_pose()

#         # First measurement - no motion estimate yet
#         if self.prev_pose is None:
#             self.prev_pose = current_pose
#             return None

#         # Simulate tracking failure
#         if random.random() < self.FAILURE_RATE:
#             self.prev_pose = current_pose
#             return None

#         # Compute true displacement
#         dx = current_pose.pos.x - self.prev_pose.pos.x
#         dy = current_pose.pos.y - self.prev_pose.pos.y
#         dtheta = current_pose.theta - self.prev_pose.theta
#         dtheta = (dtheta + math.pi) % (2 * math.pi) - math.pi

#         # Add noise
#         noisy_dx = random.gauss(dx, abs(dx) * self.TRANSLATION_NOISE + 0.01)
#         noisy_dy = random.gauss(dy, abs(dy) * self.TRANSLATION_NOISE + 0.01)
#         noisy_dtheta = random.gauss(dtheta, abs(dtheta) * self.ROTATION_NOISE + 0.01)

#         self.prev_pose = current_pose

#         return {
#             "dx": noisy_dx,
#             "dy": noisy_dy,
#             "dtheta": noisy_dtheta
#         }
