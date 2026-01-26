"""
An abstract base class that all sensor classes must inherit from. This structure guarantees that all sensors have certain traits, including a name, sampling interval, and sampling function.

In addition to basic features, all sensors should have noise constants. Different sensors may use different distributions to model noise, and may take in different parameters to shape that noise. For example, one sensor might have a constant noise mean, while another might have noise that grows proportionally with distance or time.

Exteroceptive sensors measure the robot's relationship to the world. This includes GPS, cameras, LiDAR, and anything else that takes a measurement that can relate the robot's state to things beyond the robot.

Proprioceptive sensors measure the robot's relationship to its past states. This includes IMUs, wheel encoders, and anything else that measures how the robot's state is relatively changing, without relating the robot to the world.
"""

from abc import ABC, abstractmethod


class SensorInterface(ABC):
    """
    A basic Interface to standardize all sensors.

    Attributes:
        name: string identifier
        robot: reference robot. required for observing the environment
        interval: period between measurements
        last_meas_t: time of last sensor measurement
    """

    def __init__(self, name: str, robot, interval: float):
        """
        Initialize a sensor class instace.

        Args:
            name: reference identifier
            robot: reference robot
            interval: period between measurements
        """
        self._name = name
        self.robot = robot
        self._interval = interval
        self.last_meas_t = robot.env.time

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
    def sample(self):
        """
        Sample the environment and return the noisy measurement(s).
        """
        pass


class GPS(SensorInterface):
    """
    This class represents a GPS sensor that measures the position of the robot in 2D space.

    Attributes:
        name: string identifier
        robot: reference robot
        interval: period between measurements
        last_meas_t: time of last measurement
        GPS_NOISE: absolute noise for stdev
    """

    def __init__(
        self,
        robot,
        name,
        interval,
        x_noise,
        y_noise,
    ):
        """
        Initialize an instance of the GPS class.

        Args:
            name (str): reference identifier
            robot (Robot): reference robot
            interval (float): period between measurements
            x_noise (float): constant noise for x position stdev
            y_noise (float): constant noise for y position stdev
        """
        super().__init__(name, robot, interval)
        self.X_NOISE = x_noise
        self.Y_NOISE = y_noise

    def sample(self):
        """
        Sample the robot's x position and y position.
        """
        pass


class Odometry(SensorInterface):
    """
    This class represents an odometry sensor that measures the robot's velocity based on wheel encoders.
    Reports noisy estimates of linear and angular velocities.

    Attributes:
        name: string identifier
        robot: reference robot
        interval: period between measurements
        last_meas_t: time of last measurement
        LIN_NOISE: absolute noise for linear velocity stdev
        ANG_NOISE: absolute noise for angular velocity stdev
        LINEAR_NOISE_RATIO: proportional noise for linear velocity stdev
        ANGULAR_NOISE_RATIO: proportional noise for angular velocity stdev
    """

    def __init__(
        self,
        robot,
        name,
        interval,
        lin_noise,
        ang_noise,
        linear_noise_ratio,
        angular_noise_ratio,
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
        self.LIN_NOISE = lin_noise
        self.ANG_NOISE = ang_noise
        self.LINEAR_NOISE_RATIO = linear_noise_ratio
        self.ANGULAR_NOISE_RATIO = angular_noise_ratio

    def sample(self):
        """
        Sample the robot's linear and angular velocity.
        """
        pass
