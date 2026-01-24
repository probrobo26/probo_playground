"""
An abstract base class that all sensor classes must inherit from. This structure guarantees that all sensors have certain traits, including a name, sampling interval, and sampling function.

In addition to basic features, all sensors should have noise constants. Different sensors may use different distributions to model noise, and may take in different parameters to shape that noise. For example, one sensor might have a constant noise mean, while another might have noise that grows proportionally with distance or time.

Exteroceptive sensors measure the robot's relationship to the world. This includes GPS, cameras, LiDAR, and anything else that takes a measurement that can relate the robot's state to things beyond the robot.

Procioceptive sensors measure the robot's relationship to its past states. This includes IMUs, wheel encoders, and anything else that measures how the robot's state is relatively changing, without relating the robot to the world.
"""

from abc import ABC, abstractmethod


class SensorInterface(ABC):
    """
    A basic Interface to standardize all sensors.

    Attributes:
        name: string identifier
        robot: reference robot. required for observing the environment
        interval: period between measurements
    """

    def __init__(self, name: str, robot, interval: float):
        """
        Initialize a sensor class instace.

        Args:
            name (str): reference identifier
            robot (Robot): reference robot
            interval (float): period between measurements
        """
        self._name = name
        self.robot = robot
        self._interval = interval

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

    @abstractmethod
    def sample(self):
        """
        Sample the environment and return the noisy measurement(s).
        """
        pass
