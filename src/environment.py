"""
A simulation environment for a mobile robot operating in two dimensions.

The Environment class models the world that the robots navigate in. The world is continuous and two-dimensional. The world possesses an outer border, internal obstacles, and identifiable landmarks. The world also manages the passage of time and the motion of robotic agents within the world over time.

Critically, the environment tracks the robot's state. In this case, the robot's state is a vector that includes three state variables: x position, y position, and heading.
"""


class Environment:
    """
    A class that models the world simulation environment and the robot's state.

    Attributes:
        width: the horizontal size of the world
        height: the vertical size of the world
        dt: the length of each timestep, in seconds
        obstacles: a list of obstacles
        landmarks: a list of landmarks
        robot_pose: the position and heading of the robot in the world
    """

    def __init__(
        self,
        width: int,
        height: int,
        dt: float,
        obstacles: list,
        landmarks: list,
        robot_starting_pose: list,
    ):
        """
        Initialize an instance of the Environment class.

        Args:
            width: the horizontal size of the world
            height: the vertical size of the world
            dt: the length of each timestep, in seconds
            obstacles: a list of obstacles
            landmarks: a list of landmarks
            robot_starting_pose: the initial position and heading of the robot
        """
        pass

    def robot_step(self, dx: float, dy: float, dtheta: float):
        """
        Update the robot's position and heading in the world. The robot should not be able to pass through obstacles or outside of the world bounds.

        Args:
            dx: change in x position
            dy: change in y position
            dtheta: change in heading
        """
        pass

    def is_valid_motion(self, dx: float, dy: float):
        """
        Given attempted x and y motion by the robot, determine what motion is physically possible (i.e. doesn't go through any obstacles or barriers). Return the actual motion that will be executed.

        Args:
            dx: attempted change in x position
            dy: attempted change in y position

        Returns:
            dx: change in x position that should be executed
            dy: change in y position that should be executed
        """
        pass

    def is_valid_position(self, x: float, y: float):
        """
        Check if a given robot position is valid; i.e. not out-of-bounds or within an obstacle. Return a boolean representing whether or not this condition is true.

        Args:
            x (float): the given x position
            y (float): the given y position

        Returns:
            true if the position is valid and false otherwise
        """
        pass

    def get_robot_pose(self):
        """
        Return the true robot pose.
        """
        pass

    def get_proximity_to_landmarks(self):
        """
        Return the robot's true range and bearing to all landmarks.
        """
        pass

    def take_state_snapshot(self):
        """
        Return true state information about this timestep, including robot position and the robot's bearing/range to landmarks, in a table format.
        """
        pass
