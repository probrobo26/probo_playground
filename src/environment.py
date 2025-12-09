"""
A simulation environment for a mobile robot operating in two dimensions.
"""

from utils import Position, Pose, BearingRange, Bounds, Landmark
import pandas as pd
import math


class Environment:
    """
    The Environment class models the world that the robots navigate in. The world is continuous and two-dimensional, with a border and internal obstacles and landmarks. The world also manages the passage of time and the motion of robotic agents within it.

    Attributes:
        DIMS (Bounds): the four corners of the environment
        DT (float): the size of one timestep in seconds
        time (float): the current time
        agent_pose (Pose): the starting position and heading of the agent
        OBSTACLES (list[Bounds]): a list of all intraversible areas
        LANDMARKS (list[Landmark]): a list of all identifiable landmarks
    """

    def __init__(
        self,
        dimensions: Bounds,
        agent_pose: Pose,
        obstacles: list[Bounds],
        landmarks: list[Landmark],
        timestep: float = 0.1,
    ):
        """
        Initialize the environment, ensuring that all settings are compatible with each other.

        Args:
            dimensions (Bounds): the four corners of the environment
            agent_pose (Pose): the starting position and heading of the agent
            obstacles (list[Bounds]): a list of all intraversible areas
            landmarks (list[Landmark]): a list of all identifiable landmarks
            timestep (float): the size of one timestep in seconds
        """
        # nab the dimensions
        self.DIMS = dimensions

        # note the timestep and time
        self.DT = timestep
        self.time = 0.0

        # validate agent pose
        assert dimensions.within_bounds(agent_pose.pos)
        self.agent_pose = agent_pose

        # validate all obstacle regions
        for obs in obstacles:
            assert dimensions.within_x(obs.x_min)
            assert dimensions.within_x(obs.x_max)
            assert dimensions.within_y(obs.y_min)
            assert dimensions.within_y(obs.y_max)
        self.OBSTACLES = obstacles

        # validate all landmark positions and uniqueness
        for l in landmarks:
            assert dimensions.within_bounds(l.pos)
        assert len(set(l.id for l in landmarks)) == len(landmarks)
        self.LANDMARKS = landmarks

    # --- Motion Execution ---

    def robot_step(self, dx, dy, dtheta):
        """
        Move the robot during a single timestep.

        Args:
            dx: change in x position
            dy: change in y position
            dtheta: change in heading
        """
        # use dx and dy to find new agent position after collisions
        updated_pos = self.validate_xy_motion(dx, dy)

        # new theta, free rotation is always allowed
        updated_theta = self.agent_pose.theta + dtheta
        updated_theta = (updated_theta + math.pi) % (2 * math.pi) - math.pi

        # step and update
        self.time = round(self.time + self.DT, 3)
        self.agent_pose = Pose(updated_pos, updated_theta)

    # --- Validation ---

    def validate_xy_motion(self, dx: float, dy: float) -> Position:
        """
        Given attempted x and y motion by the robot, determine what motion is physically possible and return the actual motion that will be executed.

        Args:
            dx: change in x position
            dy: change in y position

        Returns:
            the actual position of the robot once the motion is executed
        """
        # check if new x is legal
        x_new = self.agent_pose.pos.x
        if self.is_valid_pos(
            Position(self.agent_pose.pos.x + dx, self.agent_pose.pos.y)
        ):
            x_new = self.agent_pose.pos.x + dx

        # check if new y is legal
        y_new = self.agent_pose.pos.y
        if self.is_valid_pos(Position(x_new, self.agent_pose.pos.y + dy)):
            y_new = self.agent_pose.pos.y + dy

        return Position(x_new, y_new)

    def is_valid_pos(self, pos: Position) -> bool:
        """
        Check if a given position is valid; i.e. not out-of-bounds or over an obstacle region.

        Args:
            x (float): the x position
            y (float): the y position

        Returns:
            true if the position is valid and false otherwise
        """
        if self.DIMS.within_bounds(pos):
            result = True
            for obs in self.OBSTACLES:
                result = result and not obs.within_bounds(pos)
            return result
        return False

    # --- Ground Truth Sensing ---

    def get_gt_robot_pose(self) -> Pose:
        """
        Return the ground truth robot pose.
        """
        return self.agent_pose

    def get_gt_to_landmarks(self) -> pd.DataFrame:
        """
        Return ground truth range and bearing to all landmarks.
        """
        measurements = pd.DataFrame()
        for l in self.LANDMARKS:
            x_diff = l.pos.x - self.agent_pose.pos.x
            y_diff = l.pos.y - self.agent_pose.pos.y
            range = math.sqrt(x_diff**2 + y_diff**2)
            bearing = math.atan2(y_diff, x_diff) - self.agent_pose.theta
            bearing = (bearing + math.pi) % (2 * math.pi) - math.pi
            measurements[f"Landmark{l.id}"] = [BearingRange(bearing, range)]
        return measurements

    def get_landmark_by_id(self, id: int):
        """
        Retrieve a landmark object using its id number.
        """
        for l in self.LANDMARKS:
            if l.id == id:
                return l
        return None

    def get_landmarks_by_pos(self, pos: Position):
        """
        Retrieve any landmarks at a given position.
        """
        lms = []
        for l in self.LANDMARKS:
            if l.pos == pos:
                lms.append(l)
        return lms

    # --- Logging ---
    def info(self) -> dict:
        """
        Return a dictionary of frozen environment information.
        """
        return {
            "Dimensions": self.DIMS.to_dict(),
            "Obstacles": [obs.to_dict() for obs in self.OBSTACLES],
            "Landmarks": [l.to_dict() for l in self.LANDMARKS],
            "Timestep": self.DT,
        }

    def take_gt_snapshot(self):
        """
        Return sensor information about this timestep.
        """
        # regulars
        df1 = pd.DataFrame(
            {
                "Time": [self.time],
                "RobotPose": [self.agent_pose.to_string()],
            }
        )
        # landmark hell
        gt_to_lms = self.get_gt_to_landmarks()
        df2 = pd.DataFrame()
        for lm in gt_to_lms.columns:
            df2[lm] = [
                gt_to_lms[lm].values[0].to_string()
            ]  # flip BearingRange to string representation

        # combine and return
        return pd.merge(
            df1,
            df2,
            left_index=True,
            right_index=True,
        )
