import pytest
import math
from mobile_robot_sim.utils import Position, Pose, Bounds, Landmark, BearingRange
from mobile_robot_sim.environment import Environment

class TestEnvironmentInitialization:
    """
    Tests for Environment initialization.
    """

    @pytest.fixture
    def basic_env(self):
        """
        Fixture providing a basic environment.
        """
        return Environment(
            dimensions=Bounds(0, 10, 0, 10),
            agent_pose=Pose(Position(5, 5), 0),
            obstacles=[Bounds(2, 3, 2, 3)],
            landmarks=[Landmark(Position(7, 7), 0), Landmark(Position(3, 8), 1)],
            timestep=0.1,
        )

    def test_initialization_basic(self, basic_env: Environment):
        """
        Test the environment properly instantiates all attributes.
        """
        assert basic_env.DIMS == Bounds(0, 10, 0, 10)
        assert basic_env.DT == 0.1
        assert basic_env.time == 0.0
        assert basic_env.agent_pose == Pose(Position(5, 5), 0)
        assert len(basic_env.OBSTACLES) == 1
        assert len(basic_env.LANDMARKS) == 2

    def test_initialization_rejects_agent_outside_bounds(self):
        """
        Test initialization fails if agent starts outside environment.
        """
        with pytest.raises(AssertionError):
            Environment(
                dimensions=Bounds(0, 10, 0, 10),
                agent_pose=Pose(Position(15, 5), 0),  # Outside bounds
                obstacles=[],
                landmarks=[],
                timestep=0.1,
            )

    def test_initialization_rejects_obstacle_outside_bounds(self):
        """
        Test initialization fails if obstacle is outside environment.
        """
        with pytest.raises(AssertionError):
            Environment(
                dimensions=Bounds(0, 10, 0, 10),
                agent_pose=Pose(Position(5, 5), 0),
                obstacles=[Bounds(-1, 2, 2, 3)],  # Extends outside
                landmarks=[],
                timestep=0.1,
            )

    def test_initialization_rejects_landmark_outside_bounds(self):
        """
        Test initialization fails if landmark is outside environment.
        """
        with pytest.raises(AssertionError):
            Environment(
                dimensions=Bounds(0, 10, 0, 10),
                agent_pose=Pose(Position(5, 5), 0),
                obstacles=[],
                landmarks=[Landmark(Position(15, 15), 0)],  # Outside
                timestep=0.1,
            )

    def test_initialization_rejects_duplicate_landmark_ids(self):
        """
        Test initialization fails if landmarks have duplicate IDs.
        """
        with pytest.raises(AssertionError):
            Environment(
                dimensions=Bounds(0, 10, 0, 10),
                agent_pose=Pose(Position(5, 5), 0),
                obstacles=[],
                landmarks=[
                    Landmark(Position(2, 2), 0),
                    Landmark(Position(8, 8), 0),  # Duplicate ID
                ],
                timestep=0.1,
            )


class TestRobotStep:
    """
    Tests for moving robotic agents in the environment.
    """

    @pytest.fixture
    def env(self):
        """
        Fixture providing environment for robot_step tests.
        """
        return Environment(
            dimensions=Bounds(0, 10, 0, 10),
            agent_pose=Pose(Position(5, 5), 0),
            obstacles=[Bounds(2, 3, 2, 3)],
            landmarks=[Landmark(Position(7, 7), 0)],
            timestep=0.1,
        )

    def test_robot_step_updates_position(self, env: Environment):
        """
        Test robot_step updates agent position.
        """
        initial_x = env.agent_pose.pos.x
        initial_y = env.agent_pose.pos.y

        env.robot_step(dx=1.0, dy=0.5, dtheta=0)

        assert env.agent_pose.pos.x == initial_x + 1.0
        assert env.agent_pose.pos.y == initial_y + 0.5

    def test_robot_step_updates_heading(self, env):
        """
        Test robot_step updates agent heading.
        """
        initial_theta = env.agent_pose.theta
        dtheta = 0.5

        env.robot_step(dx=0, dy=0, dtheta=dtheta)

        assert env.agent_pose.theta == pytest.approx(initial_theta + dtheta)

    def test_robot_step_increments_time(self, env):
        """
        Test robot_step increments time by DT.
        """
        initial_time = env.time

        env.robot_step(dx=0, dy=0, dtheta=0)

        assert env.time == initial_time + env.DT

    def test_robot_step_wraps_heading(self, env):
        """
        Test robot_step wraps heading to [-π, π].
        """
        env.agent_pose = Pose(Position(5, 5), math.pi - 0.1)

        # Rotate past π
        env.robot_step(dx=0, dy=0, dtheta=0.5)

        # Should wrap to negative
        assert env.agent_pose.theta < -math.pi + 0.5
        assert env.agent_pose.theta >= -math.pi

    def test_robot_step_wraps_negative_heading(self, env):
        """
        Test robot_step wraps large negative heading.
        """
        env.agent_pose = Pose(Position(5, 5), -math.pi + 0.1)

        # Rotate past -π
        env.robot_step(dx=0, dy=0, dtheta=-0.5)

        # Should wrap to positive
        assert env.agent_pose.theta > math.pi - 0.5
        assert env.agent_pose.theta <= math.pi

    def test_robot_step_multiple_steps(self, env):
        """
        Test multiple robot steps accumulate correctly.
        """
        initial_time = env.time
        initial_x = env.agent_pose.pos.x

        steps = 10
        dx_per_step = 0.1

        for _ in range(steps):
            env.robot_step(dx=dx_per_step, dy=0, dtheta=0)

        assert env.time == pytest.approx(initial_time + steps * env.DT)
        assert env.agent_pose.pos.x == pytest.approx(initial_x + steps * dx_per_step)


class TestValidateXYMotion:
    """
    Tests for validating the robot's movement.
    """

    @pytest.fixture
    def env(self):
        """
        Fixture providing environment with obstacle.
        """
        return Environment(
            dimensions=Bounds(0, 10, 0, 10),
            agent_pose=Pose(Position(5, 5), 0),
            obstacles=[Bounds(2, 3, 2, 3)],
            landmarks=[],
            timestep=0.1,
        )

    def test_validate_xy_motion_accepts_valid_motion(self, env: Environment):
        """
        Test validate_xy_motion allows valid motion.
        """
        env.agent_pose = Pose(Position(5, 5), 0)

        new_pos = env.validate_xy_motion(dx=1.0, dy=1.0)

        assert new_pos.x == 6.0
        assert new_pos.y == 6.0

    def test_validate_xy_motion_blocks_x_into_obstacle(self, env: Environment):
        """
        Test validate_xy_motion blocks x motion into obstacle.
        """
        # Position agent just before obstacle
        env.agent_pose = Pose(Position(1.5, 2.5), 0)
        # Obstacle is at Bounds(2, 3, 2, 3)

        # Try to move into obstacle
        new_pos = env.validate_xy_motion(dx=1.0, dy=0)

        # X should not have moved
        assert new_pos.x == 1.5
        assert new_pos.y == 2.5

    def test_validate_xy_motion_blocks_y_into_obstacle(self, env: Environment):
        """
        Test validate_xy_motion blocks y motion into obstacle.
        """
        # Position agent just before obstacle
        env.agent_pose = Pose(Position(2.5, 1.5), 0)
        # Obstacle is at Bounds(2, 3, 2, 3)

        # Try to move into obstacle
        new_pos = env.validate_xy_motion(dx=0, dy=1.0)

        # Y should not have moved
        assert new_pos.x == 2.5
        assert new_pos.y == 1.5

    def test_validate_xy_motion_allows_sliding_x(self, env: Environment):
        """
        Test validate_xy_motion allows sliding in x when y is blocked.
        """
        # Position agent at (2.5, 1.5) - y is at obstacle boundary
        env.agent_pose = Pose(Position(2.5, 1.5), 0)
        # Obstacle at Bounds(2, 3, 2, 3)

        # Try to move diagonally - x is clear, y would go into obstacle
        new_pos = env.validate_xy_motion(dx=0.3, dy=0.5)

        # X should move (valid from 2.5 to 2.8, not in obstacle)
        assert new_pos.x == pytest.approx(2.8)
        # Y should not move (1.5 + 0.5 = 2.0, which is inside obstacle y-range [2,3])
        assert new_pos.y == 1.5

    def test_validate_xy_motion_allows_sliding_y(self, env: Environment):
        """
        Test validate_xy_motion allows sliding in y when x is blocked.
        """
        # Position agent at (1.5, 2.5) - inside obstacle's y-range
        env.agent_pose = Pose(Position(1.5, 2.5), 0)
        # Obstacle at Bounds(2, 3, 2, 3)

        # Try to move diagonally - x would go into obstacle, y might be valid
        new_pos = env.validate_xy_motion(dx=0.6, dy=-0.6)

        # X should not move (1.5 + 0.6 = 2.1, which is inside obstacle x-range [2,3])
        assert new_pos.x == 1.5
        # Y should move (2.5 - 0.6 = 1.9, which is outside obstacle y-range [2,3])
        assert new_pos.y == pytest.approx(1.9)

    def test_validate_xy_motion_blocks_at_boundary(self, env: Environment):
        """
        Test validate_xy_motion prevents leaving environment bounds.
        """
        env.agent_pose = Pose(Position(9.5, 5), 0)
        # Environment is Bounds(0, 10, 0, 10)

        # Try to move past boundary
        new_pos = env.validate_xy_motion(dx=1.0, dy=0)

        # Should be blocked at boundary
        assert new_pos.x == 9.5

    def test_validate_xy_motion_negative_motion(self, env: Environment):
        """
        Test validate_xy_motion works with negative deltas.
        """
        env.agent_pose = Pose(Position(5, 5), 0)

        new_pos = env.validate_xy_motion(dx=-1.0, dy=-0.5)

        assert new_pos.x == 4.0
        assert new_pos.y == 4.5

    def test_validate_xy_motion_zero_motion(self, env: Environment):
        """
        Test validate_xy_motion handles zero motion.
        """
        env.agent_pose = Pose(Position(5, 5), 0)

        new_pos = env.validate_xy_motion(dx=0, dy=0)

        assert new_pos.x == 5.0
        assert new_pos.y == 5.0


class TestIsValidPos:
    """
    Tests for validating arbitrary positions in the environment.
    """

    @pytest.fixture
    def env(self):
        """Fixture providing environment with obstacle"""
        return Environment(
            dimensions=Bounds(0, 10, 0, 10),
            agent_pose=Pose(Position(5, 5), 0),
            obstacles=[Bounds(2, 3, 2, 3)],
            landmarks=[],
            timestep=0.1,
        )

    def test_is_valid_pos_accepts_free_space(self, env: Environment):
        """
        Test is_valid_pos returns True for valid position.
        """
        # Position (5, 5) is in free space
        assert env.is_valid_pos(Position(5, 5)) is True

    def test_is_valid_pos_rejects_obstacle(self, env: Environment):
        """
        Test is_valid_pos returns False for position in obstacle.
        """
        # Obstacle at Bounds(2, 3, 2, 3)
        assert env.is_valid_pos(Position(2.5, 2.5)) is False

    def test_is_valid_pos_rejects_outside_bounds(self, env: Environment):
        """
        Test is_valid_pos returns False for out-of-bounds positions.
        """
        for pos in [
            Position(-1, 5),
            Position(11, 5),
            Position(5, -1),
            Position(5, 11),
        ]:
            assert env.is_valid_pos(pos) is False

    def test_is_valid_pos_on_boundary(self, env: Environment):
        """
        Test is_valid_pos accepts positions exactly on boundary.
        """
        for pos in [
            Position(0, 5),
            Position(10, 5),
            Position(5, 0),
            Position(5, 10),
        ]:
            assert env.is_valid_pos(pos) is True

    def test_is_valid_pos_on_obstacle_edge(self, env: Environment):
        """
        Test is_valid_pos rejects positions on obstacle edge.
        """
        # Obstacle at Bounds(2, 3, 2, 3)
        # Edge is considered part of obstacle
        for pos in [
            Position(2, 2.5),
            Position(3, 2.5),
            Position(2.5, 2),
            Position(2.5, 3),
        ]:
            assert env.is_valid_pos(pos) is False


class TestGetGtRobotPose:
    """
    Tests for accessing ground truth robot pose.
    """

    @pytest.fixture
    def env(self):
        """
        Fixture providing basic environment.
        """
        return Environment(
            dimensions=Bounds(0, 10, 0, 10),
            agent_pose=Pose(Position(5, 5), 0),
            obstacles=[],
            landmarks=[],
            timestep=0.1,
        )

    def test_get_gt_robot_pose_returns_current_pose(self, env: Environment):
        """
        Test get_gt_robot_pose returns current agent pose.
        """
        pose = env.get_gt_robot_pose()

        assert pose.pos.x == env.agent_pose.pos.x
        assert pose.pos.y == env.agent_pose.pos.y
        assert pose.theta == env.agent_pose.theta

    def test_get_gt_robot_pose_after_motion(self, env: Environment):
        """
        Test get_gt_robot_pose reflects motion.
        """
        env.robot_step(dx=1.0, dy=0.5, dtheta=0.3)

        pose = env.get_gt_robot_pose()

        assert pose.pos.x == 6.0
        assert pose.pos.y == 5.5
        assert pose.theta == pytest.approx(0.3)


class TestGetGtToLandmarks:
    """
    Tests for working with landmarks.
    """

    @pytest.fixture
    def env(self):
        """
        Fixture providing environment with landmarks.
        """
        return Environment(
            dimensions=Bounds(0, 10, 0, 10),
            agent_pose=Pose(Position(5, 5), 0),
            obstacles=[],
            landmarks=[Landmark(Position(7, 7), 0), Landmark(Position(3, 8), 1)],
            timestep=0.1,
        )

    def test_get_landmark_by_id_found(self, env: Environment):
        """
        Test that an existing landmark ID returns the correct landmark.
        """
        lm = env.get_landmark_by_id(0)
        assert lm is not None
        assert lm.id == 0
        assert lm.pos == Position(7, 7)

    def test_get_landmark_by_id_not_found(self, env: Environment):
        """
        Test that a non-existent landmark ID returns None.
        """
        lm = env.get_landmark_by_id(99)
        assert lm is None

    def test_get_landmark_by_pos_single_match(self, env: Environment):
        """
        Test that querying a position with a single landmark returns a list of length one.
        """
        lms = env.get_landmarks_by_pos(Position(7, 7))
        assert len(lms) == 1
        assert lms[0].id == 0

    def test_get_landmark_by_pos_no_match(self, env: Environment):
        """
        Test that querying a position with no associated landmark returns an empty list.
        """
        lms = env.get_landmarks_by_pos(Position(0, 0))
        assert lms == []

    def test_get_landmark_by_pos_correct_landmark_returned(self, env: Environment):
        """
        Test that the returned landmark at a queried position has the expected ID.
        """
        lms = env.get_landmarks_by_pos(Position(3, 8))
        assert len(lms) == 1
        assert lms[0].id == 1

    def test_get_gt_to_landmarks_returns_dict(self, env: Environment):
        """
        Test get_gt_to_landmarks returns dictionary.
        """
        measurements = env.get_gt_to_landmarks()
        assert isinstance(measurements, dict)

    def test_get_gt_to_landmarks_has_all_landmarks(self, env: Environment):
        """
        Test get_gt_to_landmarks includes all landmarks.
        """
        measurements = env.get_gt_to_landmarks()

        assert len(measurements) == len(env.LANDMARKS)
        for landmark in env.LANDMARKS:
            assert landmark in measurements

    def test_get_gt_to_landmarks_correct_range(self, env: Environment):
        """
        Test get_gt_to_landmarks computes correct range.
        """
        # Agent at (5, 5), landmark at (7, 7)
        env.agent_pose = Pose(Position(5, 5), 0)

        measurements = env.get_gt_to_landmarks()

        # Find the single landmark at (7, 7)
        landmark_77 = env.get_landmarks_by_pos(Position(7, 7))[0]

        expected_range = math.sqrt((7 - 5) ** 2 + (7 - 5) ** 2)
        assert measurements[landmark_77].range == pytest.approx(expected_range)

    def test_get_gt_to_landmarks_correct_bearing_facing_north(self):
        """
        Test get_gt_to_landmarks computes correct bearing when facing north.
        """
        # Agent at (5, 5) facing north (θ = π/2)
        # Landmark directly to the right (east) at (7, 5)
        env = Environment(
            dimensions=Bounds(0, 10, 0, 10),
            agent_pose=Pose(Position(5, 5), math.pi / 2),
            obstacles=[],
            landmarks=[Landmark(Position(7, 5), 0)],
            timestep=0.1,
        )

        measurements = env.get_gt_to_landmarks()
        landmark = env.LANDMARKS[0]

        # Landmark is to the right, so bearing should be -π/2 (90° right)
        assert measurements[landmark].bearing == pytest.approx(-math.pi / 2, abs=1e-5)

    def test_get_gt_to_landmarks_correct_bearing_facing_east(self):
        """
        Test get_gt_to_landmarks computes correct bearing when facing east.
        """
        # Agent at (5, 5) facing east (θ = 0)
        # Landmark ahead at (7, 5)
        env = Environment(
            dimensions=Bounds(0, 10, 0, 10),
            agent_pose=Pose(Position(5, 5), 0),
            obstacles=[],
            landmarks=[Landmark(Position(7, 5), 0)],
            timestep=0.1,
        )

        measurements = env.get_gt_to_landmarks()
        landmark = env.LANDMARKS[0]

        # Landmark is directly ahead, bearing should be 0
        assert measurements[landmark].bearing == pytest.approx(0, abs=1e-5)

    def test_get_gt_to_landmarks_bearing_behind(self):
        """
        Test get_gt_to_landmarks computes correct bearing for landmark behind.
        """
        # Agent at (5, 5) facing east (θ = 0)
        # Landmark behind at (3, 5)
        env = Environment(
            dimensions=Bounds(0, 10, 0, 10),
            agent_pose=Pose(Position(5, 5), 0),
            obstacles=[],
            landmarks=[Landmark(Position(3, 5), 0)],
            timestep=0.1,
        )

        measurements = env.get_gt_to_landmarks()
        landmark = env.LANDMARKS[0]

        # Landmark is directly behind, bearing should be ±π
        bearing_mag = abs(measurements[landmark].bearing)
        assert bearing_mag == pytest.approx(math.pi, abs=1e-5)

    def test_get_gt_to_landmarks_bearing_wrapped(self, env: Environment):
        """
        Test get_gt_to_landmarks wraps bearing to [-π, π].
        """
        measurements = env.get_gt_to_landmarks()

        for _, br in measurements.items():
            assert br.bearing >= -math.pi
            assert br.bearing <= math.pi

    def test_get_gt_to_landmarks_zero_range(self):
        """
        Test get_gt_to_landmarks handles robot on landmark.
        """
        # Agent exactly on landmark position
        env = Environment(
            dimensions=Bounds(0, 10, 0, 10),
            agent_pose=Pose(Position(5, 5), 0),
            obstacles=[],
            landmarks=[Landmark(Position(5, 5), 0)],
            timestep=0.1,
        )

        measurements = env.get_gt_to_landmarks()
        landmark = env.LANDMARKS[0]

        # Range should be zero
        assert measurements[landmark].range == pytest.approx(0, abs=1e-5)

    def test_get_gt_to_landmarks_empty(self):
        """
        Test get_gt_to_landmarks returns empty dict with no landmarks.
        """
        env = Environment(
            dimensions=Bounds(0, 10, 0, 10),
            agent_pose=Pose(Position(5, 5), 0),
            obstacles=[],
            landmarks=[],
            timestep=0.1,
        )

        measurements = env.get_gt_to_landmarks()

        assert len(measurements) == 0
        assert isinstance(measurements, dict)
