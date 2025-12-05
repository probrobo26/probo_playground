import pytest
import math
from mobile_robot_sim.utils import Position, Pose, Bounds, Landmark
from mobile_robot_sim.environment import Environment
from mobile_robot_sim.robot import Robot

class TestRobotInitialization:
    """
    Tests for Robot initialization.
    """

    @pytest.fixture
    def env(self) -> Environment:
        """
        Fixture providing basic environment.
        """
        return Environment(
            dimensions=Bounds(0, 10, 0, 10),
            agent_pose=Pose(Position(5, 5), 0),
            obstacles=[],
            landmarks=[Landmark(Position(7, 7), 0)],
            timestep=0.1,
        )

    def test_initialization(self, env: Environment):
        """
        Robot initializes with correct default values.
        """
        robot = Robot(env)

        assert isinstance(robot.sensors, dict)
        assert len(robot.sensors) > 0
        assert robot.env is not None

    def test_has_required_sensors(self, env: Environment):
        """
        Robot initializes with GPS and LandmarkPinger.
        """
        robot = Robot(env)
        sensor_names = [s.name for s in robot.sensors.values()]

        assert "GPS" in sensor_names
        assert "LandmarkPinger" in sensor_names


class TestAgentStepDifferential:
    """
    Tests for differential drive math.
    """

    @pytest.fixture
    def robot(self) -> Robot:
        """
        Fixture providing robot in basic environment.
        """
        env = Environment(
            dimensions=Bounds(0, 10, 0, 10),
            agent_pose=Pose(Position(5, 5), 0),
            obstacles=[],
            landmarks=[Landmark(Position(7, 7), 0)],
            timestep=0.1,
        )
        return Robot(env)

    def test_differential_straight_line_motion(self, robot: Robot):
        """
        Robot moves forward in straight line with zero angular velocity.
        """
        initial_x = robot.env.agent_pose.pos.x
        initial_y = robot.env.agent_pose.pos.y
        initial_theta = robot.env.agent_pose.theta

        lin_vel = 1.0
        ang_vel = 0.0

        robot.agent_step_differential(lin_vel, ang_vel)

        dx = robot.env.agent_pose.pos.x - initial_x
        dy = robot.env.agent_pose.pos.y - initial_y
        distance_moved = math.sqrt(dx**2 + dy**2)

        expected_distance = lin_vel * robot.env.DT
        assert distance_moved == pytest.approx(expected_distance, abs=0.1)
        assert robot.env.agent_pose.theta == pytest.approx(initial_theta, abs=0.01)

    def test_differential_pure_rotation(self, robot: Robot):
        """
        Robot rotates in place with zero linear velocity.
        """
        initial_x = robot.env.agent_pose.pos.x
        initial_y = robot.env.agent_pose.pos.y
        initial_theta = robot.env.agent_pose.theta

        lin_vel = 0.0
        ang_vel = 1.0

        robot.agent_step_differential(lin_vel, ang_vel)

        dx = robot.env.agent_pose.pos.x - initial_x
        dy = robot.env.agent_pose.pos.y - initial_y
        distance_moved = math.sqrt(dx**2 + dy**2)
        assert distance_moved < 0.05

        expected_theta = initial_theta + ang_vel * robot.env.DT
        expected_theta = (expected_theta + math.pi) % (2 * math.pi) - math.pi
        assert robot.env.agent_pose.theta == pytest.approx(expected_theta, abs=0.1)

    def test_differential_arc_motion(self, robot: Robot):
        """
        Robot follows arc with both linear and angular velocity.
        """
        initial_pose = Pose(Position(5, 5), 0)
        robot.env.agent_pose = initial_pose

        lin_vel = 1.0
        ang_vel = 0.5

        robot.agent_step_differential(lin_vel, ang_vel)

        assert robot.env.agent_pose.pos.x != initial_pose.pos.x
        assert robot.env.agent_pose.pos.y != initial_pose.pos.y
        assert robot.env.agent_pose.theta != initial_pose.theta

    def test_differential_stores_velocities(self, robot: Robot):
        """
        Commanded velocities are stored for odometry.
        """
        lin_vel = 2.0
        ang_vel = 0.3

        robot.agent_step_differential(lin_vel, ang_vel)

        assert robot.cmd_lin_vel == pytest.approx(lin_vel, abs=0.2)
        assert robot.cmd_ang_vel == pytest.approx(ang_vel, abs=0.1)

    def test_differential_heading_wraps_correctly(self, robot: Robot):
        """
        Heading wraps to [-π, π] after large rotations.
        """
        robot.env.agent_pose = Pose(Position(5, 5), math.pi - 0.1)

        robot.agent_step_differential(0, 5.0)

        assert robot.env.agent_pose.theta >= -math.pi
        assert robot.env.agent_pose.theta <= math.pi

    def test_differential_respects_timestep(self):
        """
        Motion scales with environment timestep.
        """
        env_fast = Environment(
            dimensions=Bounds(0, 10, 0, 10),
            agent_pose=Pose(Position(5, 5), 0),
            obstacles=[],
            landmarks=[],
            timestep=0.1,
        )
        env_slow = Environment(
            dimensions=Bounds(0, 10, 0, 10),
            agent_pose=Pose(Position(5, 5), 0),
            obstacles=[],
            landmarks=[],
            timestep=0.01,
        )

        robot_fast = Robot(env_fast)
        robot_slow = Robot(env_slow)

        robot_fast.agent_step_differential(1.0, 0)
        robot_slow.agent_step_differential(1.0, 0)

        dist_fast = math.sqrt(
            (env_fast.agent_pose.pos.x - 5) ** 2 + (env_fast.agent_pose.pos.y - 5) ** 2
        )
        dist_slow = math.sqrt(
            (env_slow.agent_pose.pos.x - 5) ** 2 + (env_slow.agent_pose.pos.y - 5) ** 2
        )

        assert dist_fast > dist_slow

    def test_differential_backward_motion(self, robot: Robot):
        """
        Robot can move backward with negative linear velocity.
        """
        initial_x = robot.env.agent_pose.pos.x

        robot.agent_step_differential(-1.0, 0)

        assert robot.env.agent_pose.pos.x < initial_x

    def test_differential_negative_rotation(self, robot: Robot):
        """
        Robot can rotate clockwise with negative angular velocity.
        """
        initial_theta = robot.env.agent_pose.theta

        robot.agent_step_differential(0, -1.0)

        expected_theta = initial_theta - 1.0 * robot.env.DT
        expected_theta = (expected_theta + math.pi) % (2 * math.pi) - math.pi

        assert robot.env.agent_pose.theta == pytest.approx(expected_theta, abs=0.1)

    def test_differential_multiple_steps_accumulate(self, robot: Robot):
        """
        Multiple steps accumulate motion over time.
        """
        initial_x = robot.env.agent_pose.pos.x

        for _ in range(10):
            robot.agent_step_differential(1.0, 0)

        total_distance = robot.env.agent_pose.pos.x - initial_x
        assert total_distance > 0.5


# class TestAgentStepTranslational:
#     """Tests for Robot.agent_step_translational method"""

#     @pytest.fixture
#     def robot(self) -> Robot:
#         """Fixture providing robot in basic environment"""
#         env = Environment(
#             dimensions=Bounds(0, 10, 0, 10),
#             agent_pose=Pose(Position(5, 5), 0),
#             obstacles=[],
#             landmarks=[],
#             timestep=0.1,
#         )
#         return Robot(env)

#     def test_translational_x_motion(self, robot: Robot) -> None:
#         """Robot moves in x direction with translational drive."""
#         initial_x = robot.env.agent_pose.pos.x
#         initial_y = robot.env.agent_pose.pos.y

#         x_vel = 2.0
#         y_vel = 0.0

#         robot.agent_step_translational(x_vel, y_vel)

#         expected_dx = x_vel * robot.env.DT
#         actual_dx = robot.env.agent_pose.pos.x - initial_x
#         assert actual_dx == pytest.approx(expected_dx, abs=0.01)
#         assert robot.env.agent_pose.pos.y == pytest.approx(initial_y, abs=0.01)

#     def test_translational_y_motion(self, robot: Robot) -> None:
#         """Robot moves in y direction with translational drive."""
#         initial_x = robot.env.agent_pose.pos.x
#         initial_y = robot.env.agent_pose.pos.y

#         x_vel = 0.0
#         y_vel = 1.5

#         robot.agent_step_translational(x_vel, y_vel)

#         expected_dy = y_vel * robot.env.DT
#         actual_dy = robot.env.agent_pose.pos.y - initial_y
#         assert actual_dy == pytest.approx(expected_dy, abs=0.01)
#         assert robot.env.agent_pose.pos.x == pytest.approx(initial_x, abs=0.01)

#     def test_translational_diagonal_motion(self, robot: Robot) -> None:
#         """Robot moves diagonally with both x and y velocity."""
#         initial_pos = Position(5, 5)
#         robot.env.agent_pose = Pose(initial_pos, 0)

#         x_vel = 1.0
#         y_vel = 1.0

#         robot.agent_step_translational(x_vel, y_vel)

#         assert robot.env.agent_pose.pos.x != initial_pos.x
#         assert robot.env.agent_pose.pos.y != initial_pos.y

#         dx = robot.env.agent_pose.pos.x - initial_pos.x
#         dy = robot.env.agent_pose.pos.y - initial_pos.y
#         distance = math.sqrt(dx**2 + dy**2)
#         expected = math.sqrt((x_vel * robot.env.DT) ** 2 + (y_vel * robot.env.DT) ** 2)
#         assert distance == pytest.approx(expected, abs=0.01)

#     def test_translational_no_rotation(self, robot: Robot) -> None:
#         """Translational drive doesn't change heading."""
#         initial_theta = robot.env.agent_pose.theta

#         robot.agent_step_translational(1.0, 1.0)

#         assert robot.env.agent_pose.theta == initial_theta

#     def test_translational_negative_velocities(self, robot: Robot) -> None:
#         """Robot can move with negative velocities."""
#         initial_x = robot.env.agent_pose.pos.x
#         initial_y = robot.env.agent_pose.pos.y

#         robot.agent_step_translational(-1.0, -0.5)

#         assert robot.env.agent_pose.pos.x < initial_x
#         assert robot.env.agent_pose.pos.y < initial_y

#     def test_translational_zero_motion(self, robot: Robot) -> None:
#         """Zero velocity results in no motion."""
#         initial_pos = robot.env.agent_pose.pos

#         robot.agent_step_translational(0, 0)

#         assert robot.env.agent_pose.pos.x == initial_pos.x
#         assert robot.env.agent_pose.pos.y == initial_pos.y


class TestTakeSensorMeasurements:
    """
    Tests for taking sensor data.
    """

    @pytest.fixture
    def robot(self) -> Robot:
        """
        Fixture providing robot with sensors.
        """
        env = Environment(
            dimensions=Bounds(0, 10, 0, 10),
            agent_pose=Pose(Position(5, 5), 0),
            obstacles=[],
            landmarks=[Landmark(Position(7, 7), 0)],
            timestep=0.1,
        )
        return Robot(env)

    def test_sensor_measurements_has_all_sensors(self, robot: Robot):
        """
        Measurements dict contains all sensor names.
        """
        measurements = robot.take_sensor_measurements()

        for sensor in robot.sensors.values():
            assert sensor.name in measurements

    def test_sensor_measurements_respects_interval(self, robot: Robot):
        """
        Sensors only sample at their specified intervals.
        """
        robot.env.time = robot.sensors["GPS"].interval * 1

        measurements = robot.take_sensor_measurements()
        assert measurements["GPS"] is not None

        robot.env.time = robot.sensors["GPS"].interval * 1.5
        measurements = robot.take_sensor_measurements()
        assert measurements["GPS"] is None

        robot.env.time = robot.sensors["GPS"].interval * 2
        measurements = robot.take_sensor_measurements()
        assert measurements["GPS"] is not None

    def test_sensor_measurements_different_intervals(self, robot: Robot):
        """
        Sensors with different intervals sample independently.
        """
        robot.env.time = 0.0

        gps_samples = 0
        lp_samples = 0

        steps = 20

        for i in range(steps):
            robot.env.time = i * robot.env.DT  # two total seconds
            measurements = robot.take_sensor_measurements()

            if measurements["GPS"] is not None:
                gps_samples += 1
            if measurements["LandmarkPinger"] is not None:
                lp_samples += 1

        assert (
            lp_samples
            == steps * robot.env.DT / robot.sensors["LandmarkPinger"].interval
        )
        assert gps_samples == steps * robot.env.DT / robot.sensors["GPS"].interval

    def test_sensor_measurements_multiple_calls(self, robot: Robot):
        """
        Can call sensor measurements multiple times.
        """
        robot.env.time = 0.0

        m1 = robot.take_sensor_measurements()
        m2 = robot.take_sensor_measurements()

        assert isinstance(m1, dict)
        assert isinstance(m2, dict)
        assert len(m1) == len(m2)
