import matplotlib.pyplot as plt
import matplotlib.patches as patches
import pandas as pd
import numpy as np
import re
import pickle


class Visualizer:
    """
    Visualizer for Kalman Filter trajectory estimation results.
    """

    def __init__(
        self,
        env_info,
        gt_log,
        sensor_log,
    ):
        """
        Initialize the visualizer class.
        """
        self.env_info = env_info
        self.gt_log = gt_log
        self.sensor_log = sensor_log

    def plot_env(self):
        """
        Plot the environment features with no trajectories.
        """
        # set up axis
        fig, ax = plt.subplots(figsize=(10, 10))
        ax.set_xlabel("X Position (m)")
        ax.set_ylabel("Y Position (m)")
        ax.set_title(f"Environment Map")

        # Set up the plot boundaries
        dims = self.env_info["Dimensions"]
        ax.set_xlim(dims["x_min"] - 1, dims["x_max"] + 1)
        ax.set_ylim(dims["y_min"] - 1, dims["y_max"] + 1)
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)

        # set up env boundaries
        width = dims["x_max"] - dims["x_min"]
        height = dims["y_max"] - dims["y_min"]
        walls = patches.Rectangle(
            (dims["x_min"], dims["y_min"]),
            width,
            height,
            linewidth=5,
            edgecolor="black",
            facecolor="none",
            alpha=1.0,
        )
        ax.add_patch(walls)

        # Plot obstacles
        for obs in self.env_info["Obstacles"]:
            width = obs["x_max"] - obs["x_min"]
            height = obs["y_max"] - obs["y_min"]
            rect = patches.Rectangle(
                (obs["x_min"], obs["y_min"]),
                width,
                height,
                linewidth=2,
                edgecolor="black",
                facecolor="gray",
                alpha=0.5,
                label="Obstacle" if obs == self.env_info["Obstacles"][0] else "",
            )
            ax.add_patch(rect)

        # Plot landmarks
        for lm in self.env_info["Landmarks"]:
            # Plot pinging range circle (10m radius)
            circle = patches.Circle(
                (lm["pos"]["x"], lm["pos"]["y"]),
                10,  # 10m radius
                linewidth=1,
                edgecolor="red",
                facecolor="red",
                alpha=0.1,
                label="Pinging Range" if lm == self.env_info["Landmarks"][0] else "",
            )
            ax.add_patch(circle)

            # plot floating point landmarks
            ax.plot(
                lm["pos"]["x"],
                lm["pos"]["y"],
                "r*",
                markersize=15,
                label="Landmark" if lm == self.env_info["Landmarks"][0] else "",
            )
            ax.annotate(
                f"LM{lm['id']}",
                (lm["pos"]["x"], lm["pos"]["y"]),
                xytext=(5, 5),
                textcoords="offset points",
                fontsize=10,
                color="red",
            )

        ax.legend(loc="upper right")
        return fig, ax

    def poses_from_odom(self):
        """
        Given a DataFrame of odometry data with the following columns:
        Time | Odometry_LinearVelocity | Odometry_AngularVelocity
        Output a DataFrame of estimated pose data with the following columns:
        Time | x | y | theta
        """
        poses = []
        x, y, theta = 0.0, 0.0, 0.0
        dt = self.env_info["Timestep"]

        for row in self.sensor_log.itertuples():
            v = row.Odometry_LinearVelocity
            w = row.Odometry_AngularVelocity

            # Dead reckoning integration
            x += np.cos(theta) * v * dt
            y += np.sin(theta) * v * dt
            theta += w * dt

            # Wrap theta to [-pi, pi]
            theta = theta % (2 * np.pi)
            if theta > np.pi:
                theta -= 2 * np.pi

            poses.append({"Time": row.Time, "x": x, "y": y, "theta": theta})

        return pd.DataFrame(poses)

    def poses_from_gt(self):
        """
        Given a DataFrame of ground truth data with the following columns:
        Time | RobotPose
        Where RobotPose data is in the form: X{xposition}Y{yposition}T{heading}
        And time data is in decaseconds.
        Output a DataFrame of ground truth pose data with the following columns:
        Time | x | y | theta
        Where time data is in seconds.
        """
        poses = []

        for row in self.gt_log.itertuples():
            # Parse the RobotPose string (format: X{x}Y{y}T{theta})
            pose_str = row.RobotPose
            match = re.match(r"X([-\d.]+)Y([-\d.]+)T([-\d.]+)", pose_str)

            if match:
                x = float(match.group(1))
                y = float(match.group(2))
                theta = float(match.group(3))

                poses.append({"Time": row.Time, "x": x, "y": y, "theta": theta})

        return pd.DataFrame(poses)

    def poses_from_gps(self):
        """
        Given a DataFrame of GPS data with the following columns:
        Time | GPS
        Where GPS data is a Position dataclass with fields x and y.
        Output a DataFrame of GPS pose data with the following columns:
        Time | x | y | theta
        Note: theta is set to NaN since GPS doesn't measure heading.
        """
        poses = []

        for row in self.sensor_log.itertuples():
            # Check if GPS data exists and is a Position object
            if hasattr(row, "GPS") and row.GPS is not None:
                try:
                    # Access Position dataclass fields
                    x = row.GPS.x
                    y = row.GPS.y

                    poses.append(
                        {
                            "Time": row.Time,
                            "x": x,
                            "y": y,
                            "theta": np.nan,  # GPS doesn't measure heading
                        }
                    )
                except (AttributeError, TypeError):
                    # Skip if GPS is not a valid Position object
                    continue

        return pd.DataFrame(poses)

    def plot_single_trajectory(
        self, label, pose_table: pd.DataFrame, color="blue", alpha=1.0
    ):
        """
        Given a DataFrame of robot pose data with the following columns:
        Time | x | y | theta
        Plot the trajectory as a quiver on an xy grid with theta as arrow angle.
        """
        # Get current axes or create new ones
        if plt.get_fignums():
            ax = plt.gca()
        else:
            fig, ax = self.plot_env()

        # Plot trajectory path
        ax.plot(
            pose_table["x"],
            pose_table["y"],
            "-",
            color=color,
            alpha=alpha,
            linewidth=2,
            label=label,
        )

        # Plot arrows showing heading at intervals
        # Show arrows every N points to avoid clutter
        skip = max(1, len(pose_table) // 20)

        for idx in range(0, len(pose_table), skip):
            row = pose_table.iloc[idx]
            dx = 0.5 * np.cos(row["theta"])
            dy = 0.5 * np.sin(row["theta"])

            ax.arrow(
                row["x"],
                row["y"],
                dx,
                dy,
                head_width=0.3,
                head_length=0.2,
                fc=color,
                ec=color,
                alpha=alpha * 0.7,
            )

        # Mark start and end positions
        start = pose_table.iloc[0]
        end = pose_table.iloc[-1]

        ax.plot(
            start["x"],
            start["y"],
            "o",
            color=color,
            markersize=10,
            alpha=alpha,
        )
        ax.plot(
            end["x"],
            end["y"],
            "s",
            color=color,
            markersize=10,
            alpha=alpha,
        )

        ax.legend(loc="upper right")
        return ax

    def draw_all(self):
        """
        Docstring for draw_all

        :param self: Description
        """
        self.plot_env()
        self.plot_single_trajectory(
            "Ground Truth",
            self.poses_from_gt(),
            "green",
        )
        self.plot_single_trajectory(
            "Dead Reckoning",
            self.poses_from_odom(),
            "red",
        )
        self.plot_single_trajectory(
            "GPS Only",
            self.poses_from_gps(),
            "orange",
        )
        plt.savefig("./logs/dataset_viz.png")
        plt.show()


if __name__ == "__main__":
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
