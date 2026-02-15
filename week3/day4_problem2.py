import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

imu = pd.read_csv("imu1.csv")
vicon = pd.read_csv("vi1.csv")

print("IMU columns:\n", imu.columns)
print("Vicon columns:\n", vicon.columns)

plt.figure()
plt.plot(imu["Time"], imu["attitude_roll_radians"], label="roll")
plt.plot(imu["Time"], imu["attitude_pitch_radians"], label="pitch")
plt.plot(imu["Time"], imu["attitude_yaw_radians"], label="yaw")
plt.title("IMU Attitude")
plt.xlabel("time")
plt.ylabel("radians")
plt.legend()
plt.show()

plt.figure()
plt.plot(vicon["Time"], vicon["roll"], label="roll")
plt.plot(vicon["Time"], vicon["pitch"], label="pitch")
plt.plot(vicon["Time"], vicon["yaw"], label="yaw")
plt.title("Vicon Attitude")
plt.xlabel("time")
plt.ylabel("radians")
plt.legend()
plt.show()

plt.figure()
plt.plot(imu["Time"], imu["magnetic_field_x_microteslas"], label="x")
plt.plot(imu["Time"], imu["magnetic_field_y_microteslas"], label="y")
plt.plot(imu["Time"], imu["magnetic_field_z_microteslas"], label="z")
plt.title("Magnetic Field")
plt.legend()
plt.show()

plt.figure()
plt.plot(imu["Time"], imu["gravity_x_G"], label="x")
plt.plot(imu["Time"], imu["gravity_y_G"], label="y")
plt.plot(imu["Time"], imu["gravity_z_G"], label="z")
plt.title("Gravity")
plt.legend()
plt.show()

for a in ["roll","pitch","yaw"]:
    plt.figure()
    plt.plot(imu["Time"], imu[f"attitude_{a}_radians"], label="IMU")
    plt.plot(vicon["Time"], vicon[a], label="Vicon")
    plt.title(a)
    plt.legend()
    plt.show()

def cov_corr(df, cols, name):
    X = df[cols].to_numpy().T
    print("\n", name)
    print("COV\n", np.cov(X))
    print("CORR\n", np.corrcoef(X))

cov_corr(
    imu,
    ["user_acc_x_G","user_acc_y_G","user_acc_z_G"],
    "Acceleration"
)

cov_corr(
    imu,
    ["attitude_roll_radians","attitude_pitch_radians","attitude_yaw_radians"],
    "Attitude"
)

cov_corr(
    imu,
    ["rotation_rate_x_rad_per_sec",
     "rotation_rate_y_rad_per_sec",
     "rotation_rate_z_rad_per_s"],
    "Rotation"
)

for axis in ["x","y","z"]:
    cols = [
        f"gravity_{axis}_G",
        f"user_acc_{axis}_G",
        f"magnetic_field_{axis}_microteslas"
    ]
    cov_corr(imu, cols, f"grav/acc/mag axis {axis}")

for axis in ["x","y","z"]:
    cols = [
        f"rotation_rate_{axis}_rad_per_sec" if axis!="z"
        else "rotation_rate_z_rad_per_s",
        f"user_acc_{axis}_G"
    ]
    cov_corr(imu, cols, f"gyro + acc axis {axis}")

merged = pd.merge_asof(
    imu.sort_values("Time"),
    vicon.sort_values("Time"),
    on="Time",
    suffixes=("_imu", "_vicon"),
    direction="nearest"
)

for a in ["roll","pitch","yaw"]:
    X = merged[[f"attitude_{a}_radians", a]].to_numpy().T
    print("\n", a)
    print("COV\n", np.cov(X))
    print("CORR\n", np.corrcoef(X))
