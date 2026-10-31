# Mobile Robotics: Kalman Filter Localization & PID Path Following

Python simulations of two core building blocks of a differential-drive mobile robot (designed for the **Thymio**):
a **Kalman filter** that estimates the robot's position and velocity from noisy wheel-speed data, and a
**PID controller** that steers the robot along a path of waypoints.

<!-- Add a picture here: run the scripts, save the plots in a media/ folder, then uncomment.
![Kalman filter estimate vs. true path](media/kalman_estimate.png)
![PID path following](media/pid_path.png)
-->

## What it does

- **State estimation:** a 4-state linear Kalman filter tracks position and velocity `[x, y, vx, vy]`
- **Sensor simulation:** generates noisy positions, speeds, orientations and motor speeds to test the filter
- **Path following:** a PID controller on the heading error drives the robot through an L-shaped series of waypoints
- **Visualisation:** matplotlib plots compare the true path with the estimated or followed trajectory

## How it works

### Kalman filter (`KalmanFilter.py`)

Constant-velocity motion model with time step `dt = 0.1 s`:

```
x_k = A · x_(k-1)          A = | 1  0  dt  0 |
                               | 0  1  0  dt |
                               | 0  0  1   0 |
                               | 0  0  0   1 |
```

Each cycle runs the standard predict / update steps:

1. **Predict:** `x = A·x`, `P = A·P·Aᵀ + Q`
2. **Measure:** the velocity components come from the left and right wheel speeds, converted with a calibration factor (`0.42`) and projected on the robot's heading
3. **Update:** innovation `i = y − H·x`, gain `K = P·Hᵀ·(H·P·Hᵀ + R)⁻¹`, then `x = x + K·i` and `P = P − K·H·P`

| Parameter | Value | Meaning |
| --- | --- | --- |
| `Q` | diag(0.04, 0.04, 6, 6) | Process noise (position, velocity) |
| `R` | diag(0.25, 0.25, 6, 6) | Measurement noise |
| `P₀` | 1000 · I | Initial uncertainty (robot position unknown) |

### PID path following (`MotionControl.py`)

- The heading error is the angle between the robot's orientation and the direction to the next waypoint: `error = atan2(dy, dx) − θ`
- A PID controller (`Kp = 7.5`, `Ki = 0.1`, `Kd = 0.01`) turns this error into a correction applied in opposite directions to the two wheels: `left = v − u`, `right = v + u`
- The robot switches to the next waypoint once it is within 0.2 units of the current one
- A unicycle model (`simulate_movement`) updates the robot's position and heading from the wheel speeds

## Run it

```bash
git clone https://github.com/antoinehinary/Mobile_Robotics.git
cd Mobile_Robotics
pip install -r requirements.txt

python KalmanFilter.py     # filter estimate vs. simulated path
python MotionControl.py    # PID path following on an L-shaped path
```

## Project structure

```
Mobile_Robotics/
├── KalmanFilter.py        # Kalman filter class + simulated sensor data + plot
├── MotionControl.py       # PID heading controller + path-following simulation
├── KalmanFilter_Robot/    # TODO: one line on what this folder contains
├── Git_filter/            # TODO: one line on what this folder contains
└── requirements.txt
```

## Tech stack

Python · NumPy · Matplotlib · Kalman filtering · PID control · differential-drive kinematics

## What I learned / next steps

- Tuning `Q` and `R` changes how much the filter trusts its model versus its sensors: a low `R` follows noisy measurements, a high one smooths but lags
- Next: fuse a position measurement (camera or odometry) so the position states are corrected too, not only the velocities
- Next: move from the linear filter to an **Extended Kalman Filter** with the heading in the state, matching the non-linear kinematics
- Next: run the controller on a real Thymio through `tdmclient`

## Credits

<!-- TODO: e.g. "Group project for [course name], [school], [year], with [teammates]." Remove if it was a solo project. -->
