# Vehicle Speed Controller

This project implements a PID controller to maintain a vehicle speed of **60 km/h** using **ROS2**.

---

## Setup Instructions

1. Navigate to your workspace:
   ```bash
   cd ~/task_ws
   ```

2. Install required dependencies:
   ```bash
   pip install matplotlib
   ```

3. Build the package:
   ```bash
   colcon build --packages-select pre_interview
   source install/setup.bash
   ```

## How to Run

Use the launch file to run both the vehicle simulator and controller:

```bash
ros2 launch pre_interview sim.launch.py
```

**Note**: Do not run the controller alone with `ros2 run pre_interview controller` as it requires the vehicle simulator to be running.
## How to Run

Use the launch file (recommended) to run both nodes together:

```bash
ros2 launch pre_interview sim.launch.py
```

> **Note:** Do **not** run the controller alone with `ros2 run pre_interview controller` unless the vehicle simulator node is running.

---

## What it Does

1. **Vehicle Simulation**: starts from 0 km/h.
2. **PID Control**: reaches and maintains 60 km/h target.
3. **Data Collection**: records speed values for 60 seconds.
4. **Visualization**: generates a matplotlib plot comparing target vs actual speed.
5. **Output**: saves `speed_control_results.png` and prints performance metrics.

* Timing in the controller uses real system time.

