# Capstone Integrated Robot Workspace

This repository is a source snapshot of the integrated LIMO PRO coverage robot and OpenManipulator arm demo.

It contains two ROS 2 Humble workspaces:

- `wego_ws`: LIMO base, Nav2, mapping/navigation, coverage path planner, mine-search mission logic.
- `colcon_ws`: OpenManipulator, camera detection, metal sensor bridge, blower/grasp/drop integration.

Generated folders such as `build/`, `install/`, `log/`, and large zip archives are intentionally excluded. Rebuild both workspaces after cloning.

## Environment

Use these environment variables on every terminal:

```bash
export ROS_DOMAIN_ID=2
export ROS_LOCALHOST_ONLY=0
```

## Build `wego_ws`

```bash
cd ~/captone_robot_arm/wego_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Build `colcon_ws`

```bash
cd ~/captone_robot_arm/colcon_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

If you want to use the same absolute paths as the original robot computer, clone the repository under `~` and create symlinks:

```bash
cd ~
git clone https://github.com/YJS906/captone_robot_arm.git
ln -s ~/captone_robot_arm/wego_ws ~/wego_ws
ln -s ~/captone_robot_arm/colcon_ws ~/colcon_ws
```

## Typical Host Launch Order

```bash
cd ~/captone_robot_arm/wego_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=2
export ROS_LOCALHOST_ONLY=0

ros2 launch wego limo_bringup_launch.py
ros2 launch wego navigation_diff_launch.py map:=/home/agilex/wego_map.yaml
ros2 launch wego coverage_path_launch.py coverage_config_file:=$PWD/src/wego/config/coverage_grid.yaml
ros2 service call /start_coverage std_srvs/srv/Trigger
```

## Typical Arm Docker Launch

Inside the arm Docker container:

```bash
cd /root/colcon_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=2
export ROS_LOCALHOST_ONLY=0

ros2 launch open_manipulator_vision_grasp dabai_vision_grasp.launch.py \
  port_name:=/dev/ttyACM0 \
  arduino_serial_port:=/dev/ttyACM1
```

Current port convention:

- `/dev/ttyACM0`: OpenCR / OpenManipulator
- `/dev/ttyACM1`: Arduino metal sensor
- `/dev/ttyUSB0`: LiDAR
- `/dev/ttyTHS0`: LIMO base

## Main Documentation

Algorithm documentation is in:

```text
wego_ws/docs/navigation_and_coverage_algorithms.md
```
