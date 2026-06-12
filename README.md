# Capstone Robot Arm

ROS 2 Humble package for an OpenMANIPULATOR-X arm with RGB-D target detection, Arduino metal sensing, and blower motor coordination.

## Main Sequence

```text
Metal detected
-> Move near detected target pose
-> Run Arduino blower motor
-> Return arm to prepared pose
-> Detect a fresh target pose
-> Pick and lift the object
```

## Package

```text
src/open_manipulator_vision_grasp
```

Important files:

- `launch/dabai_vision_grasp.launch.py`
- `config/color_grasp.yaml`
- `scripts/arduino_metal_sensor.py`
- `scripts/metal_grasp_coordinator.py`
- `scripts/detect_depth_object.py`
- `src/color_grasp_moveit.cpp`

## Build

```bash
cd ~/colcon_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select open_manipulator_vision_grasp
source install/setup.bash
```

## Launch

Adjust serial ports for your Docker/container environment.

```bash
ros2 launch open_manipulator_vision_grasp dabai_vision_grasp.launch.py \
  port_name:=/dev/ttyACM1 \
  arduino_serial_port:=/dev/ttyACM0
```

## Useful Debug Commands

```bash
ros2 topic echo /arduino/events
ros2 topic echo /arduino/command
ros2 topic echo /vision/target_pose
ros2 service call /pick_latest_target std_srvs/srv/Trigger {}
```

## Key Parameters

`config/color_grasp.yaml`

- `motor_power_percent`: Arduino blower power, `100` maps to PWM 255 in the Arduino sketch.
- `motor_run_time_ms`: blower runtime.
- `prepare_approach_height_m`: height above the detected object for the blower step.
- `approach_height_m`: pre-grasp approach height.
- `grasp_height_m`: grasp z offset.

`launch/dabai_vision_grasp.launch.py`

- `camera_x`, `camera_y`, `camera_z`: camera TF calibration values that affect `/vision/target_pose`.
- `port_name`: OpenCR/OpenManipulator serial port.
- `arduino_serial_port`: Arduino serial port.

## Notes

This repository contains the project package only. External dependencies such as OpenMANIPULATOR-X packages, DynamixelSDK, Dynamixel hardware interface packages, and Orbbec/Astra camera drivers should be installed or cloned separately in the ROS 2 workspace.
