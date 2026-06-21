Capstone Design: LIMO Mine Detection and OpenManipulator Collection System

ROS 2 Humble 기반의 LIMO 자율주행 차량, OpenMANIPULATOR-X 로봇팔, RGB-D 카메라, Arduino 금속 센서 및 송풍 모터를 통합한 캡스톤 디자인 프로젝트입니다.

본 프로젝트는 차량이 지정된 grid 영역을 탐색하다가 금속 물체를 감지하면 차량을 정지 및 후진시키고, 로봇팔과 Arduino 송풍 모터를 이용해 지뢰 모형 주변을 정리한 뒤 카메라로 목표 pose를 재검출하여 지뢰 모형을 집어 안전지대로 운반하는 흐름을 구현합니다.

⸻

Demo Video

LIMO Mine Detection and OpenManipulator Collection System

<a href="https://youtu.be/hDBHK8Z4lEc?si=tarsNxkFKu5722tS">
  <img src="https://img.youtube.com/vi/hDBHK8Z4lEc/hqdefault.jpg" width="900" alt="Capstone Design LIMO Mine Detection and OpenManipulator Collection Demo">
</a>

⸻

System Architecture

본 프로젝트는 차량 주행 시스템과 로봇팔 작업 시스템을 분리된 ROS 2 환경에서 통합하여 구성하였습니다.

Vehicle System (Host)

* Ubuntu 20.04
* ROS 2 Humble
* LIMO ROS 2 Packages
* Coverage Grid Navigation
* Mine Detection Logic
* Safe Zone Navigation
* Mission Management

Robot Arm System (Docker)

* Ubuntu 22.04
* ROS 2 Humble
* OpenMANIPULATOR-X
* MoveIt 2
* RGB-D Camera
* Grasp Planning
* Arduino Blower Control

External Devices

* OpenCR
* Arduino UNO
* Metal Sensors
* Blower Motor
* Orbbec Astra RGB-D Camera

Project Structure

.
├── robot_arm_ws/
│   └── src/
│       └── open_manipulator_vision_grasp/
│           ├── config/color_grasp.yaml
│           ├── launch/dabai_vision_grasp.launch.py
│           ├── scripts/arduino_metal_sensor.py
│           ├── scripts/detect_color_object.py
│           ├── scripts/detect_depth_object.py
│           ├── scripts/metal_grasp_coordinator.py
│           └── src/color_grasp_moveit.cpp
│
└── vehicle_ws/
    └── src/
        └── wego/
            ├── config/coverage_grid.yaml
            ├── launch/coverage_path_launch.py
            └── scripts/coverage_path_planner.py

⸻

Mission Flow

Normal Operation

Coverage Grid Exploration
        ↓
Metal Detection
        ↓
Vehicle Stop
        ↓
Vehicle Backup
        ↓
Robot Arm Pre-Motor Pose 1
        ↓
Robot Arm Pre-Motor Pose 2
        ↓
Arduino Blower Activation
        ↓
Robot Arm Ready Pose
        ↓
Target Pose Re-Detection
        ↓
MoveIt Grasp & Lift
        ↓
Additional Pick Verification
        ↓
Move to Safe Zone
        ↓
Yaw Alignment
        ↓
Drop Sequence
        ↓
Return to Last Exploration Point
        ↓
Resume Coverage Exploration

Target Not Detected Case

Blower Activation
        ↓
Ready Pose
        ↓
Target Not Detected
        ↓
Vehicle Backup (3 cm)
        ↓
Pre-Motor Pose Sequence
        ↓
Blower Re-Activation
        ↓
Target Re-Detection
        ↓
Grasp Retry

⸻

Main Features

* LIMO 기반 Coverage Grid 탐색
* Arduino 금속 센서를 이용한 지뢰 모형 감지
* 금속 감지 후 차량 정지 및 후진
* OpenMANIPULATOR-X Joint Pose 제어
* Arduino 송풍 모터 제어
* RGB-D 카메라 기반 빨간색 직육면체 지뢰 모형 검출
* /vision/target_pose 기반 MoveIt Grasp Sequence
* 목표 Pose 미검출 시 3cm 후진 후 재검출
* 수거한 지뢰 모형을 안전지대로 운반
* 안전지대 Yaw 정렬
* 4단계 Drop Pose Sequence 수행
* 마지막 탐색 지점 복귀 후 탐색 재개

⸻

Workspace Layout

실제 실행 환경에서는 아래와 같이 분리된 Workspace를 사용하였습니다.

Robot Arm / Camera / Arduino

/root/colcon_ws

Docker 내부 실행 Workspace

/home/agilex/colcon_ws

Host 백업 및 개발 Workspace

Vehicle Navigation

/home/agilex/wego_ws

Host Vehicle Workspace

⸻

Dependencies

본 저장소에는 직접 작성 및 수정한 핵심 패키지만 포함되어 있습니다.

아래 패키지들은 별도로 설치되어 있어야 합니다.

* ROS 2 Humble
* MoveIt 2
* ros2_control
* OpenMANIPULATOR-X ROS 2 Packages
* DynamixelSDK
* dynamixel_hardware_interface
* dynamixel_interfaces
* Orbbec Astra Camera Driver
* LIMO ROS 2 Packages
* Nav2
* ydlidar_ros2_driver
* robot_localization
* Arduino Serial Configuration

⸻

Build

Robot Arm Build (Docker)

cd /root/colcon_ws
source /opt/ros/humble/setup.bash
colcon build \
  --packages-select open_manipulator_vision_grasp \
  --symlink-install
source install/setup.bash

Robot Arm Build (Host)

cd ~/colcon_ws
source /opt/ros/humble/setup.bash
colcon build \
  --packages-select open_manipulator_vision_grasp \
  --symlink-install
source install/setup.bash

Vehicle Build

cd ~/wego_ws
source /opt/ros/humble/setup.bash
colcon build \
  --packages-select wego \
  --symlink-install
source install/setup.bash

⸻

Robot Arm Launch

포트 번호는 실행 시마다 달라질 수 있습니다.

먼저 확인합니다.

ls /dev/ttyACM* /dev/ttyUSB*

예시

OpenCR  : /dev/ttyACM0
Arduino : /dev/ttyACM1

Full Launch

ros2 launch open_manipulator_vision_grasp \
  dabai_vision_grasp.launch.py \
  port_name:=/dev/ttyACM0 \
  arduino_serial_port:=/dev/ttyACM1

Camera Separate Launch

ros2 launch open_manipulator_vision_grasp \
  dabai_vision_grasp.launch.py \
  start_camera:=false \
  port_name:=/dev/ttyACM0 \
  arduino_serial_port:=/dev/ttyACM1

⸻

Camera Launch

ros2 launch astra_camera \
  dabai_dc1.launch.xml \
  camera_name:=camera \
  enable_color:=true \
  enable_depth:=true \
  enable_ir:=false \
  enable_point_cloud:=false \
  enable_colored_point_cloud:=false \
  use_uvc_camera:=true \
  uvc_product_id:=0x0557

Camera Topics

ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic hz /vision/debug_image

⸻

Vehicle Coverage Launch

cd ~/wego_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wego coverage_path_launch.py

프로젝트 환경에 따라 아래 Launch가 별도로 필요할 수 있습니다.

* LIMO Bringup
* LiDAR Driver
* Localization
* Map Server
* RViz

⸻

Configuration

Robot Arm Parameters

config/color_grasp.yaml

motor_power_percent: 100
motor_run_time_ms: 3000
use_pre_motor_pose_sequence: true
target_after_return_timeout_sec: 100.0
extra_pick_attempts_after_lift: 1
extra_pick_target_timeout_sec: 2.0

Camera Calibration

dabai_vision_grasp.launch.py

camera_x
camera_y
camera_z
camera_roll
camera_pitch
camera_yaw

실제 grasp 위치가 맞지 않을 경우

* camera_x
* camera_y
* camera_z

를 우선 조정합니다.

Vehicle Mission Parameters

coverage_grid.yaml

metal_backup_distance: 0.09
metal_backup_speed: 0.03
arm_sequence_retry_on_target_timeout: true
arm_sequence_retry_count: 1
arm_sequence_retry_backup_distance: 0.03
return_to_safe_zone_on_mine: true
safe_zone_drop_yaw_rad: 0.0

⸻

Arduino Blower Command

BLOW,<power_percent>,<time_ms>

예시

ros2 topic pub --once /arduino/command \
std_msgs/msg/String \
"{data: 'BLOW,100,3000'}"

정상 동작 시

BLOW_COMMAND,POWER,100,TIME_MS,3000
MOTOR_STARTED,FORWARD,PWM,255,TIME_MS,3000
MOTOR_AUTO_STOP

⸻

Debug Commands

Arduino Events

ros2 topic echo /arduino/events

Arduino Commands

ros2 topic echo /arduino/command

Target Pose

ros2 topic echo --once /vision/target_pose

Manual Pick

ros2 service call \
/pick_latest_target \
std_srvs/srv/Trigger {}

Safe Zone Drop

ros2 service call \
/drop_mine_at_safe_zone \
std_srvs/srv/Trigger {}

Full Arm Sequence

ros2 service call \
/run_metal_grasp_sequence \
std_srvs/srv/Trigger {}

Controller Status

ros2 control list_controllers
ros2 topic hz /joint_states
ros2 topic echo --once /joint_states

⸻

Notes

* OpenCR과 Arduino 포트 번호는 자주 변경될 수 있습니다.
* Docker 내부에서도 USB Device Mapping이 정상적으로 되어 있어야 합니다.
* MoveIt Planning 실패는 작업공간 밖의 목표 Pose 또는 Joint Constraint 문제일 수 있습니다.
* build/, install/, log/ 디렉토리는 GitHub에 포함하지 않습니다.
* OpenManipulator Joint2 범위를 확장한 경우 URDF/Xacro 수정이 필요합니다.

⸻

Project Status

Completed

* Metal Detection 기반 차량 정지 및 후진
* OpenMANIPULATOR-X Pre-Motor Pose Sequence
* Arduino Blower Motor Control
* RGB-D 기반 Mine Pose Detection
* MoveIt Grasp & Lift
* Target Missing 시 3cm Backup 후 Retry
* Safe Zone Navigation
* Yaw Alignment
* Four-Step Drop Sequence
* Return to Last Coverage Position
* Resume Coverage Exploration
* Full Vehicle–Manipulator Integrated Mission

:::
이 버전이면 캡스톤 GitHub README로 바로 올려도 될 정도고, 교수님이나 기업 면접관이 봤을 때도 "무엇을 만들었는지 → 어떻게 동작하는지 → 어떻게 실행하는지" 흐름이 깔끔하게 보인다. 마지막에 실제 시스템 구성도 이미지(`docs/system_architecture.png`) 하나만 추가하면 완성도가 더 올라간다. think about it step-by-step.