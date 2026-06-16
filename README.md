Capstone Design: LIMO 지뢰 탐지 및 OpenManipulator 수거 시스템

ROS 2 Humble 기반의 LIMO 자율주행 차량, OpenMANIPULATOR-X 로봇팔, RGB-D 카메라, Arduino 금속 센서 및 송풍 모터를 통합한 캡스톤 디자인 프로젝트입니다.

본 프로젝트는 차량이 지정된 grid 영역을 탐색하다가 금속 물체를 감지하면 차량을 정지 및 후진시키고, 로봇팔과 Arduino 송풍 모터를 이용해 지뢰 모형 주변을 정리한 뒤 카메라로 목표 pose를 재검출하여 지뢰 모형을 집어 안전지대로 운반하는 흐름을 구현합니다.

## Demo Video

LIMO 지뢰 탐지 및 OpenManipulator 수거 시스템

<a href="https://youtu.be/AcADcXJuaxs?si=a4T9PJYDxr5ZYq_a">
  <img src="https://img.youtube.com/vi/AcADcXJuaxs/hqdefault.jpg" width="900" alt="Capstone Design LIMO Mine Detection and OpenManipulator Collection Demo">
</a>
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

주요 기능

* LIMO 차량 기반 coverage grid 탐색
* Arduino 금속 센서를 이용한 지뢰 모형 감지
* 금속 감지 후 차량 정지 및 후진
* OpenMANIPULATOR-X 로봇팔 joint pose 제어
* Arduino 송풍 모터 제어
* RGB-D 카메라 기반 빨간색 직육면체 지뢰 모형 검출
* /vision/target_pose 기반 MoveIt grasp sequence
* 목표 pose 미검출 시 차량 3cm 후진 후 송풍 및 재검출 재시도
* 수거한 지뢰 모형을 안전지대 grid 1로 운반
* 안전지대 yaw 정렬 후 로봇팔 drop pose sequence 수행
* 지뢰 투하 후 마지막 탐색 지점으로 복귀하여 탐색 재개

최종 통합 시퀀스

차량 주행
→ 금속 탐지
→ 차량 정지
→ 차량 후진
→ 로봇팔 pre-motor pose 1
→ 로봇팔 pre-motor pose 2
→ Arduino 송풍 모터 분사
→ 로봇팔 준비 pose 복귀
→ 카메라로 목표 pose 재검출
→ 물체 잡고 들어올림
→ 물체가 계속 검출되면 추가 pick 1회 수행
→ 차량이 안전지대 grid 1로 이동
→ 안전지대 도착 후 yaw 정렬
→ drop pose 1
→ drop pose 2
→ drop pose 3
→ drop pose 4
→ 그리퍼 열기
→ drop pose 3
→ drop pose 2
→ drop pose 1
→ 로봇팔 준비 pose 복귀
→ 그리퍼 닫기
→ 차량이 마지막 탐색 지점으로 이동
→ 탐색 재개

목표 pose가 검출되지 않는 경우:

송풍 모터 분사
→ 로봇팔 준비 pose 복귀
→ 목표 pose 미검출
→ 차량 3cm 후진
→ pre-motor pose sequence 재실행
→ 송풍 모터 재분사
→ 목표 pose 재검출
→ 물체 잡기

Workspace 구성

실제 실행 환경에서는 아래와 같이 별도 workspace로 사용했습니다.

로봇팔 / Arduino / 카메라:
/root/colcon_ws           # Docker 내부
/home/agilex/colcon_ws    # Host 백업 및 개발
차량 주행:
/home/agilex/wego_ws      # Host

Dependencies

이 저장소에는 직접 작성 및 수정한 핵심 패키지만 포함되어 있습니다.
아래 패키지들은 실행 환경에 별도로 설치되어 있어야 합니다.

* ROS 2 Humble
* MoveIt 2
* ros2_control
* OpenMANIPULATOR-X ROS 2 packages
* DynamixelSDK
* dynamixel_hardware_interface
* dynamixel_interfaces
* Orbbec / Astra camera driver
* LIMO ROS 2 packages
* Nav2
* ydlidar_ros2_driver
* robot_localization
* Arduino serial 권한 및 USB device mapping

Robot Arm Build

Docker 내부 로봇팔 workspace 기준:

cd /root/colcon_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select open_manipulator_vision_grasp --symlink-install
source install/setup.bash

Host에서 빌드할 경우:

cd ~/colcon_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select open_manipulator_vision_grasp --symlink-install
source install/setup.bash

Vehicle Build

cd ~/wego_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select wego --symlink-install
source install/setup.bash

Robot Arm 실행

OpenCR과 Arduino 포트는 실행할 때마다 바뀔 수 있으므로 먼저 확인합니다.

ls /dev/ttyACM* /dev/ttyUSB*

예시:

OpenCR  : /dev/ttyACM0 또는 /dev/ttyACM2
Arduino : /dev/ttyACM1 또는 /dev/ttyACM0

로봇팔 통합 런치:

cd /root/colcon_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch open_manipulator_vision_grasp dabai_vision_grasp.launch.py \
  port_name:=/dev/ttyACM0 \
  arduino_serial_port:=/dev/ttyACM1

카메라를 별도 터미널에서 실행하고 로봇팔 런치에서는 카메라를 끄는 경우:

ros2 launch open_manipulator_vision_grasp dabai_vision_grasp.launch.py \
  start_camera:=false \
  port_name:=/dev/ttyACM0 \
  arduino_serial_port:=/dev/ttyACM1

Camera 실행

cd /root/colcon_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch astra_camera dabai_dc1.launch.xml \
  camera_name:=camera \
  enable_color:=true \
  enable_depth:=true \
  enable_ir:=false \
  enable_point_cloud:=false \
  enable_colored_point_cloud:=false \
  use_uvc_camera:=true \
  uvc_product_id:=0x0557

카메라 topic 확인:

ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic hz /vision/debug_image

Vehicle Coverage 실행

차량 workspace에서 실행합니다.

cd ~/wego_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wego coverage_path_launch.py

프로젝트 환경에 따라 LIMO bringup, LiDAR, localization, map, RViz launch가 별도로 필요할 수 있습니다.

주요 설정 파일

로봇팔 / 카메라 / Arduino

robot_arm_ws/src/open_manipulator_vision_grasp/config/color_grasp.yaml

주요 파라미터:

motor_power_percent: 100
motor_run_time_ms: 3000
use_pre_motor_pose_sequence: true
target_after_return_timeout_sec: 100.0
extra_pick_attempts_after_lift: 1
extra_pick_target_timeout_sec: 2.0

카메라 TF 보정

robot_arm_ws/src/open_manipulator_vision_grasp/launch/dabai_vision_grasp.launch.py

주요 launch argument:

camera_x
camera_y
camera_z
camera_roll
camera_pitch
camera_yaw

이 값은 /vision/target_pose 자체에 영향을 줍니다.
실제 잡는 위치가 맞지 않을 경우 camera_x, camera_y, camera_z를 우선 조정합니다.

차량 coverage / 통합 시퀀스

vehicle_ws/src/wego/config/coverage_grid.yaml

주요 파라미터:

metal_backup_distance: 0.09
metal_backup_speed: 0.03
arm_sequence_retry_on_target_timeout: true
arm_sequence_retry_count: 1
arm_sequence_retry_backup_distance: 0.03
return_to_safe_zone_on_mine: true
safe_zone_drop_yaw_rad: 0.0

Arduino 송풍 모터 명령

ROS 2에서 Arduino로 다음 문자열을 발행합니다.

BLOW,<power_percent>,<time_ms>

예시:

ros2 topic pub --once /arduino/command std_msgs/msg/String "{data: 'BLOW,100,3000'}"

정상 동작 시 /arduino/events에 다음과 같은 메시지가 출력됩니다.

BLOW_COMMAND,POWER,100,TIME_MS,3000
MOTOR_STARTED,FORWARD,PWM,255,TIME_MS,3000
MOTOR_AUTO_STOP

디버깅 명령어

Arduino 이벤트 확인:

ros2 topic echo /arduino/events

Arduino 명령 확인:

ros2 topic echo /arduino/command

목표 pose 확인:

ros2 topic echo --once /vision/target_pose

수동 pick:

ros2 service call /pick_latest_target std_srvs/srv/Trigger {}

안전지대 drop sequence 수동 실행:

ros2 service call /drop_mine_at_safe_zone std_srvs/srv/Trigger {}

통합 arm sequence 수동 실행:

ros2 service call /run_metal_grasp_sequence std_srvs/srv/Trigger {}

컨트롤러 상태 확인:

ros2 control list_controllers
ros2 topic hz /joint_states
ros2 topic echo --once /joint_states

주의 사항

* OpenCR과 Arduino 포트 번호가 자주 바뀌므로 launch 전 반드시 /dev/ttyACM*를 확인해야 합니다.
* Docker 환경에서는 컨테이너 내부에서도 USB device가 보여야 합니다.
* MoveIt planning 실패는 target pose가 실제 로봇팔 작업공간 밖이거나 joint constraint가 강할 때 발생할 수 있습니다.
* build/, install/, log/, zip 백업 파일은 GitHub에 올리지 않습니다.
* OpenManipulator joint2 동작 범위를 넓혀 사용한 경우, OpenManipulator URDF/xacro의 joint2 limit 수정이 필요합니다.

Project Status

최종 구현 완료:

* 금속 탐지 기반 차량 정지 및 후진
* 로봇팔 pre-motor pose sequence
* Arduino 송풍 모터 제어
* 카메라 기반 빨간색 직육면체 지뢰 모형 pose 검출
* MoveIt 기반 grasp 및 lift
* 목표 미검출 시 3cm 후진 후 재시도
* 안전지대 이동 및 yaw 정렬
* 4단계 drop pose와 역순 복귀
* 마지막 탐색 지점 복귀 후 coverage 재개


