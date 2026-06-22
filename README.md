# LIMO PRO Mine Detection and OpenManipulator Collection System

ROS 2 Humble 기반의 **LIMO PRO 자율주행 차량**, **OpenMANIPULATOR-X 로봇팔**, **RGB-D 카메라**, **Arduino 금속 센서**, **송풍 모터**를 통합한 캡스톤 디자인 프로젝트입니다.

본 프로젝트는 차량이 지정된 탐색 영역을 grid 단위로 주행하면서 금속 물체를 감지하고, 감지 지점에서 차량을 정지 및 후진시킨 뒤 로봇팔과 송풍 모터를 이용하여 지뢰 모형 주변을 정리하고, RGB-D 카메라로 목표 pose를 재검출하여 지뢰 모형을 집어 안전지대로 운반하는 전체 미션을 구현한 프로젝트입니다.

단순히 로봇팔 또는 차량을 개별적으로 동작시키는 것이 아니라, **차량 주행, 금속 감지, 카메라 pose 추정, 로봇팔 grasp, 송풍 모터 제어, 안전지대 복귀, drop sequence, 탐색 재개**까지 하나의 미션 흐름으로 연결하는 데 중점을 두었습니다.

---

## Demo Video

아래 이미지를 클릭하면 전체 통합 데모 영상을 확인할 수 있습니다.

<a href="https://youtu.be/hDBHK8Z4lEc?si=tarsNxkFKu5722tS">
  <img src="https://img.youtube.com/vi/hDBHK8Z4lEc/hqdefault.jpg" width="900" alt="LIMO Mine Detection and OpenManipulator Collection Demo">
</a>

---

## Project Summary

본 프로젝트의 목표는 제한된 실내 환경에서 LIMO PRO 차량이 지정된 영역을 자율적으로 탐색하고, 금속 센서를 통해 지뢰 모형을 감지한 뒤 로봇팔로 해당 물체를 수거하여 안전지대로 옮기는 것입니다.

전체 시스템은 크게 두 부분으로 나뉩니다.

| Part               | Role                                                               |
| ------------------ | ------------------------------------------------------------------ |
| Vehicle System     | LIMO PRO 기반 grid 탐색, 금속 감지 이벤트 처리, 안전지대 복귀 및 탐색 재개                 |
| Manipulator System | RGB-D 카메라 기반 목표 pose 검출, OpenMANIPULATOR-X grasp, Arduino 송풍 모터 제어 |

차량은 coverage grid 기반으로 지정 영역을 탐색하고, 금속 감지 이벤트가 발생하면 현재 위치를 기준으로 미션 상태를 전환합니다. 이후 로봇팔 시스템이 송풍, 재검출, grasp 과정을 수행하고, 차량은 지뢰 모형을 안전지대로 운반한 뒤 drop sequence를 실행합니다.

---

## System Architecture

본 프로젝트는 Host와 Docker 환경을 분리하여 구성하였습니다.

```text
Host PC / LIMO PRO
├── Ubuntu 20.04
├── ROS 2 Humble
├── LIMO base control
├── LiDAR / map / localization
├── Nav2 infrastructure
├── Coverage grid planner
└── Mine handling mission logic

Docker / Manipulator Workspace
├── Ubuntu 22.04
├── ROS 2 Humble
├── OpenMANIPULATOR-X
├── MoveIt 2
├── RGB-D camera
├── Vision target detection
├── Arduino metal sensor bridge
├── Blower motor control
└── Grasp / lift / drop sequence
```

두 환경은 동일한 ROS 2 network domain에서 topic과 service를 통해 연결됩니다.

```bash
export ROS_DOMAIN_ID=2
export ROS_LOCALHOST_ONLY=0
```

---

## Hardware

| Component                   | Description                    |
| --------------------------- | ------------------------------ |
| LIMO PRO                    | Differential-drive 기반 자율주행 플랫폼 |
| OpenMANIPULATOR-X           | 지뢰 모형 수거를 위한 4-DOF 로봇팔         |
| OpenCR                      | OpenMANIPULATOR-X 제어 보드        |
| Orbbec / Astra RGB-D Camera | 목표 물체의 color/depth 기반 pose 추정  |
| Arduino UNO                 | 금속 센서 데이터 수신 및 송풍 모터 제어        |
| Metal Sensor                | 금속 물체 감지                       |
| Blower Motor                | 지뢰 모형 주변 이물질 제거를 가정한 송풍 장치     |
| LiDAR                       | 지도 기반 위치추정 및 장애물 감지            |

---

## Software Stack

| Category         | Stack                                     |
| ---------------- | ----------------------------------------- |
| OS               | Ubuntu 20.04, Ubuntu 22.04                |
| Middleware       | ROS 2 Humble                              |
| Mobile Robot     | LIMO ROS 2 packages                       |
| Navigation       | Nav2, AMCL, map server, TF                |
| Manipulator      | OpenMANIPULATOR-X, MoveIt 2, ros2_control |
| Vision           | RGB-D camera, OpenCV, cv_bridge           |
| Sensor Interface | Arduino serial bridge                     |
| Mission Logic    | ROS 2 topic/service 기반 state machine      |

---

## Repository Layout

```text
capstone_design/
├── README.md
├── colcon_ws/
│   ├── README.md
│   ├── setup_orbbec_local_deps.bash
│   └── src/
│       └── open_manipulator_vision_grasp/
│           ├── config/
│           │   └── color_grasp.yaml
│           ├── launch/
│           │   └── dabai_vision_grasp.launch.py
│           ├── scripts/
│           │   ├── arduino_metal_sensor.py
│           │   ├── detect_color_object.py
│           │   ├── detect_depth_object.py
│           │   └── metal_grasp_coordinator.py
│           └── src/
│               └── color_grasp_moveit.cpp
│
└── wego_ws/
    ├── docs/
    │   └── navigation_and_coverage_algorithms.md
    └── src/
        ├── wego/
        │   ├── config/
        │   │   └── coverage_grid.yaml
        │   ├── launch/
        │   │   ├── limo_bringup_launch.py
        │   │   ├── navigation_diff_launch.py
        │   │   └── coverage_path_launch.py
        │   └── scripts/
        │       └── coverage_path_planner.py
        └── wego_2d_nav/
            └── params/
                └── diff_navigation_params.yaml
```

`build/`, `install/`, `log/`와 같은 생성 디렉토리는 저장소에 포함하지 않습니다.
clone 후 각 workspace에서 다시 build하여 사용하는 구조입니다.

---

## Mission Flow

전체 미션 흐름은 다음과 같습니다.

```text
Coverage Grid Exploration
        ↓
Metal Detection
        ↓
Vehicle Stop
        ↓
Vehicle Backup
        ↓
Robot Arm Pre-Motor Pose Sequence
        ↓
Blower Motor Activation
        ↓
Robot Arm Return to Ready Pose
        ↓
RGB-D Camera Target Re-Detection
        ↓
MoveIt Grasp and Lift
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
```

본 프로젝트에서 중요한 점은 금속 감지 이후 차량과 로봇팔이 독립적으로 동작하는 것이 아니라, ROS 2 service와 topic을 통해 하나의 상태 흐름으로 연결된다는 점입니다.

---

## Vehicle System

차량 시스템은 `wego_ws`에서 관리됩니다.

주요 역할은 다음과 같습니다.

* LIMO PRO bringup
* LiDAR 및 map 기반 localization
* coverage grid 생성
* ㄷ자 형태의 coverage path 생성
* pure-pursuit 방식의 path following
* 금속 감지 이벤트 처리
* 감지 지점 후진
* 로봇팔 pickup sequence 요청
* 안전지대 복귀
* drop service 호출
* 기존 탐색 지점으로 복귀 후 coverage 재개

---

## Coverage Grid Planning

탐색 영역은 map frame 기준의 네 꼭짓점으로 정의됩니다.

```yaml
corner_mode: param
corner_points: '-0.247,-1.621,0.972,-1.471,0.739,0.325,-0.502,0.197'
coverage_cell_size: 0.3
coverage_boundary_margin: 0.15
require_free_cell_center: true
```

grid 생성 과정은 다음과 같습니다.

```text
1. map frame 기준 네 점을 읽음
2. 네 점의 중심을 기준으로 정렬
3. 가장 긴 edge를 grid 진행 방향으로 설정
4. coverage boundary margin 적용
5. coverage cell size 기준으로 grid 분할
6. map에서 free cell 여부 확인
7. 각 cell을 UNVISITED / VISITED / BLOCKED 상태로 관리
```

탐색 경로는 ㄷ자 형태의 boustrophedon pattern을 사용합니다.

```text
row 0: 0  → 1  → 2  → 3
row 1: 7  ← 6  ← 5  ← 4
row 2: 8  → 9  → 10 → 11
```

이 방식은 한 줄씩 왕복하면서 전체 영역을 커버할 수 있어, 불필요한 회전과 이동을 줄이는 데 유리합니다.

---

## Path Following

현재 차량 주행은 coverage point를 Nav2 goal로 하나씩 보내는 방식이 아니라, `coverage_path_planner.py`에서 직접 `/cmd_vel`을 publish하는 방식으로 구성되어 있습니다.

```yaml
execution_mode: path_follower
cmd_vel_topic: /cmd_vel
linear_speed: 0.08
lookahead_distance: 0.15
max_angular_speed: 0.25
```

path following은 pure-pursuit 방식에 가깝게 동작합니다.

```text
1. TF를 통해 현재 base_link pose 확인
2. 현재 목표 grid cell 선택
3. lookahead point 계산
4. lookahead point를 robot frame으로 변환
5. curvature 계산
6. linear.x와 angular.z publish
```

기본 제어식은 다음과 같습니다.

```text
curvature = 2 * y_local / lookahead_distance^2
angular.z = clamp(linear_speed * curvature, -max_angular_speed, max_angular_speed)
```

---

## Metal Detection and Mine Handling

금속 감지는 Arduino에서 들어오는 sensor event를 ROS 2 topic으로 변환하여 처리합니다.

주요 topic은 다음과 같습니다.

| Topic                        | Type              | Description                |
| ---------------------------- | ----------------- | -------------------------- |
| `/arduino/events`            | `std_msgs/String` | Arduino에서 수신한 event string |
| `/metal_sensor/any_detected` | `std_msgs/Bool`   | 하나 이상의 금속 센서가 감지되었는지 여부    |
| `/arduino/command`           | `std_msgs/String` | Arduino로 송풍 모터 명령 전송       |

금속이 감지되면 차량 시스템은 다음 과정을 수행합니다.

```text
1. 차량 정지
2. 현재 grid cell을 visited로 표시
3. 감지 cell을 mine cell로 등록
4. 설정된 거리만큼 후진
5. /run_metal_grasp_sequence service 호출
6. 로봇팔 pickup 성공 시 안전지대로 복귀
7. 안전지대 yaw 정렬
8. /drop_mine_at_safe_zone service 호출
9. drop 성공 후 mine cell 제거
10. 다음 coverage cell로 복귀하여 탐색 재개
```

관련 설정은 `wego_ws/src/wego/config/coverage_grid.yaml`에서 관리합니다.

```yaml
mine_detection_mode: sensor
metal_events_topic: /arduino/events
metal_state_topic: /metal_sensor/any_detected
arm_sequence_service: /run_metal_grasp_sequence
arm_drop_service: /drop_mine_at_safe_zone
metal_backup_distance: 0.075
metal_backup_speed: 0.03
return_to_safe_zone_on_mine: true
```

---

## Safe Zone Return and Resume

지뢰 모형을 집은 상태에서는 아직 탐색하지 않은 영역을 지나가지 않도록 설계하였습니다.

이를 위해 pickup 이후에는 기존 ㄷ자 coverage path를 그대로 역주행하는 것이 아니라, 이미 방문하여 안전하다고 판단된 grid cell을 기반으로 A* 경로를 생성합니다.

A* graph 구성은 다음과 같습니다.

```text
Node: grid cell
Edge: 4-neighbor movement
Cost: 1
Heuristic: Manhattan distance
```

이때 traversability rule은 다음과 같습니다.

```text
start cell: allowed
goal cell: allowed
mine cell: blocked unless it is the start cell
other cells: allowed only if VISITED
```

즉, 지뢰 모형을 운반하는 동안에는 미탐색 영역을 가능한 피하고, 이미 확인된 경로를 통해 안전지대로 복귀하도록 구성하였습니다.

---

## Manipulator System

로봇팔 시스템은 `colcon_ws`에서 관리됩니다.

주요 역할은 다음과 같습니다.

* RGB-D 카메라 기반 목표 물체 검출
* `/vision/target_pose` publish
* OpenMANIPULATOR-X MoveIt 기반 제어
* 금속 감지 event 수신
* 송풍 모터 명령 전송
* pre-motor pose 이동
* 송풍 후 ready pose 복귀
* 목표 pose 재검출
* grasp, lift, drop sequence 수행

---

## Vision Target Detection

목표 물체는 RGB-D 카메라의 color image와 depth image를 함께 사용하여 검출합니다.

현재 설정에서는 빨간색 직육면체 형태의 지뢰 모형을 대상으로 하며, HSV threshold와 contour shape filter를 이용합니다.

주요 topic은 다음과 같습니다.

| Topic                       | Description                  |
| --------------------------- | ---------------------------- |
| `/camera/color/image_raw`   | RGB image                    |
| `/camera/depth/image_raw`   | Depth image                  |
| `/camera/depth/camera_info` | Camera intrinsic information |
| `/vision/target_pose`       | 로봇팔 기준 목표 pose               |
| `/vision/target_marker`     | RViz visualization marker    |
| `/vision/debug_image`       | 검출 결과 디버그 이미지                |

검출 과정은 다음과 같습니다.

```text
1. RGB image와 depth image 수신
2. HSV threshold를 이용해 목표 색상 영역 추출
3. morphology 연산으로 noise 제거
4. contour 검출
5. 면적, aspect ratio, extent, solidity, corner count 기반 shape filtering
6. contour 중심 pixel 계산
7. depth image에서 거리값 sampling
8. camera intrinsic을 이용해 3D point 계산
9. TF를 통해 camera frame에서 robot base frame으로 변환
10. pose filter를 통해 안정적인 target pose publish
```

---

## Grasp Sequence

로봇팔 통합 sequence는 `metal_grasp_coordinator.py`에서 관리합니다.

기본 흐름은 다음과 같습니다.

```text
1. /run_metal_grasp_sequence service 요청 수신
2. pre-motor pose sequence 실행
3. Arduino로 BLOW command 전송
4. 송풍 시간 동안 대기
5. 로봇팔을 start pose로 복귀
6. 복귀 후 새로운 /vision/target_pose 대기
7. /pick_latest_target service 호출
8. grasp 및 lift 수행
9. lift 이후 목표가 여전히 검출되면 추가 pick attempt 수행
```

Arduino 송풍 명령 형식은 다음과 같습니다.

```text
BLOW,<power_percent>,<time_ms>
```

예시:

```text
BLOW,100,3000
```

정상 동작 시 Arduino event topic에서 다음과 같은 메시지를 확인할 수 있습니다.

```text
BLOW_COMMAND,POWER,100,TIME_MS,3000
MOTOR_STARTED,FORWARD,PWM,255,TIME_MS,3000
MOTOR_AUTO_STOP
```

---

## ROS 2 Interface

### Main Topics

| Topic                        | Type                             | Description         |
| ---------------------------- | -------------------------------- | ------------------- |
| `/cmd_vel`                   | `geometry_msgs/Twist`            | LIMO PRO 주행 명령      |
| `/scan`                      | `sensor_msgs/LaserScan`          | LiDAR scan          |
| `/map`                       | `nav_msgs/OccupancyGrid`         | occupancy map       |
| `/tf`, `/tf_static`          | `tf2_msgs/TFMessage`             | frame transform     |
| `/coverage_path`             | `nav_msgs/Path`                  | 생성된 coverage path   |
| `/coverage_grid_markers`     | `visualization_msgs/MarkerArray` | grid 시각화            |
| `/coverage_mission_status`   | `std_msgs/String`                | coverage mission 상태 |
| `/metal_sensor/any_detected` | `std_msgs/Bool`                  | 금속 감지 여부            |
| `/arduino/events`            | `std_msgs/String`                | Arduino event       |
| `/arduino/command`           | `std_msgs/String`                | Arduino command     |
| `/vision/target_pose`        | `geometry_msgs/PoseStamped`      | RGB-D 기반 목표 pose    |
| `/vision/debug_image`        | `sensor_msgs/Image`              | vision debug image  |

### Main Services

| Service                         | Description                     |
| ------------------------------- | ------------------------------- |
| `/start_coverage`               | coverage mission 시작             |
| `/stop_coverage`                | coverage mission 정지             |
| `/run_metal_grasp_sequence`     | 송풍 및 grasp 통합 sequence 실행       |
| `/pick_latest_target`           | 최신 target pose 기준 grasp 실행      |
| `/drop_mine_at_safe_zone`       | 안전지대 drop sequence 실행           |
| `/return_to_start_pose`         | 로봇팔 start pose 복귀               |
| `/move_pre_motor_pose_sequence` | 송풍 전 pre-motor pose sequence 이동 |

---

## Build

### 1. Clone Repository

```bash
cd ~
git clone https://github.com/YJS906/capstone_design.git
```

필요한 경우 실제 로봇 PC에서 사용하는 경로에 맞게 symbolic link를 생성할 수 있습니다.

```bash
ln -s ~/capstone_design/wego_ws ~/wego_ws
ln -s ~/capstone_design/colcon_ws ~/colcon_ws
```

---

### 2. Build Vehicle Workspace

```bash
cd ~/wego_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

### 3. Build Manipulator Workspace

Docker 내부 또는 로봇팔 workspace에서 실행합니다.

```bash
cd ~/colcon_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

로봇팔 패키지만 빌드하려면 다음 명령을 사용할 수 있습니다.

```bash
colcon build --packages-select open_manipulator_vision_grasp --symlink-install
source install/setup.bash
```

---

## Launch

### 1. Common Environment

모든 터미널에서 ROS 2 통신 설정을 맞춥니다.

```bash
export ROS_DOMAIN_ID=2
export ROS_LOCALHOST_ONLY=0
```

---

### 2. Vehicle Launch

Host PC에서 LIMO bringup, navigation, coverage planner를 실행합니다.

```bash
cd ~/wego_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=2
export ROS_LOCALHOST_ONLY=0
```

LIMO bringup:

```bash
ros2 launch wego limo_bringup_launch.py
```

Navigation:

```bash
ros2 launch wego navigation_diff_launch.py map:=/home/agilex/wego_map.yaml
```

Coverage planner:

```bash
ros2 launch wego coverage_path_launch.py coverage_config_file:=$PWD/src/wego/config/coverage_grid.yaml
```

Coverage mission 시작:

```bash
ros2 service call /start_coverage std_srvs/srv/Trigger
```

---

### 3. Manipulator Launch

Docker 내부 또는 로봇팔 workspace에서 실행합니다.

```bash
cd ~/colcon_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=2
export ROS_LOCALHOST_ONLY=0
```

USB port를 먼저 확인합니다.

```bash
ls /dev/ttyACM* /dev/ttyUSB*
```

일반적인 port convention은 다음과 같습니다.

```text
/dev/ttyACM0 : OpenCR / OpenMANIPULATOR-X
/dev/ttyACM1 : Arduino metal sensor and blower motor
/dev/ttyUSB0 : LiDAR
/dev/ttyTHS0 : LIMO base
```

로봇팔, MoveIt, 카메라, Arduino bridge, vision detector, grasp coordinator를 함께 실행합니다.

```bash
ros2 launch open_manipulator_vision_grasp dabai_vision_grasp.launch.py \
  port_name:=/dev/ttyACM0 \
  arduino_serial_port:=/dev/ttyACM1
```

카메라를 별도 실행해야 하는 경우에는 다음과 같이 실행할 수 있습니다.

```bash
ros2 launch open_manipulator_vision_grasp dabai_vision_grasp.launch.py \
  start_camera:=false \
  port_name:=/dev/ttyACM0 \
  arduino_serial_port:=/dev/ttyACM1
```

---

## Debug Commands

### Camera Topics

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic hz /vision/debug_image
```

### Target Pose

```bash
ros2 topic echo /vision/target_pose
```

### Arduino Events

```bash
ros2 topic echo /arduino/events
```

### Metal Sensor State

```bash
ros2 topic echo /metal_sensor/any_detected
```

### Arduino Command

```bash
ros2 topic echo /arduino/command
```

### Manual Blower Test

```bash
ros2 topic pub --once /arduino/command std_msgs/msg/String "{data: 'BLOW,100,3000'}"
```

### Manual Grasp Sequence

```bash
ros2 service call /run_metal_grasp_sequence std_srvs/srv/Trigger {}
```

### Manual Pick

```bash
ros2 service call /pick_latest_target std_srvs/srv/Trigger {}
```

### Manual Drop

```bash
ros2 service call /drop_mine_at_safe_zone std_srvs/srv/Trigger {}
```

### Controller Status

```bash
ros2 control list_controllers
ros2 topic hz /joint_states
ros2 topic echo --once /joint_states
```

---

## Main Configuration Files

| File                                                                              | Description                                                  |
| --------------------------------------------------------------------------------- | ------------------------------------------------------------ |
| `wego_ws/src/wego/config/coverage_grid.yaml`                                      | coverage grid, path follower, safe zone, metal handling 설정   |
| `wego_ws/docs/navigation_and_coverage_algorithms.md`                              | navigation 및 coverage algorithm 설명                           |
| `colcon_ws/src/open_manipulator_vision_grasp/config/color_grasp.yaml`             | vision, grasp, Arduino, coordinator parameter 설정             |
| `colcon_ws/src/open_manipulator_vision_grasp/launch/dabai_vision_grasp.launch.py` | camera, MoveIt, hardware, detector, Arduino bridge 통합 launch |
| `colcon_ws/src/open_manipulator_vision_grasp/scripts/detect_color_object.py`      | RGB-D 기반 target pose detection                               |
| `colcon_ws/src/open_manipulator_vision_grasp/scripts/arduino_metal_sensor.py`     | Arduino serial bridge                                        |
| `colcon_ws/src/open_manipulator_vision_grasp/scripts/metal_grasp_coordinator.py`  | 금속 감지 후 송풍, 복귀, 재검출, grasp sequence 관리                       |
| `colcon_ws/src/open_manipulator_vision_grasp/src/color_grasp_moveit.cpp`          | MoveIt 기반 OpenMANIPULATOR-X grasp, lift, drop 동작             |

---

## Important Parameters

### Coverage Parameters

```yaml
coverage_cell_size: 0.3
path_strategy: cell_centers
execution_mode: path_follower
linear_speed: 0.08
lookahead_distance: 0.15
max_angular_speed: 0.25
enable_obstacle_stop: true
front_obstacle_stop_distance: 0.12
return_to_safe_zone_on_mine: true
```

### Manipulator Parameters

```yaml
target_pose_topic: /vision/target_pose
approach_height_m: 0.08
grasp_height_m: 0.00
lift_height_m: 0.20
use_pre_motor_pose_sequence: true
motor_power_percent: 100
motor_run_time_ms: 3000
target_after_return_timeout_sec: 100.0
extra_pick_attempts_after_lift: 1
```

### Camera TF Parameters

```text
camera_x
camera_y
camera_z
camera_roll
camera_pitch
camera_yaw
```

카메라 TF 값은 `/vision/target_pose`의 위치 정확도에 직접적인 영향을 줍니다.
로봇팔이 실제 목표보다 앞, 뒤, 좌, 우로 어긋나는 경우에는 `camera_x`, `camera_y`, `camera_z` 값을 우선 조정합니다.

---

## Implementation Highlights

### 1. Vehicle and Manipulator Integration

차량 주행과 로봇팔 작업을 별도의 workspace와 실행 환경으로 분리하면서도, ROS 2 topic/service를 통해 하나의 mission sequence로 연결하였습니다.

특히 차량은 금속 감지 후 `/run_metal_grasp_sequence` service를 호출하고, 로봇팔 작업 결과에 따라 안전지대 복귀 및 탐색 재개 여부를 결정합니다.

---

### 2. Coverage-Based Exploration

단순 waypoint 주행이 아니라, 지정한 사각형 영역을 grid로 나누고 각 cell을 방문하는 coverage path planning 구조를 구현하였습니다.

탐색 상태는 `UNVISITED`, `VISITED`, `BLOCKED`, `MINE`에 가까운 개념으로 관리되며, 금속 감지 이후의 return/resume 경로에도 활용됩니다.

---

### 3. RGB-D Based Target Pose Detection

RGB image에서 색상 기반으로 목표 물체 후보를 검출하고, depth image와 camera intrinsic을 이용해 3D 위치를 계산합니다.

검출된 pose는 TF를 통해 로봇팔 기준 frame으로 변환되며, pose filter를 적용하여 일정 시간 안정적으로 검출된 목표만 grasp 대상으로 사용합니다.

---

### 4. Blower and Grasp Sequence

금속 감지 후 바로 물체를 집는 것이 아니라, 로봇팔을 송풍 위치로 이동시킨 뒤 Arduino 송풍 모터를 작동시키고, 다시 start pose로 복귀한 후 목표 pose를 재검출합니다.

이 구조는 실제 지뢰 제거 시나리오에서 주변 흙이나 이물질을 제거한 뒤 목표를 다시 확인하는 과정을 모사하기 위한 설계입니다.

---

### 5. Safe-Zone Return with A*

지뢰 모형을 집은 이후에는 이미 방문한 grid cell을 기반으로 A* 경로를 생성하여 안전지대로 복귀합니다.

이를 통해 아직 탐색하지 않은 영역을 피하고, 확인된 경로를 중심으로 복귀하도록 구성하였습니다.

---

## Project Status

구현 및 통합 완료 항목은 다음과 같습니다.

* LIMO PRO 기반 coverage grid 탐색
* map 기반 localization 및 TF 연동
* pure-pursuit 방식 coverage path following
* 금속 센서 기반 mine detection
* 금속 감지 후 차량 정지 및 후진
* RGB-D 카메라 기반 target pose detection
* OpenMANIPULATOR-X MoveIt grasp and lift
* Arduino 송풍 모터 제어
* 송풍 후 ready pose 복귀 및 target pose 재검출
* `/run_metal_grasp_sequence` 기반 로봇팔 통합 sequence
* 안전지대 복귀
* yaw alignment
* drop sequence
* 마지막 탐색 지점 복귀
* coverage exploration 재개
* 차량, 로봇팔, 카메라, Arduino를 포함한 전체 통합 미션

---

## Troubleshooting

### USB Port가 바뀌는 경우

OpenCR과 Arduino는 재부팅 또는 재연결 시 `/dev/ttyACM*` 번호가 바뀔 수 있습니다.

```bash
ls /dev/ttyACM* /dev/ttyUSB*
```

실행 전에 반드시 port를 확인하고 launch argument를 수정해야 합니다.

---

### Docker에서 장치가 보이지 않는 경우

Docker 내부에서 USB device가 보이지 않으면 로봇팔, Arduino, 카메라가 정상적으로 실행되지 않습니다.

```bash
ls /dev/ttyACM*
ls /dev/ttyUSB*
```

컨테이너 실행 시 device mapping 또는 privileged 옵션을 확인해야 합니다.

---

### Target Pose가 불안정한 경우

```bash
ros2 topic echo /vision/target_pose
rqt_image_view
```

다음 항목을 확인합니다.

* RGB-D 카메라 topic이 정상적으로 들어오는지
* `/camera/depth/camera_info`가 publish되는지
* HSV threshold가 현재 조명에 맞는지
* target object가 depth range 안에 있는지
* camera TF 값이 실제 장착 위치와 맞는지

---

### MoveIt Planning이 실패하는 경우

MoveIt planning 실패는 보통 다음 원인에서 발생합니다.

* target pose가 로봇팔 workspace 밖에 있음
* 카메라 TF 오차로 목표 pose가 잘못 계산됨
* grasp height가 너무 낮거나 높음
* joint constraint 또는 collision constraint 문제
* OpenManipulator joint limit 설정 문제

먼저 `/vision/target_pose` 값을 확인한 뒤, `camera_x`, `camera_y`, `camera_z`, `approach_height_m`, `grasp_height_m` 값을 순서대로 조정합니다.

---

## Notes

본 저장소는 캡스톤 디자인 프로젝트의 통합 구현 결과를 정리한 source snapshot입니다.

실제 로봇 구동 환경에서는 workspace 경로, USB port, camera calibration, map path, ROS_DOMAIN_ID 설정이 장비 상태에 따라 달라질 수 있습니다.

또한 본 프로젝트는 실내 데모 환경에서 지뢰 모형을 대상으로 검증한 시스템이며, 실제 폭발물 처리 환경을 대상으로 한 안전 인증 또는 실사용 시스템은 아닙니다.

---

## Author

여진승
Chungbuk National University
Department of Intelligent Robotics Engineering
