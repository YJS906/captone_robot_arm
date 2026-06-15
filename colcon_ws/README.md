# Capstone Robot Arm

ROS 2 Humble 기반 OpenMANIPULATOR-X 로봇팔 통합 패키지입니다.
RGB-D 카메라로 목표 물체의 pose를 검출하고, Arduino 금속 센서 및 송풍 모터와 연동하여 물체 접근, 송풍, 복귀, 재검출, 집기 동작을 수행합니다.

## 주요 동작 흐름

```text
금속 탐지
-> 목표 pose 검출
-> 목표 물체 근처로 로봇팔 이동
-> Arduino 송풍 모터 작동
-> 로봇팔 준비 자세로 복귀
-> 목표 pose 재검출
-> 물체 집기
-> 물체 들어 올리기
```

## 패키지 위치

```text
src/open_manipulator_vision_grasp
```

주요 파일:

```text
launch/dabai_vision_grasp.launch.py
config/color_grasp.yaml
scripts/arduino_metal_sensor.py
scripts/metal_grasp_coordinator.py
scripts/detect_depth_object.py
src/color_grasp_moveit.cpp
```

## 주요 기능

- RGB-D 카메라 기반 목표 물체 pose 검출
- `/vision/target_pose` 발행
- MoveIt 기반 OpenMANIPULATOR-X 제어
- Arduino 금속 센서 데이터 수신
- Arduino 송풍 모터 명령 전송
- 금속 탐지 후 자동 통합 시퀀스 실행
- 로봇팔 복귀 후 목표 pose 재검출
- 재검출된 pose 기준으로 물체 집기 및 들어 올리기

## 빌드 방법

```bash
cd ~/colcon_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select open_manipulator_vision_grasp
source install/setup.bash
```

## 실행 방법

OpenCR과 Arduino 포트는 환경에 맞게 수정해야 합니다.

```bash
ros2 launch open_manipulator_vision_grasp dabai_vision_grasp.launch.py \
  port_name:=/dev/ttyACM1 \
  arduino_serial_port:=/dev/ttyACM0
```

포트 확인:

```bash
ls /dev/ttyACM* /dev/ttyUSB*
```

## 주요 파라미터

### 카메라 TF 보정

파일:

```text
launch/dabai_vision_grasp.launch.py
```

주요 값:

```python
camera_x
camera_y
camera_z
camera_roll
camera_pitch
camera_yaw
```

이 값들은 `/vision/target_pose` 자체에 영향을 줍니다.
따라서 실제 로봇팔이 물체를 잡는 위치가 맞지 않을 경우 이 값을 조정합니다.

### 로봇팔 및 Arduino 시퀀스 설정

파일:

```text
config/color_grasp.yaml
```

주요 값:

```yaml
prepare_approach_height_m
approach_height_m
grasp_height_m
motor_power_percent
motor_run_time_ms
```

의미:

- `prepare_approach_height_m`: 금속 탐지 후 송풍 모터를 작동시키기 위해 물체 근처로 접근할 때 사용하는 z축 높이
- `approach_height_m`: 실제 물체를 잡기 전 pre-grasp 위치의 z축 높이
- `grasp_height_m`: 실제 grasp 위치의 z축 보정값
- `motor_power_percent`: Arduino 송풍 모터 출력, 100으로 설정하면 Arduino 코드 기준 PWM 255
- `motor_run_time_ms`: 송풍 모터 작동 시간, 단위는 ms

## Arduino 명령 형식

ROS 2에서 Arduino로 다음 형식의 문자열을 전송합니다.

```text
BLOW,<출력 퍼센트>,<작동 시간 ms>
```

예시:

```text
BLOW,100,3000
```

Arduino 코드 기준:

```text
100% -> PWM 255
```

## 디버깅 명령어

Arduino 이벤트 확인:

```bash
ros2 topic echo /arduino/events
```

Arduino로 전송되는 명령 확인:

```bash
ros2 topic echo /arduino/command
```

목표 pose 확인:

```bash
ros2 topic echo /vision/target_pose
```

수동 집기 실행:

```bash
ros2 service call /pick_latest_target std_srvs/srv/Trigger {}
```

Arduino 모터 단독 테스트:

```bash
ros2 topic pub --once /arduino/command std_msgs/msg/String "{data: 'BLOW,100,3000'}"
```

정상일 경우 `/arduino/events`에 다음과 비슷한 메시지가 출력됩니다.

```text
BLOW_COMMAND,POWER,100,TIME_MS,3000
MOTOR_STARTED,FORWARD,PWM,255,TIME_MS,3000
MOTOR_AUTO_STOP
```

## 현재 통합 시퀀스

`metal_grasp_coordinator.py`에서 전체 동작을 제어합니다.

동작 순서:

```text
1. Arduino 금속 센서에서 금속 감지
2. /vision/target_pose가 안정적으로 들어오는지 확인
3. /prepare_grasp_pose 서비스 호출
4. 목표 물체 근처로 로봇팔 이동
5. Arduino 송풍 모터 명령 전송
6. /return_to_start_pose 서비스 호출
7. 로봇팔 준비 자세로 복귀
8. 복귀 후 새로운 /vision/target_pose 대기
9. /pick_latest_target 서비스 호출
10. 물체 집기 및 들어 올리기
```

## 주의 사항

- OpenCR 포트와 Arduino 포트를 정확히 지정해야 합니다.
- Docker 환경에서는 컨테이너 내부에서 `/dev/ttyACM*`, `/dev/ttyUSB*`가 보이는지 확인해야 합니다.
- 카메라 TF 값이 맞지 않으면 로봇팔이 실제 물체 위치와 다른 곳으로 이동할 수 있습니다.
- 송풍 모터 위치만 조정하고 싶으면 `prepare_approach_height_m`을 수정합니다.
- 실제 잡는 위치가 맞지 않으면 `camera_x`, `camera_y`, `camera_z`를 우선 조정합니다.

## 외부 의존성

이 저장소에는 프로젝트 패키지만 포함되어 있습니다.
아래 패키지들은 별도로 ROS 2 workspace에 설치하거나 clone해야 합니다.

- OpenMANIPULATOR-X ROS 2 패키지
- DynamixelSDK
- dynamixel_hardware_interface
- dynamixel_interfaces
- Orbbec / Astra camera driver
- MoveIt 2
- ros2_control 관련 패키지

## 개발 및 테스트 환경

```text
Ubuntu 22.04
ROS 2 Humble
Docker 기반 ROS 2 개발 환경
OpenMANIPULATOR-X
OpenCR
Arduino UNO
RGB-D Camera
```
