# capstone_leader — 선두로봇 검증 구성

Jetson Orin Nano / Ubuntu 22.04 / ROS 2 Humble에서 2026-10-27 실기체로 확인한 선두로봇 코드·설정의 스냅샷입니다. 후발로봇 코드는 포함하지 않습니다.

사용자가 확인한 범위: 직진 거리 보정, 좌우 회전, Rotation Shim 적용 후 회전→목표 이동→도착→자동 정지, 장애물 우회 관찰, 후발 Raspberry Pi의 `/leader/odom`·`/leader/path` 수신. 센서 손실·통신 두절 시 전체 시스템 정지, 장시간 추종, 모든 속도의 거리 정확도는 검증 완료 항목이 아닙니다.

## 현재 사용할 파일

| 역할 | 파일 |
|---|---|
| 센서·EKF·보정 모터 제어기 | `diagnostics/leader_nav_015_base.launch.py` |
| PID216 주기 조회·피드백 환산 | `diagnostics/md200t_polling_controller.py` |
| EKF 횡속도 제약 | `diagnostics/leader_polling_constrained_ekf.yaml` |
| 실제 성공 Nav2 설정 | **`diagnostics/nav2_015_rotation_shim_check.yaml`** |
| 저속 회전 복구 BT | `diagnostics/navigate_015_recovery.xml` |
| 선두 인터페이스 | `src/convoy_robot/convoy_robot/real_leader_bridge_node.py` |
| 실제 센서 TF | `src/leader_robot_control/urdf/leader_robot.urdf.xacro` |
| 저장 지도 | `maps/leader_map.yaml` + `leader_map.pgm` |

`src/leader_robot_control`의 기존 bringup/Nav2 설정은 출처 보존용입니다. **현재 성공 구성을 실행할 때는 위 diagnostics의 launch와 Rotation Shim YAML을 사용합니다.** `simple_odom`은 별도로 실행하지 않습니다. `convoy_robot`은 bridge 소스만 추출한 선두 전용 패키지이며 기존 통합 패키지와 동시에 빌드하지 않습니다.

## 설정

- 바퀴 반지름 `0.085m`, 유효 윤거 `0.466m`, 피드백 환산 배율 `1.18`.
- M1 왼쪽·M2 오른쪽. MD200T ID 1, 57600bps, RPM 명령 상한 25.
- Nav2 직진 **명령 상한** `0.15m/s`, 회전 `0.10rad/s`. 실제 속도 정밀 계측은 미완료입니다. 피드백 보정은 송신 RPM 환산에는 적용되지 않습니다.
- 선두 차체 외곽 폭 60cm·길이 54cm, 축에서 앞뒤 각각 27cm, costmap 반경 0.42m.
- LiDAR `base_link → laser`: `(0.26, 0, 0.24)m`, yaw π. IMU `(0, 0, 0.15)m`.
- `/leader/odom`, `/leader/path` 좌표계 `map`, 약 10Hz. 경로는 약 0.1m 이동마다 추가됩니다.

## 빌드

새 작업공간으로 빌드합니다. 기존 `~/robot_ws`나 성공 백업을 덮어쓰지 않습니다.

```bash
git clone git@github.com:wpdhks1029/capstone_leader.git
cd capstone_leader
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
# 기존 manifests에 누락된 런타임 의존성을 명시합니다.
sudo apt install python3-serial python3-yaml psmisc \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-nav2-rotation-shim-controller ros-humble-robot-localization \
  ros-humble-robot-state-publisher ros-humble-xacro
colcon build --symlink-install
source install/setup.bash
```

이 작업공간의 패키지 이름은 기존 것과 같습니다. 여러 작업공간의 install을 무작위로 함께 source하지 말고, 현재 터미널에서 이 checkout의 `install/setup.bash`가 마지막으로 적용되도록 합니다. 시리얼 장치 접근 권한과 by-id 경로는 실제 기체에서 확인하세요.

## 실행 순서

현재 기체의 중복 센서·제어기·EKF·AMCL·Nav2를 먼저 종료한 **새 운용 세션**에 사용합니다. 아래 helper는 복제 위치에 맞게 BT 파일 경로만 임시 YAML로 바꿉니다. 원본 스냅샷은 수정하지 않습니다. helper 자체의 실기체 재검증은 미완료이며, 구성 요소는 기존 실기체 시험과 동일합니다.

각 Jetson 터미널에서 repository로 이동하고 실행합니다.

```bash
# 1번: 모터 이동 금지 상태로 센서·TF·EKF·실제 모터 통신 시작
bash scripts/run_leader.sh base false
```

`false`라도 실제 포트를 열고 0RPM 및 피드백 조회를 전송합니다. 포트 소유자를 중복 실행하지 마세요.

```bash
# 2번: 저장 지도 + AMCL
bash scripts/run_leader.sh localization
```

노트북 로컬 터미널에서:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0
ros2 launch nav2_bringup rviz_launch.py use_sim_time:=false
```

Fixed Frame `map`, Map `/map`는 Transient Local/Reliable, LaserScan `/scan`은 Best Effort. **2D Pose Estimate**는 바퀴축 중앙 위치와 실제 전방을 지정합니다. 저장 지도 자체를 새로 만드는 도구가 아닙니다.

실제 이동 시험 준비가 끝나면 1번 launch를 종료하고 `base true`로 재시작합니다. **EKF가 초기화되므로 AMCL 초기 위치를 다시 맞춘 뒤** 다음을 실행합니다. `allow_motion`은 실행 중 param set만으로 바뀌지 않습니다.

```bash
# 1번 교체: 실제 구동 허용
bash scripts/run_leader.sh base true
# 3번: Rotation Shim을 사용하는 Nav2
bash scripts/run_leader.sh navigation
```

목표 지정 전 다른 Jetson 터미널에서 환경을 source하고 확인합니다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0
ros2 param get /controller_server FollowPath.plugin
# nav2_rotation_shim_controller::RotationShimController
ros2 param get /md200t_controller feedback_speed_scale
# 1.18
```

초기에는 빈 공간에서 가까운 목표 하나를 시험합니다. **Nav2 Goal/2D Nav Goal**은 도착 차체 중심 위치와 도착 방향이며, 지정하면 실제 움직입니다. 제자리 회전 공간과 정지 수단을 준비합니다. 180° 회전에는 현재 제한에서 31초 이상 걸릴 수 있습니다.

```bash
# 목표 취소. Ctrl+C로 action client만 끄는 것으로는 취소를 보장하지 않습니다.
ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal '{}'
```

실제 정지를 확인하고, 정지 실패 시 즉시 하드웨어 전원을 차단합니다.

```bash
# 4번: 위치 정합 후 bridge 시작, 새 정합/재시작 시 기존 경로 누적에 주의
bash scripts/run_leader.sh bridge
```

후발/노트북에서 Domain 10을 설정한 뒤 `/leader/odom`, `/leader/path`를 확인합니다. 선두의 `/cmd_vel`, `/odom`, `/scan`, TF와 후발 토픽을 공유하지 않도록 후발 이름공간을 분리해야 합니다. 이 저장소는 후발 모터를 실행하지 않습니다.

## 검증과 한계

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/validate_snapshot.py
python3 -B -m unittest discover -s diagnostics/tests
python3 -B -m unittest discover -s src/convoy_robot/tests
```

모의 회귀 테스트는 ROS 메시지 형식을 사용하지만 ROS context/노드·모터를 시작하지 않습니다. `diagnostics`의 주행 시험 도구는 명시적 실행 옵션이 있어야 구동하며, 과거의 max_rpm=5 전용 precheck가 있으므로 현재 max_rpm=25 운용 상태에서 무작정 실행하지 않습니다.

[검증 기록과 미검증 항목](docs/VALIDATION.md), [파일 해시·출처](provenance/SOURCE_MANIFEST.json), [보존한 기준선](provenance/BASELINES.md)을 참고하세요. 로그·bag·SSH 키·인증정보·build/install 산출물은 포함하지 않았습니다. 원본 성공 archive와 기존 workspace는 그대로 보존했습니다.

SLLidar의 원래 라이선스는 `src/sllidar_ros2/LICENSE`에 있습니다. 자체 패키지의 기존 라이선스 미정(TODO) 표기는 이 스냅샷에서 임의로 새 라이선스로 바꾸지 않았습니다.
