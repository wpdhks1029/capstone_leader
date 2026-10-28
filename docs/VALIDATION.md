# 검증 범위 — 2026-10-27 실기체 결과

## 사용자가 실제 확인한 항목

- IMU 약 20Hz, LiDAR 약 12Hz, 고정 TF 및 저장 지도/AMCL 스캔 정합.
- MD200T 실제 RPM 응답, M1 왼쪽/M2 오른쪽, 전진 및 정지.
- 새 polling+vy=0 EKF 구성에서 180초 정지 관측 위치 점프 없음, stale feedback 0회.
- 반지름 0.085m/피드백 배율 1.18에서 40cm 지상 직진 5회 실측 40cm, 쏠림 없이 정지.
- 좌·우 45° 지상 회전 각 3회 실측 약45° 및 실제 정지.
- 0.15m/s 설정에서 실제 주행 및 장애물 우회 관찰.
- 원래 DWB 새 목표 회전에서 angular0.005 → 정수RPM0, 이후 ±1RPM 저속반복을 실제 제어기 로그로 확인.
- 실제 실행 plugin이 RotationShimController인 구성에서 버벅거림 없는 회전, 반대 방향 목표로 회전→이동→도착→자동 정지를 사용자가 확인.
- 현재 bridge 구성에서 /leader/path 포즈 1→13 증가, 이후 Raspberry Pi에서도 map 좌표의 /leader/odom과 path21 실제 수신.

## 한계와 미검증

- 마지막 목표 도착/정지는 사용자 관찰 보고이며, 그 최종 NavigateToPose SUCCEEDED 로그는 따로 수집하지 않았음.
- 최대 명령 0.15m/s에서 실제 속도와 제동 거리의 정밀 계측 미완료. 1.18은 경험적 피드백 환산값이며 모든 속도/바닥의 보정 보장은 아님.
- 약10Hz 수신 중 최대0.484초 간격·버스트 관찰. 통신지연/스케줄링 원인, 후발 watchdog/시간동기 검증 필요.
- 전체 센서 손실·TF 손실·통신 두절에서 후발 정지, 경로 초기화/AMCL 재정합 연속성, 장시간 추종 검증 미완료.
- IMU Ctrl+C 이중 shutdown 오류는 원래 드라이버에 남아 있음.
- URDF의 바퀴 시각화 반지름/높이는 원래 0.075m 표기가 남아 있으나 실제 제어 반지름은 0.085m. 센서 TF는 실측으로 정정됨.
- 원래 MD stale covariance의 비대칭 표기 및 bridge의 pose covariance0 표기는 통합 개선 후보이며 이번 성공 스냅샷에서 변경하지 않음.
- 포팅용 setup/실행 helper와 새 checkout 전체의 하드웨어 재시험은 별도. 원래 소스는 manifest hash로 보존.

원래 2026-09-06 성공 archive 및 보호 Git clone은 이번 export로 변경하지 않음.

## Export-time software checks (2026-10-28)

- 140 original file hashes match the live source snapshot.
- 33 motor/pulse/ground mock regression checks and 9 bridge freshness checks passed (42 total); no ROS nodes or motor commands started by these tests.
- Python syntax, YAML/XML parsing, shell syntax and relocated BT rendering passed.
- Isolated colcon build completed convoy_robot and iahrs_ros2, then stalled during leader_robot_control setup without an error result; the incomplete build was stopped. Full clean-build reproducibility is therefore not certified by this export.
- Existing successful robot workspace/install and the protected archive were not rebuilt or modified. Original archive SHA256 check passed again.
