# kubot26_pkgs

KUDOS Kubot26 휴머노이드 — **ROS 2 Humble + Gazebo Classic** 패키지.

SolidWorks URDF 익스포터로 뽑은 ROS1(catkin) 패키지를, kubot25_ws 구조에 맞춰
ROS 2 + Gazebo Classic 으로 마이그레이션한 것이다.

## 빌드 & 실행

```bash
cd ~/kubot26_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch kubot26_pkgs gazebo.launch.py
```

동작 명령은 `/KubotMode` 토픽으로 준다 (rqt 사용 가능).

```bash
ros2 topic pub --once /KubotMode std_msgs/msg/Int32 "{data: 2}"   # WALK READY
```

## 기구 구성

다리 체인이 kubot25 와 다르다 — **Hip_pitch → Hip_roll → Hip_yaw → Knee → Ankle_pitch → Ankle_roll**
(kubot25 는 yaw → roll → pitch). 힙 3축이 한 점에 모이지 않고
**Hip_pitch 축이 Hip_roll/yaw 교점에서 75.6 mm 떨어져 있다** (서보 사이 브래킷).

| 축 쌍 | 최단거리 |
|---|---|
| Hip_pitch ↔ Hip_roll | 75.600 mm |
| Hip_roll ↔ Hip_yaw | 0.000 mm |
| Ankle_pitch ↔ Ankle_roll | 0.000 mm |

## 역기구학

힙이 구면관절이 아니라서 kubot25 의 Kajita 형 닫힌 해를 쓸 수 없다.
`CKubot::Geometric_IK_L/R` 은 이름만 유지하고 내부는 **수치해(Levenberg–Marquardt DLS)** 다.

- 관절 한계 active-set 처리, 다중 시드 + 무작위 재시작, warm-start
- 검증: FK↔IK 왕복 오차 < 1e-6 m (200회 무작위, 미수렴 0)
- 야코비안 해석식 vs 수치미분 오차 1.7e-8
- warm-start 시 양다리 **10.4 µs/스텝** (1 kHz 예산의 1 %)

q 인덱스 규약은 기존 enum 을 그대로 유지한다:
`q = [Hip_yaw, Hip_roll, Hip_pitch, Knee, Ankle_pitch, Ankle_roll]`

## 모터

| 모터 | 관절 |
|---|---|
| Dynamixel MX-106 (8.4 N·m, 4.71 rad/s) | Hip_pitch, Hip_roll, Knee_pitch, Ankle_pitch, Ankle_roll |
| Dynamixel MX-64 (6.0 N·m, 6.60 rad/s) | Hip_yaw, Shoulder_roll, Elbow_pitch |
| Dynamixel MX-28 (2.5 N·m, 5.76 rad/s) | Hand_pitch, Neck_yaw, Head_pitch |

URDF 의 effort/velocity 는 위 스펙으로 맞췄고, 플러그인이 매 틱 토크를
해당 스톨 토크로 포화시킨다.

## Gazebo 안정화 — 관성 하한

SolidWorks 실측 관성(1e-5 ~ 1e-4 kg·m²)은 물리적으로 맞지만 ODE 접촉 솔버에
너무 작아서, 접촉 임펄스가 발산하며 모델 전체가 원점으로 붕괴한다.

실측 결과: 하한 `1e-4` → 붕괴 / `3e-4` → 정상. 여유를 둬 **5e-4** 를 채택했다.
원본 관성값은 `_backup_pre_migration/urdf/kubot26_pkgs.urdf` 에 보존.

충돌 형상도 visual 메쉬(총 275,574 삼각형, base_link 만 110,234개) 대신
링크별 **경계상자**로 대체했다.

## 마이그레이션 메모

- `<?xml ... encoding="utf-8"?>` 선언 제거 — `spawn_entity.py`(lxml)가 거부한다
- ROS1 launch 파일은 `launch/ros1_legacy/` 로 격리 (ROS 2 에서 동작 불가)
- URDF 의 world 고정 핀은 주석 상태 (고정 베이스 검증용, 기본 비활성)

## 남은 과제

`CKubot.cpp` 의 링크 길이·질량 파라미터 일부가 아직 kubot25 값이다.
보행 패턴 생성기(`walkingPatternGenerator`, ZMP preview)는 kubot26 기구로
재검증이 필요하다.
