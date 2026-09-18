# kubot26 여러 대 동시 구동 / 개별 제어

한 Gazebo 월드에 kubot26 을 여러 대 띄우고 각각 독립적으로 제어하는 방법.

---

## 1. 실행

```bash
cd ~/kubot26_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch kubot26_pkgs multi_robot.launch.py            # 기본 3대
ros2 launch kubot26_pkgs multi_robot.launch.py count:=2   # 대수 변경
```

로봇은 `kubot1`, `kubot2`, `kubot3` 으로 생성되고 y축 **0.8 m 간격**으로 가운데 정렬된다.

| 로봇 | y 위치 |
|---|---|
| kubot1 | −0.8 |
| kubot2 | 0.0 |
| kubot3 | +0.8 |

스폰 높이는 0.60 m (기립 높이 0.4315 m 보다 높게 잡아 지면 관통을 피한다).

---

## 2. 제어

**토픽 이름 앞에 로봇 이름이 붙는 것 말고는 1대일 때와 동일하다.**

```bash
ros2 topic pub --once /kubot1/KubotMode std_msgs/msg/Int32 "{data: 2}"   # 1번만 walk ready
ros2 topic pub --once /kubot2/KubotMode std_msgs/msg/Int32 "{data: 4}"   # 2번만 제자리 걷기
ros2 topic pub --once /kubot3/KubotMode std_msgs/msg/Int32 "{data: 1}"   # 3번만 home pose
```

상태 확인도 마찬가지.

```bash
ros2 topic echo /kubot2/joint_states
ros2 topic echo /kubot3/zmp_X
```

### rqt

```bash
rqt
```

**Plugins → Topics → Message Publisher** → `+` 버튼 → 토픽 선택

```
/kubot1/KubotMode
/kubot2/KubotMode
/kubot3/KubotMode
```

타입 `std_msgs/msg/Int32`, `data` 에 모드 번호 입력 후 체크박스 ON.
세 개를 다 등록해두면 한 창에서 로봇별로 따로 명령할 수 있다.

### 분리되는 토픽

플러그인이 만드는 토픽 21개가 전부 네임스페이스별로 갈라진다.

```
/kubotN/KubotMode          /kubotN/joint_states      /kubotN/COM_x
/kubotN/Kubot_Control_Msg  /kubotN/zmp_X             /kubotN/COM_y
/kubotN/joy                /kubotN/zmp_Y             /kubotN/zmpFK_X
/kubotN/L_foot_FK_X        /kubotN/L_foot_ref_X      ...
```

---

## 3. 1대로 쓸 때는 그대로

```bash
ros2 launch kubot26_pkgs gazebo.launch.py
```

이 런치는 네임스페이스를 주지 않으므로 토픽이 **예전처럼 전역**으로 나온다.

| 실행 | 제어 토픽 |
|---|---|
| `gazebo.launch.py` (1대) | `/KubotMode` |
| `multi_robot.launch.py` (N대) | `/kubot1/KubotMode` … |

즉 평소 단독 작업 흐름은 바뀌지 않았다.

---

## 4. 동작하게 만들기 위해 고쳐야 했던 것 (중요)

> 이 세 가지를 되돌리면 다중 로봇이 다시 깨진다. 코드 수정 시 주의.

### 4-1. 절대경로 토픽

플러그인이 `/KubotMode`, `/Kubot_Control_Msg`, `/joy`, `/joint_states` 네 개를
**`/` 로 시작하는 절대경로**로 만들고 있었다. 절대경로는 네임스페이스를 무시하므로
모든 로봇이 같은 명령을 받고 같은 토픽에 발행했다.

→ 상대경로로 변경. (나머지 17개는 원래 상대경로였다)

```cpp
// 이렇게 쓰면 안 된다
create_subscription<...>("/KubotMode", 10, ...)
// 이렇게
create_subscription<...>("KubotMode", 10, ...)
```

### 4-2. IMU 전역 조회

```cpp
Sensor = sensors::get_sensor("IMU");     // 여러 대면 "IMU" 가 N 개 → 남의 로봇 것을 잡음
```

→ 모델 스코프 이름으로 변경.

```cpp
const std::string scoped =
    model->GetWorld()->Name() + "::" + model->GetName() + "::base_link::IMU";
Sensor = sensors::get_sensor(scoped);
```

### 4-3. ROS 노드 이름 충돌 → gzserver 크래시

가장 크게 막혔던 부분.

`gazebo_ros::Node::Get(_sdf)` 는 **플러그인 이름을 그대로 ROS 노드 이름으로 쓴다.**
3대가 전부 `/kubot26_plugin` 이 되어 충돌했고, 이름이 겹치면 `Get()` 은
**nullptr 를 반환**하는데 그걸 그대로 역참조해서 **gzserver 가 SIGSEGV 로 죽었다.**

```
[ERROR] [gazebo_ros_node]: Found multiple nodes with same name: /kubot26_plugin.
[ERROR] [gzserver-1]: process has died ... exit code -11
```

→ 모델 이름으로 고유화 + null 검사.

```cpp
this->node_ = gazebo_ros::Node::Get(_sdf, this->model->GetName() + "_plugin");
if (!this->node_) { gzerr << "..."; return; }
```

### 4-4. URDF 에 빈 namespace 를 넣으면 안 된다

`spawn_entity.py -robot_namespace` 는 플러그인 태그에 `<ros><namespace>` 를
**주입**한다. URDF 에 빈 `<namespace></namespace>` 를 미리 박아두면 주입이
덮어쓰지 못해 네임스페이스가 먹지 않는다.

```xml
<!-- 이렇게 두어야 한다 (namespace 태그 없이) -->
<gazebo>
  <plugin name="kubot26_plugin" filename="libkubot_control_node.so">
  </plugin>
</gazebo>
```

---

## 5. 설정 변경

`src/kubot26_pkgs/launch/multi_robot.launch.py`

```python
SPACING_Y = 0.8     # 로봇 간 y 간격 [m]
SPAWN_Z   = 0.60    # 스폰 높이 [m]
```

로봇 이름 규칙은 `spawn_robots()` 안의 `name = f'kubot{i + 1}'`.

---

## 6. 성능

3대 동시 구동 기준.

| 항목 | 값 |
|---|---|
| `/kubot1/joint_states` 발행 주기 | 966 Hz |
| 물리 스텝 | 1000 Hz |
| 실시간 배율 | 약 0.96 |

대수를 늘리면 접촉 계산이 증가해 떨어진다. 필요하면 `gz stats` 로 확인.

---

## 7. 문제 해결

| 증상 | 확인할 것 |
|---|---|
| 모든 로봇이 같이 움직임 | 플러그인 토픽이 절대경로(`/`)로 돌아갔는지 |
| gzserver 가 스폰 직후 죽음 | 노드 이름 충돌. 로그에서 `Found multiple nodes with same name` |
| `/kubotN/...` 토픽이 안 생김 | `-robot_namespace` 가 전달됐는지, URDF 에 빈 namespace 태그가 없는지 |
| 로봇은 떴는데 안 움직임 | `ros2 topic hz /kubotN/joint_states` 로 제어 루프 생존 확인 |
| 런치 종료 후 토픽이 남음 | `robot_state_publisher` 잔존. `pkill -x robot_state_publisher` |
