# Gazebo 로봇 리스폰 / 리셋 정리

kubot26 시뮬레이션에서 로봇이 넘어졌을 때 되돌리는 방법.

---

## 1. 방법 비교

| 방법 | 모델 위치 | 관절 각도 | sim 시간 | 용도 |
|---|---|---|---|---|
| `/reset_world` | 스폰 위치로 | **유지** | 유지 | 넘어졌을 때 빠르게 세우기 |
| `/reset_simulation` | 스폰 위치로 | 0으로 | **0으로 리셋** | 완전 초기화 |
| `delete_entity` + `spawn_entity` | 지정 위치 | 0으로 | 유지 | 위치를 바꿔서 다시 놓고 싶을 때 |
| GUI `Ctrl+R` | 스폰 위치로 | 0으로 | 0으로 리셋 | 화면에서 바로 |
| GUI `Ctrl+Shift+R` | 스폰 위치로 | 유지 | 유지 | 화면에서 위치만 |

> `/reset_world`는 **관절 각도를 그대로 둔 채 몸통 위치만** 되돌린다.
> walk ready 상태에서 리셋하면 웅크린 자세(z≈0.380) 그대로 다시 선다.
> 관절까지 펴려면 모드 1(HOME POSE)을 같이 준다.

---

## 2. 명령어

### 위치만 되돌리기 (가장 자주 씀)

```bash
ros2 service call /reset_world std_srvs/srv/Empty
```

### 완전 초기화

```bash
ros2 service call /reset_simulation std_srvs/srv/Empty
```

### 삭제 후 원하는 위치에 재스폰

```bash
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'kubot26_robot'}"

ros2 run gazebo_ros spawn_entity.py \
  -entity kubot26_robot \
  -file ~/kubot26_ws/install/kubot26_pkgs/share/kubot26_pkgs/urdf/kubot26_pkgs.urdf \
  -x 0 -y 0 -z 0.6
```

### 리셋 + 기본 자세까지 한 번에

```bash
ros2 service call /reset_world std_srvs/srv/Empty
sleep 1
ros2 topic pub --once /KubotMode std_msgs/msg/Int32 "{data: 2}"   # WALK READY
```

### rqt에서

`Plugins → Services → Service Caller` 에서 `/reset_world` 선택 후 Call.

---

## 3. 스폰 높이 기준값

| 상태 | base_link 높이 |
|---|---|
| 스폰 (공중에서 낙하 시작) | 0.60 m |
| 두 발로 기립 (전 관절 0°) | **0.4315 m** |
| WALK READY (웅크림) | **0.3796 m** |
| 넘어짐 | 0.07 m 근처 |

발바닥은 `L_Foot` / `R_Foot` 링크 원점과 일치한다. 즉 base가 0.431535 m일 때 발바닥이 지면(z=0)에 닿는다.

스폰 높이를 0.4315보다 낮게 주면 지면에 파고든 채 시작해 접촉이 튄다. 0.5~0.6 권장.

---

## 4. 주의: 리셋 후 로봇이 힘없이 쓰러지던 버그 (수정 완료)

**증상** — `/reset_simulation` 또는 GUI `Ctrl+R` 이후 로봇이 토크를 잃고 주저앉으며, `/joint_states` 발행도 멈추고 영영 복구되지 않음.

**원인** — 플러그인의 `dt` 가드. sim 시간이 0으로 되감기면 `dt`가 음수가 되는데, 기준 시각을 갱신하지 않고 반환해서 매 틱 같은 가드에 걸렸다.

```cpp
dt = current_time.Double() - last_update_time.Double();
if (dt <= 0.0) {
    return;                        // last_update_time 갱신 없이 반환 → 영구 정지
}
...
last_update_time = current_time;   // 도달 불가
```

제어 루프가 멈추니 `jointcontroller()`가 호출되지 않고 `SetForce`가 안 걸려 토크 0이 된다.

`/reset_world`는 sim 시간을 건드리지 않아서 이 버그를 피해 간다. 그래서 "어떤 리셋은 되고 어떤 리셋은 안 되는" 것처럼 보였다.

**수정** — 되감김을 감지해 기준 시각과 내부 시계를 다시 맞춘다.

```cpp
if (dt <= 0.0) {
    last_update_time = current_time;
    if (dt < 0.0) {            // 되감김 = 리셋
        time = 0.0;
        Kubot.realTime = 0.0;
        Kubot.startTime = 0.0;
    }
    return;
}
```

**검증** — `/reset_simulation` 3회 연속 호출 시 매번 z=0.43145로 정상 기립, `/joint_states` 발행 유지.

> 이 수정이 들어간 커밋 이후 버전을 써야 한다. 그 전 빌드에서는 `/reset_world`만 쓸 것.

---

## 5. 자주 쓰는 확인 명령

```bash
# 로봇 현재 높이
gz topic -e /gazebo/kubot26_world/pose/info \
  | grep -A6 'name: "kubot26_robot"$' | grep -E "^    z:" | head -1

# 관절 상태
ros2 topic echo /joint_states --once

# 제어 루프 살아있는지 (메시지가 계속 와야 정상)
ros2 topic hz /joint_states
```

---

## 6. 동작 모드 번호

```bash
ros2 topic pub --once /KubotMode std_msgs/msg/Int32 "{data: <번호>}"
```

| 번호 | 동작 |
|---|---|
| 1 | HOME POSE (전 관절 0°) |
| 2 | WALK READY |
| 3 | RAISE LEFT ARM |
| 4 | WALKTHREETIMES (제자리 걷기) |
| 5 | KICK READY |
| 7 | WALKING PHYSICS |
| 25 / 26 | STANDUP FRONT / BACK |
| 44 | KICK LEFT |
| 45 | KICK RIGHT *(미구현 — 빈 함수)* |
| 70 / 71 | TURN LEFT / RIGHT *(미구현 — 빈 함수)* |
| 100 | GOALKEEPER |
