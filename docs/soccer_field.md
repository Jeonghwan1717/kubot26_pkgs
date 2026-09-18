# RoboCup 축구장 (HSL) Gazebo 세팅

kubot26 용 RoboCup Humanoid Soccer League 축구장 월드.
**필드 수치는 파일 한 곳의 숫자만 바꾸면 전부 재생성된다.**

---

## 1. 실행

```bash
cd ~/kubot26_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch kubot26_pkgs soccer.launch.py                      # 축구장 + 로봇 3대 (킥오프)
ros2 launch kubot26_pkgs soccer.launch.py formation:=defense   # 골키퍼 + 수비 2
ros2 launch kubot26_pkgs soccer.launch.py formation:=line      # 한 줄 정렬
ros2 launch kubot26_pkgs soccer.launch.py count:=2             # 대수 변경
```

제어는 로봇별 네임스페이스로 한다 (별도 문서 참고).

```bash
ros2 topic pub --once /kubot1/KubotMode std_msgs/msg/Int32 "{data: 2}"
```

---

## 2. 수치 바꾸는 법  ← 핵심

필드는 스크립트가 생성한다. 손으로 world 파일을 고치지 말 것.

### 2-1. 숫자 수정

`src/kubot26_pkgs/scripts/gen_field.py` 의 `DIVISIONS` 딕셔너리만 고친다.

```python
DIVISIONS = {
    'small': dict(A=9.0,  B=6.0,  C=0.5, D=1.8, E=1.2,
                  F=1.0, G=3.0, H=1.5, I=1.5, J=1.0, K=2.0, L=4.0,
                  line_w=0.05, goal_line_w=0.10, corner_r=0.0),
    'mid':   dict(A=14.0, B=9.0,  C=0.7, D=2.4, E=1.5, ...),
    'large': dict(A=22.0, B=14.0, C=0.6, D=2.4, E=1.8, ...),
}
```

| 기호 | 의미 | 단위 |
|---|---|---|
| `A` | 필드 길이 (라인 안쪽) | m |
| `B` | 필드 폭 | m |
| `C` | 골 깊이 | m |
| `D` | 골 폭 (포스트 안쪽) | m |
| `E` | 골 높이 | m |
| `F` | 골에어리어 길이 | m |
| `G` | 골에어리어 폭 | m |
| `H` | 페널티마크 ~ 골라인 거리 | m |
| `I` | 센터서클 지름 | m |
| `J` | 보더(외곽 여유) | m |
| `K` | 페널티에어리어 길이 | m |
| `L` | 페널티에어리어 폭 | m |
| `line_w` | 일반 라인 폭 | m |
| `goal_line_w` | 골라인 폭 (일반 라인보다 두껍다) | m |
| `corner_r` | 코너 아크 반지름 (0 이면 없음) | m |

스크립트 상단의 다른 상수도 필요하면 조정한다.

```python
LINE_T   = 0.005   # 라인 두께(높이)
POST_R   = 0.05    # 골포스트 반지름
BALL_R   = 0.065   # 공 반지름 (FIFA size 1)
CARPET_T = 0.02    # 카펫 두께
```

### 2-2. 재생성

```bash
cd ~/kubot26_ws/src/kubot26_pkgs
python3 scripts/gen_field.py small > worlds/robocup_hsl_small.world
python3 scripts/gen_field.py mid   > worlds/robocup_hsl_mid.world
python3 scripts/gen_field.py large > worlds/robocup_hsl_large.world
```

인자를 생략하면 `small` 이다.

### 2-3. 빌드 후 실행

```bash
cd ~/kubot26_ws && colcon build --symlink-install
```

`--symlink-install` 이라 world 파일은 심링크로 연결되어 재빌드 없이도 반영되지만,
새 파일을 추가했다면 한 번 빌드해야 한다.

### 2-4. 다른 디비전 필드로 띄우기

`launch/soccer.launch.py` 의 이 줄을 바꾼다.

```python
world_path = os.path.join(pkg_share_dir, 'worlds', 'robocup_hsl_small.world')
```

---

## 3. 현재 적용된 공식 규격

출처: [HSL-Rules](https://github.com/RoboCup-HumanoidSoccerLeague/HSL-Rules) 저장소의
`rules/field_diagram_macros.tex` (2026-06-29 커밋 = RoboCup 2026 인천 대회 최종본)

| 항목 | small | mid | large |
|---|---|---|---|
| 필드 길이 (A) | 9.0 | 14.0 | 22.0 |
| 필드 폭 (B) | 6.0 | 9.0 | 14.0 |
| 골 깊이 (C) | 0.5 | 0.7 | 0.6 |
| 골 폭 (D) | 1.8 | 2.4 | 2.4 |
| 골에어리어 (F×G) | 1.0×3.0 | 1.0×4.0 | 1.0×4.0 |
| 페널티에어리어 (K×L) | 2.0×4.0 | 3.0×6.0 | 3.5×7.0 |
| 페널티마크 (H) | 1.5 | 2.0 | 2.5 |
| 센터서클 (I) | 1.5 | 3.0 | 4.0 |
| 라인 폭 | 0.05 | 0.05 | 0.12 |
| 골라인 폭 | 0.10 | 0.10 | 0.20 |
| 보더 (J) | 1.0 | 1.0 | 1.0 |
| 코너 아크 | 없음 | 0.5 | 1.0 |

> **골 높이(E) 는 잠정치다** (small 1.2 / mid 1.5 / large 1.8 m).
> 룰북의 필드 매크로에 값이 없다. 다만 **시뮬레이션상 영향이 없어 그대로 둔다**:
> 공이 지면을 구르는 수준이라 득점 판정과 물리에 관여하지 않고, 득점에 실제로
> 관여하는 것은 골라인 위치와 포스트 안쪽 간격(small 1.8 m)인데 이 둘은
> 공식값이 들어가 있다. 로봇 키가 43 cm 라 크로스바에 닿을 일도 없다.
>
> 정확한 값이 필요해지는 경우: 카메라로 골대를 인식시키는 비전 학습을 할 때.
> 그때는 룰북 본문에서 값을 찾아 `DIVISIONS` 의 `E` 만 고치면 된다.

### kubot26 은 small 디비전

2026 년부터 Humanoid League 와 Standard Platform League 가 **HSL 로 통합**되면서
체급이 KidSize/TeenSize/AdultSize → **small/mid/large** 로 재편되었다.
RoboCup 2026 (인천, 6/30~7/6) 이 통합 규칙의 첫 대회였다.

| 디비전 | 최대 신장 | 최대 중량 |
|---|---|---|
| **small** | 110 cm | 15 kg |
| mid | 125 cm | 25 kg |
| large | 무제한 | 무제한 |

kubot26 은 기립 43 cm / 5.5 kg → **small**.

---

## 4. 월드 구성

| 요소 | 형상 | 충돌 |
|---|---|---|
| 카펫 (필드 + 보더) | 박스 | O |
| 라인 전체 (터치/골/하프웨이/에어리어/센터서클/마크) | 얇은 흰 박스 | **X (시각 전용)** |
| 골포스트 2 + 크로스바 | 실린더 + 박스 | O |
| 골 뒷판 (그물 대용) | 반투명 박스 | X |
| 공 | 구 (반지름 0.065, 반발계수 0.6, 질량 55 g) | O |

**라인은 충돌이 없다.** 시각 전용이라 접촉 계산 비용이 0 이다.
센터서클은 72 개 짧은 세그먼트로 근사한다.

높이 기준: 카펫 윗면이 z = 0.02 이므로 로봇 기립 시 base_link 는 **0.4520**
(= 0.4315 + 0.02) 이다. 평지 월드(0.4315)와 값이 다르니 헷갈리지 말 것.

---

## 5. 로봇 배치 (포메이션)

`launch/soccer.launch.py` 의 `FORMATIONS` 에서 수정한다. 값은 `(x, y, yaw[rad])`.

```python
FORMATIONS = {
    'kickoff': [(-0.6, 0.0, 0.0), (-2.5,  1.2, 0.0), (-2.5, -1.2, 0.0)],
    'line':    [(-1.0, 1.0, 0.0), (-1.0,  0.0, 0.0), (-1.0, -1.0, 0.0)],
    'defense': [(-4.2, 0.0, 0.0), (-2.0,  1.5, 0.0), (-2.0, -1.5, 0.0)],
}
```

좌표계는 필드 중앙이 원점, x 가 골대 방향(길이), y 가 폭 방향이다.
기본 배치는 전부 자기 진영(x < 0).

포메이션을 추가하면 `formation:=이름` 으로 바로 쓸 수 있다.

---

## 6. 리셋

```bash
ros2 service call /reset_world std_srvs/srv/Empty        # 로봇·공 위치만 되돌림
ros2 service call /reset_simulation std_srvs/srv/Empty   # 시간까지 전부 리셋
```

Gazebo 화면에서는 `Ctrl+Shift+R` (위치만) / `Ctrl+R` (전체).

---

## 7. 파일 목록

```
src/kubot26_pkgs/
├── scripts/gen_field.py              # 필드 생성기  ← 수치는 여기서만 고친다
├── worlds/
│   ├── robocup_hsl_small.world       # 생성 결과 (직접 고치지 말 것)
│   ├── robocup_hsl_mid.world
│   ├── robocup_hsl_large.world
│   └── kubot26.world                 # 평지 (필드 없음, 단독 테스트용)
└── launch/
    ├── soccer.launch.py              # 축구장 + N대
    ├── multi_robot.launch.py         # 평지 + N대
    └── gazebo.launch.py              # 평지 + 1대
```
