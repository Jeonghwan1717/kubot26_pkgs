#!/usr/bin/env python3
"""
RoboCup Humanoid League KidSize 축구장 SDF 생성기.

수치는 아래 FIELD 딕셔너리만 고치면 된다. 연도별 룰북에 맞춰 대조할 것.
사용: python3 scripts/gen_field.py > worlds/robocup_kidsize.world
"""
import math

# ── KidSize 규격 [m] ─────────────────────────────────────────────
FIELD = dict(
    A = 9.0,     # 필드 길이 (라인 안쪽)
    B = 6.0,     # 필드 폭
    C = 0.6,     # 골 깊이
    D = 2.6,     # 골 폭 (골포스트 안쪽)
    E = 1.2,     # 골 높이
    F = 1.0,     # 골에어리어 길이
    G = 3.0,     # 골에어리어 폭
    H = 1.5,     # 페널티 마크 ~ 골라인 거리
    I = 1.5,     # 센터서클 지름
    J = 1.0,     # 외곽 여유(보더)
    K = 2.0,     # 페널티에어리어 길이
    L = 5.0,     # 페널티에어리어 폭
    line_w = 0.05,   # 라인 폭
)
LINE_T   = 0.005   # 라인 두께(높이)
POST_R   = 0.05    # 골포스트 반지름
BALL_R   = 0.065   # FIFA size 1 공 반지름
CARPET_T = 0.02    # 바닥 두께

def box(name, sx, sy, sz, x, y, z, rgba, collide=False):
    col = f"""
      <collision name="{name}_col">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
      </collision>""" if collide else ""
    return f"""
    <link name="{name}">
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
      <visual name="{name}_vis">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material>
      </visual>{col}
    </link>"""

def cyl(name, r, h, x, y, z, rgba, collide=True):
    col = f"""
      <collision name="{name}_col">
        <geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
      </collision>""" if collide else ""
    return f"""
    <link name="{name}">
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
      <visual name="{name}_vis">
        <geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
        <material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material>
      </visual>{col}"""+"""
    </link>"""

def main():
    f = FIELD
    A,B,C,D,E,F,G,H,I,J,K,L,lw = (f['A'],f['B'],f['C'],f['D'],f['E'],f['F'],
                                  f['G'],f['H'],f['I'],f['J'],f['K'],f['L'],f['line_w'])
    WHITE = "1 1 1 1"
    GREEN = "0.12 0.45 0.15 1"
    LZ = CARPET_T + LINE_T/2          # 라인 중심 높이 (카펫 위)
    parts = []

    # 카펫 (필드 + 보더)
    parts.append(box("carpet", A+2*J, B+2*J, CARPET_T, 0, 0, CARPET_T/2, GREEN, collide=True))

    # 터치라인(긴 변) / 골라인(짧은 변)
    for s in (+1,-1):
        parts.append(box(f"touch_{'p' if s>0 else 'n'}", A+lw, lw, LINE_T, 0,  s*B/2, LZ, WHITE))
        parts.append(box(f"goalline_{'p' if s>0 else 'n'}", lw, B+lw, LINE_T, s*A/2, 0, LZ, WHITE))
    # 하프웨이 라인
    parts.append(box("halfway", lw, B+lw, LINE_T, 0, 0, LZ, WHITE))

    # 골에어리어 / 페널티에어리어
    for s in (+1,-1):
        tag = 'p' if s>0 else 'n'
        for nm, length, width in (("goalarea", F, G), ("penarea", K, L)):
            xin = s*(A/2 - length)
            parts.append(box(f"{nm}_{tag}_front", lw, width+lw, LINE_T, xin, 0, LZ, WHITE))
            for t in (+1,-1):
                parts.append(box(f"{nm}_{tag}_side{'p' if t>0 else 'n'}",
                                 length, lw, LINE_T, s*(A/2)-s*length/2, t*width/2, LZ, WHITE))

    # 센터 서클 (짧은 호 세그먼트로 근사)
    N=72; r=I/2
    for k in range(N):
        a0=2*math.pi*k/N; a1=2*math.pi*(k+1)/N
        xm=r*math.cos((a0+a1)/2); ym=r*math.sin((a0+a1)/2)
        seg=2*r*math.sin((a1-a0)/2)+lw
        ang=(a0+a1)/2+math.pi/2
        parts.append(f"""
    <link name="circle_{k}">
      <pose>{xm:.4f} {ym:.4f} {LZ:.4f} 0 0 {ang:.5f}</pose>
      <visual name="circle_{k}_vis">
        <geometry><box><size>{lw} {seg:.4f} {LINE_T}</size></box></geometry>
        <material><ambient>{WHITE}</ambient><diffuse>{WHITE}</diffuse></material>
      </visual>
    </link>""")

    # 센터 마크 / 페널티 마크
    parts.append(box("center_mark", lw*2, lw*2, LINE_T, 0, 0, LZ, WHITE))
    for s in (+1,-1):
        parts.append(box(f"penmark_{'p' if s>0 else 'n'}", lw*2, lw*2, LINE_T,
                         s*(A/2 - H), 0, LZ, WHITE))

    # 골대 (포스트 2 + 크로스바), 골라인 바깥쪽으로 C 만큼
    for s in (+1,-1):
        tag='p' if s>0 else 'n'
        gx = s*(A/2)
        for t in (+1,-1):
            parts.append(cyl(f"post_{tag}_{'p' if t>0 else 'n'}", POST_R, E,
                             gx, t*(D/2+POST_R), CARPET_T+E/2, WHITE))
        parts.append(box(f"crossbar_{tag}", POST_R*2, D+4*POST_R, POST_R*2,
                         gx, 0, CARPET_T+E+POST_R, WHITE, collide=True))
        # 골 뒷그물 대용 (시각용 얇은 판)
        parts.append(box(f"net_{tag}", 0.02, D+4*POST_R, E,
                         gx + s*C, 0, CARPET_T+E/2, "0.85 0.85 0.85 0.35"))

    field = "".join(parts)
    print(f"""<?xml version="1.0" ?>
<!-- RoboCup Humanoid League KidSize 축구장
     scripts/gen_field.py 로 생성. 수치는 그 파일의 FIELD 딕셔너리에서 수정.
     필드 {A} x {B} m, 보더 {J} m, 골 {D} x {E} m (깊이 {C} m) -->
<sdf version="1.6">
  <world name="robocup_kidsize">

    <physics name="default_physics" default="0" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>100</iters>
          <sor>1.0</sor>
          <use_dynamic_moi_rescaling>true</use_dynamic_moi_rescaling>
        </solver>
      </ode>
    </physics>

    <include><uri>model://sun</uri></include>

    <model name="field">
      <static>true</static>{field}
    </model>

    <model name="ball">
      <pose>0 0 {CARPET_T+BALL_R:.4f} 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>0.055</mass>
          <inertia><ixx>9.3e-5</ixx><ixy>0</ixy><ixz>0</ixz>
                   <iyy>9.3e-5</iyy><iyz>0</iyz><izz>9.3e-5</izz></inertia>
        </inertial>
        <collision name="col">
          <geometry><sphere><radius>{BALL_R}</radius></sphere></geometry>
          <surface>
            <friction><ode><mu>0.6</mu><mu2>0.6</mu2></ode></friction>
            <bounce><restitution_coefficient>0.6</restitution_coefficient>
                    <threshold>0.01</threshold></bounce>
          </surface>
        </collision>
        <visual name="vis">
          <geometry><sphere><radius>{BALL_R}</radius></sphere></geometry>
          <material><ambient>1 0.5 0 1</ambient><diffuse>1 0.5 0 1</diffuse></material>
        </visual>
      </link>
    </model>

  </world>
</sdf>""")

if __name__ == "__main__":
    main()
