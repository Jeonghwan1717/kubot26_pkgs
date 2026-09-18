#!/usr/bin/env python3
"""
RoboCup Humanoid Soccer League (HSL) 축구장 SDF 생성기.

수치 출처: 공식 룰북 저장소 rules/field_diagram_macros.tex
  https://github.com/RoboCup-HumanoidSoccerLeague/HSL-Rules
  (2026-06-29 커밋 = RoboCup 2026 인천 대회 최종본)

2026 년부터 Humanoid League 와 Standard Platform League 가 HSL 로 통합되어
체급 구분이 KidSize/TeenSize/AdultSize -> small/mid/large 로 바뀌었다.

  디비전   최대 신장   최대 중량
  small    110 cm      15 kg
  mid      125 cm      25 kg
  large    무제한      무제한

kubot26 은 기립 높이 43 cm / 5.5 kg 이므로 small 디비전.

사용:
  python3 scripts/gen_field.py           > worlds/robocup_hsl_small.world
  python3 scripts/gen_field.py mid       > worlds/robocup_hsl_mid.world
  python3 scripts/gen_field.py large     > worlds/robocup_hsl_large.world
"""
import sys
import math

# ── HSL 디비전별 공식 규격 [m] (field_diagram_macros.tex) ──────────
DIVISIONS = {
    'small': dict(A=9.0,  B=6.0,  C=0.5, D=1.8, E=1.2,
                  F=1.0, G=3.0, H=1.5, I=1.5, J=1.0, K=2.0, L=4.0,
                  line_w=0.05, goal_line_w=0.10, corner_r=0.0),
    'mid':   dict(A=14.0, B=9.0,  C=0.7, D=2.4, E=1.5,
                  F=1.0, G=4.0, H=2.0, I=3.0, J=1.0, K=3.0, L=6.0,
                  line_w=0.05, goal_line_w=0.10, corner_r=0.5),
    'large': dict(A=22.0, B=14.0, C=0.6, D=2.4, E=1.8,
                  F=1.0, G=4.0, H=2.5, I=4.0, J=1.0, K=3.5, L=7.0,
                  line_w=0.12, goal_line_w=0.20, corner_r=1.0),
}
#  A 필드 길이 / B 필드 폭 / C 골 깊이 / D 골 폭 / E 골 높이
#  F 골에어리어 길이 / G 골에어리어 폭 / H 페널티마크 거리 / I 센터서클 지름
#  J 보더 / K 페널티에어리어 길이 / L 페널티에어리어 폭
#  [주의] E(골 높이)는 룰북 매크로에 없어 별도 확인 필요. 아래 값은 잠정치다.

DIVISION = sys.argv[1] if len(sys.argv) > 1 else 'small'
if DIVISION not in DIVISIONS:
    sys.exit(f"알 수 없는 디비전: {DIVISION} (small|mid|large)")
FIELD = DIVISIONS[DIVISION]

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
    glw = f['goal_line_w']          # 골라인은 다른 라인보다 두껍다
    for s in (+1,-1):
        parts.append(box(f"touch_{'p' if s>0 else 'n'}", A+glw, lw, LINE_T, 0,  s*B/2, LZ, WHITE))
        parts.append(box(f"goalline_{'p' if s>0 else 'n'}", glw, B+lw, LINE_T, s*A/2, 0, LZ, WHITE))
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
<!-- RoboCup Humanoid Soccer League ({DIVISION}) 축구장
     scripts/gen_field.py 로 생성. 수치 출처: 공식 룰북 field_diagram_macros.tex
     필드 {A} x {B} m, 보더 {J} m, 골 {D} x {E} m (깊이 {C} m) -->
<sdf version="1.6">
  <world name="robocup_hsl_{DIVISION}">

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
