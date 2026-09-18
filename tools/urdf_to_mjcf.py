#!/usr/bin/env python3
"""
kubot26 URDF -> MuJoCo MJCF 변환기.

MuJoCo 는 URDF 를 읽을 수 있지만 그대로는 시뮬이 안 된다:
  * base_link 가 월드에 용접되어 자유 베이스가 없다 (dof 20, 26 이어야 함)
  * 액추에이터가 하나도 안 생긴다
  * 지면/조명이 없다
이 스크립트가 그 셋을 채워 넣는다.

사용:
    source ~/kubot_rl_venv/bin/activate
    python3 tools/urdf_to_mjcf.py \
        src/kubot26_pkgs/urdf/kubot26_pkgs.urdf  tools/kubot26_scene.xml

    python3 -m mujoco.viewer --mjcf=tools/kubot26_scene.xml     # 화면으로 보기
"""
import sys, os, shutil, tempfile
import xml.etree.ElementTree as ET
import mujoco

# 모터 스톨 토크 [Nm]
MX106, MX64, MX28 = 8.4, 6.0, 2.5
TORQUE = {}
for s in ('L', 'R'):
    for j in ('Hip_pitch', 'Hip_roll', 'Knee_pitch', 'Ankle_pitch', 'Ankle_roll'):
        TORQUE[f'{s}_{j}_joint'] = MX106
    TORQUE[f'{s}_Hip_yaw_joint']       = MX64
    TORQUE[f'{s}_Shoulder_roll_joint'] = MX64
    TORQUE[f'{s}_Elbow_pitch_joint']   = MX64
    TORQUE[f'{s}_Hand_pitch_joint']    = MX28
TORQUE['Neck_yaw_joint'] = MX28
TORQUE['Head_pitch_joint'] = MX28

# 위치 제어 kp. [주의] 정책을 이식할 때는 Isaac 학습에 쓴 stiffness 로 덮어쓸 것.
def kp_of(name):
    if 'Hip_yaw' in name: return 287
    if any(k in name for k in ('Hip_pitch', 'Hip_roll', 'Knee', 'Ankle')): return 500
    if 'Shoulder' in name: return 120
    if 'Elbow' in name: return 50
    return 30

# base_link 관성 (URDF 에서 읽어 넣는다. MuJoCo 변환 시 worldbody 로 흡수되어 사라진다)
def base_inertial(urdf_path):
    r = ET.parse(urdf_path).getroot()
    for l in r.findall('link'):
        if l.get('name') != 'base_link':
            continue
        ine = l.find('inertial')
        o = ine.find('origin').get('xyz')
        m = ine.find('mass').get('value')
        I = ine.find('inertia')
        full = " ".join(I.get(k) for k in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'))
        return {'pos': o, 'mass': m, 'fullinertia': full}
    raise SystemExit("base_link 를 찾지 못했습니다")


def main(urdf_in, mjcf_out, spawn_z=0.50, self_collision=False):
    # package:// 를 상대경로로 바꾼 임시 URDF 를 만든다 (MuJoCo 는 package:// 를 모른다)
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(urdf_in)))
    tmp = tempfile.mkdtemp()
    os.makedirs(f"{tmp}/urdf", exist_ok=True)
    shutil.copytree(f"{pkg_dir}/meshes", f"{tmp}/meshes", dirs_exist_ok=True)
    s = open(urdf_in, encoding='utf-8').read()
    s = s.replace('package://kubot26_pkgs/meshes/', '../meshes/')
    open(f"{tmp}/urdf/robot.urdf", 'w', encoding='utf-8').write(s)

    bi = base_inertial(urdf_in)

    m = mujoco.MjModel.from_xml_path(f"{tmp}/urdf/robot.urdf")
    raw = f"{tmp}/raw.xml"
    mujoco.mj_saveLastXML(raw, m)

    t = ET.parse(raw); r = t.getroot()
    wb = r.find('worldbody')

    # worldbody 직속 요소(= 용접된 base_link 의 내용물)를 body 로 감싼다
    loose = [c for c in list(wb) if c.tag in ('geom', 'body')]
    for c in loose:
        wb.remove(c)
    base = ET.Element('body', {'name': 'base_link', 'pos': f'0 0 {spawn_z}'})
    ET.SubElement(base, 'freejoint', {'name': 'root'})
    ET.SubElement(base, 'inertial', bi)
    for c in loose:
        base.append(c)
    wb.append(base)

    # 자기충돌 처리 — 기본은 끈다 (Gazebo 의 self_collide=false 와 맞추기 위함).
    #
    # 볼록 껍질은 오목한 부분을 메우므로 인접하지 않은 링크끼리도 겹친다.
    # 실측: Knee_pitch <-> Ankle_roll 이 자세와 무관하게 상시 25.2 mm 침투,
    #       base_link <-> Elbow_pitch 가 1.4~2.1 mm 침투.
    # MuJoCo 는 부모-자식 쌍만 자동으로 거르므로 조부모-손자 쌍이 그대로 남는다.
    # 켜두면 허위 접촉력이 계속 걸리고, 같은 로봇이 Gazebo 와 다르게 움직인다.
    #
    # contype/conaffinity 로 차단한다. 충돌 조건은
    #   (contype1 & conaffinity2) || (contype2 & conaffinity1)
    #   로봇-로봇 : (1&0)|(1&0) = 0  -> 충돌 안 함
    #   로봇-지면 : (1&1)|(1&0) = 1  -> 충돌함
    if not self_collision:
        for g in base.iter('geom'):
            g.set('contype', '1')
            g.set('conaffinity', '0')

    ET.SubElement(wb, 'geom', {'name': 'floor', 'type': 'plane', 'size': '20 20 0.1',
                               'rgba': '0.3 0.5 0.3 1', 'condim': '3',
                               'contype': '1', 'conaffinity': '1'})
    ET.SubElement(wb, 'light', {'pos': '0 0 3', 'dir': '0 0 -1', 'diffuse': '0.8 0.8 0.8'})

    act = ET.SubElement(r, 'actuator')
    n = 0
    for jnt in r.iter('joint'):
        name = jnt.get('name')
        if name in TORQUE:
            ET.SubElement(act, 'position', {
                'name': f'act_{name}', 'joint': name, 'kp': str(kp_of(name)),
                'forcerange': f'-{TORQUE[name]} {TORQUE[name]}'})
            n += 1

    # 메쉬 경로를 절대경로로 고정한다.
    # MuJoCo 가 저장한 MJCF 는 '../meshes/...' 상대경로를 쓰는데, 이는 출력 파일
    # 위치 기준이라 tools/ 에 두면 깨진다.
    mesh_root = os.path.join(pkg_dir, 'meshes')
    for meshel in r.iter('mesh'):
        f = meshel.get('file')
        if f:
            meshel.set('file', os.path.normpath(os.path.join(mesh_root, f.replace('../meshes/', ''))))

    ET.indent(t, space='  ')
    t.write(mjcf_out, encoding='utf-8', xml_declaration=True)
    shutil.rmtree(tmp, ignore_errors=True)

    chk = mujoco.MjModel.from_xml_path(mjcf_out)
    print(f"{mjcf_out} 생성")
    print(f"  body {chk.nbody}, dof {chk.nv} (관절 {chk.njnt - 1} + 자유베이스 6), actuator {chk.nu}")
    return chk


if __name__ == '__main__':
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    # 세 번째 인자로 'selfcol' 을 주면 자기충돌을 켠다
    main(sys.argv[1], sys.argv[2], self_collision=(len(sys.argv) > 3 and sys.argv[3] == 'selfcol'))
