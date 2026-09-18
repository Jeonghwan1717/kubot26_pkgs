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

# 볼록 껍질이 오목부를 메우면서 생기는 '가짜' 접촉 쌍.
# 원본 visual 메쉬로는 10~13 mm 떨어져 있는데 껍질끼리는 겹친다.
# 부모-자식 쌍은 물리엔진이 자동으로 거르므로 조부모 관계만 여기 남긴다.
HULL_ARTIFACT_PAIRS = [
    ('base_link', 'L_Elbow_pitch'),      # 원본 10.2 mm 이격
    ('base_link', 'R_Elbow_pitch'),      # 원본 10.1 mm 이격
    ('L_Knee_pitch', 'L_Ankle_roll'),    # 원본 12.9 mm 이격
    ('R_Knee_pitch', 'R_Ankle_roll'),    # 원본 12.9 mm 이격
]

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


def main(urdf_in, mjcf_out, spawn_z=0.50):
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
    contact = ET.SubElement(r, 'contact')

    loose = [c for c in list(wb) if c.tag in ('geom', 'body')]
    for c in loose:
        wb.remove(c)
    base = ET.Element('body', {'name': 'base_link', 'pos': f'0 0 {spawn_z}'})
    ET.SubElement(base, 'freejoint', {'name': 'root'})
    ET.SubElement(base, 'inertial', bi)
    for c in loose:
        base.append(c)
    wb.append(base)

    # 자기충돌 — 켜둔다. 끄면 정책이 실기에서 불가능한 다리 교차 동작을 학습한다.
    #
    # kubot26 은 정강이 폭 55.8 mm 에 힙 간격이 117 mm 뿐이라 여유가 적다.
    # 원본 메쉬로 실측한 결과 hip_roll 이 8 도만 넘어도 좌우 정강이가 실제로 닿는다:
    #     0deg 61.6mm / 5deg 16.9mm / 8deg 이상 접촉
    # 볼록 껍질과 원본 메쉬가 모든 각도에서 같은 결과를 내므로, 다리끼리의
    # 충돌 판정에는 껍질을 그대로 써도 정확하다 (정강이 외곽 폭이 껍질과 동일).
    #
    # 다만 껍질이 오목부를 메우면서 '실제로는 안 닿는데 닿는다'고 나오는 쌍이 있다.
    # 전 링크 쌍을 원본 메쉬와 대조해 걸러낸 결과가 아래 목록이다.
    # (부모-자식 쌍은 엔진이 자동으로 거르므로 여기엔 조부모 관계만 남는다)
    for a, b in HULL_ARTIFACT_PAIRS:
        ET.SubElement(contact, 'exclude', {'body1': a, 'body2': b})

    ET.SubElement(wb, 'geom', {'name': 'floor', 'type': 'plane', 'size': '20 20 0.1',
                               'rgba': '0.3 0.5 0.3 1', 'condim': '3'})
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
    main(sys.argv[1], sys.argv[2])
