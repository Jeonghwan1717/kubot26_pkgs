#!/usr/bin/env python3
"""
자기충돌 쌍이 '실제 기구 제약' 인지 '볼록 껍질 허상' 인지 판별한다.

볼록 껍질은 오목부를 메우므로 실제로는 안 닿는 링크끼리 닿는다고 나올 수 있다.
같은 자세에서 껍질과 원본 visual 메쉬를 각각 판정해 비교한다:

    껍질만 겹침  -> 허상. 물리엔진에서 제외해야 한다.
    둘 다 겹침   -> 원래 붙어있는 부품 (대개 부모-자식, 엔진이 자동으로 거름)
    둘 다 안 겹침 -> 문제 없음

사용:
    source ~/kubot_rl_venv/bin/activate
    python3 tools/check_self_collision.py src/kubot26_pkgs            # 전수 조사
    python3 tools/check_self_collision.py src/kubot26_pkgs sweep      # hip_roll 스윕

충돌 메쉬를 다시 만든 뒤에는 반드시 이걸 돌려 제외 목록을 갱신할 것.
갱신 대상: tools/urdf_to_mjcf.py 의 HULL_ARTIFACT_PAIRS
"""
import sys, os, itertools
import numpy as np
import trimesh
import xml.etree.ElementTree as ET


def load_model(pkg):
    urdf = os.path.join(pkg, 'urdf', 'kubot26_pkgs.urdf')
    r = ET.parse(urdf).getroot()
    J = {j.get('name'): j for j in r.findall('joint')}
    parent = {j.find('child').get('link'): j.find('parent').get('link') for j in r.findall('joint')}
    jof = {j.find('child').get('link'): j.get('name') for j in r.findall('joint')}
    links = [l.get('name') for l in r.findall('link')]
    return pkg, J, parent, jof, links


def rot_axis(a, th):
    a = np.array(a, float); a = a / np.linalg.norm(a)
    c, s = np.cos(th), np.sin(th)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + s * K + (1 - c) * K @ K


def link_frame(link, q, J, parent, jof):
    """base_link 기준 링크 프레임 (메쉬 좌표계)"""
    chain = []
    c = link
    while c != 'base_link':
        chain.append(jof[c]); c = parent[c]
    chain.reverse()
    R = np.eye(3); p = np.zeros(3)
    for nm in chain:
        d = np.array([float(v) for v in J[nm].find('origin').get('xyz').split()])
        p = p + R @ d
        ax = J[nm].find('axis')
        if ax is not None:
            R = R @ rot_axis([float(v) for v in ax.get('xyz').split()], q.get(nm, 0.0))
    return R, p


_cache = {}
def posed(pkg, link, q, hull, J, parent, jof):
    f = (os.path.join(pkg, 'meshes', 'collision', f'{link}_collision.STL') if hull
         else os.path.join(pkg, 'meshes', f'{link}.STL'))
    if f not in _cache:
        _cache[f] = trimesh.load(f)
    R, p = link_frame(link, q, J, parent, jof)
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = p
    m = _cache[f].copy(); m.apply_transform(T)
    return m


def pair_state(pkg, a, b, q, hull, J, parent, jof):
    cm = trimesh.collision.CollisionManager()
    cm.add_object('a', posed(pkg, a, q, hull, J, parent, jof))
    other = posed(pkg, b, q, hull, J, parent, jof)
    return cm.in_collision_single(other), cm.min_distance_single(other)


def survey(pkg, J, parent, jof, links):
    q = {}
    artifacts, real = [], []
    for a, b in itertools.combinations(links, 2):
        h, _ = pair_state(pkg, a, b, q, True, J, parent, jof)
        if not h:
            continue
        o, d = pair_state(pkg, a, b, q, False, J, parent, jof)
        (real if o else artifacts).append((a, b, d))

    def is_adjacent(a, b):
        return parent.get(a) == b or parent.get(b) == a

    print(f"[껍질 허상] 원본은 떨어져 있는데 껍질만 겹침 ({len(artifacts)}쌍)")
    need = []
    for a, b, d in artifacts:
        adj = is_adjacent(a, b)
        if not adj:
            need.append((a, b))
        print(f"   {a:18s} <-> {b:18s} 원본 {d*1000:5.1f}mm  "
              f"{'부모-자식(자동 제외)' if adj else '** 명시 제외 필요'}")
    print(f"\n[실제 접촉] 원본도 닿음 = 원래 붙어있는 부품 ({len(real)}쌍)")
    for a, b, _ in real:
        print(f"   {a:18s} <-> {b:18s} {'부모-자식' if is_adjacent(a,b) else '** 확인 요망'}")

    print("\n=== urdf_to_mjcf.py 의 HULL_ARTIFACT_PAIRS 에 넣을 목록 ===")
    for a, b in need:
        print(f"    ('{a}', '{b}'),")


def sweep(pkg, J, parent, jof):
    """hip_roll 을 키우며 좌우 정강이가 실제로 언제 닿는지"""
    D = np.pi / 180
    print(f"{'hip_roll':>9s}{'원본 메쉬':>14s}{'볼록 껍질':>14s}")
    print("-" * 40)
    for deg in (0, 5, 8, 10, 15, 20, 25):
        q = {'L_Hip_roll_joint': -deg * D, 'R_Hip_roll_joint': deg * D}
        out = []
        for hull in (False, True):
            h, d = pair_state(pkg, 'L_Knee_pitch', 'R_Knee_pitch', q, hull, J, parent, jof)
            out.append('겹침' if h else f'{d*1000:.1f}mm')
        print(f"{deg:8d}°{out[0]:>14s}{out[1]:>14s}")


if __name__ == '__main__':
    pkg = sys.argv[1] if len(sys.argv) > 1 else 'src/kubot26_pkgs'
    pkg, J, parent, jof, links = load_model(pkg)
    if len(sys.argv) > 2 and sys.argv[2] == 'sweep':
        sweep(pkg, J, parent, jof)
    else:
        survey(pkg, J, parent, jof, links)
