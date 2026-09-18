#!/usr/bin/env python3
"""
visual STL -> 볼록 껍질(convex hull) 충돌 메쉬 생성.

원본 URDF 가 참조하던 *_collision.STL 이 존재하지 않아 만들어야 했다.
경계상자로 대체할 수도 있지만 형상이 너무 뭉개져서 볼록 껍질로 간다.
ODE / PhysX / MuJoCo 모두 볼록 형상을 선호하므로 세 엔진 모두 이득이다.

PhysX 는 볼록체 정점 256 개 상한이 있어 MAXV 로 여유 있게 줄인다.

사용:
    source ~/kubot_rl_venv/bin/activate
    python3 tools/gen_collision_meshes.py src/kubot26_pkgs
"""
import sys, os, glob
import numpy as np
import trimesh
from scipy.spatial import ConvexHull

MAXV = 200          # 볼록체 정점 상한 (PhysX 256 보다 여유)


def reduce_hull(mesh, maxv=MAXV):
    """볼록 껍질을 씌우고, 정점이 많으면 방향 샘플링으로 솎아 다시 껍질을 씌운다."""
    h = mesh.convex_hull
    V = h.vertices
    if len(V) <= maxv:
        return h
    rng = np.random.default_rng(0)
    dirs = rng.normal(size=(maxv * 3, 3))
    dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
    c = V.mean(0)
    # 각 방향에서 가장 먼 정점만 남기면 형상의 극점이 보존된다
    idx = np.unique(np.argmax((V - c) @ dirs.T, axis=0))
    if len(idx) > maxv:
        idx = idx[np.linspace(0, len(idx) - 1, maxv).astype(int)]
    pts = V[idx]
    hh = trimesh.Trimesh(vertices=pts, faces=ConvexHull(pts).simplices, process=True)
    hh.fix_normals()
    return hh


def main(pkg_dir):
    src = os.path.join(pkg_dir, 'meshes')
    dst = os.path.join(src, 'collision')
    os.makedirs(dst, exist_ok=True)

    print(f"{'link':22s}{'원본tri':>9s}{'정점':>6s}{'면':>7s}{'부피유지':>9s}")
    print("-" * 56)
    tot_o = tot_h = 0
    for p in sorted(glob.glob(os.path.join(src, '*.STL'))):
        name = os.path.basename(p)[:-4]
        m = trimesh.load(p)
        full = m.convex_hull
        red = reduce_hull(m)
        red.export(os.path.join(dst, f'{name}_collision.STL'))
        tot_o += len(m.faces)
        tot_h += len(red.faces)
        keep = red.volume / full.volume if full.volume > 0 else 1.0
        print(f"{name:22s}{len(m.faces):9d}{len(red.vertices):6d}{len(red.faces):7d}{keep:8.3f}")
    print("-" * 56)
    print(f"{'합계':22s}{tot_o:9d}{'':6s}{tot_h:7d}   ({tot_o/max(tot_h,1):.0f}배 감축)")


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'src/kubot26_pkgs')
