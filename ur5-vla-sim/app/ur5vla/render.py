"""Tiny software renderer (Pillow) used by the mock backend to produce a
camera view equivalent to the Gazebo camera. No GPU needed."""
from __future__ import annotations
import io, math
import numpy as np
from PIL import Image, ImageDraw
from . import scene as S
from . import kinematics as K

W, H = S.CAM_W, S.CAM_H
_f = (W / 2) / math.tan(S.CAM_FOV / 2)
_R = K.rpy_to_R(0, S.CAM_PITCH, S.CAM_YAW)          # columns: fwd, left, up
_C = np.array(S.CAM_POS)


def project(p):
    pc = _R.T @ (np.asarray(p, float) - _C)
    if pc[0] < 0.05:
        return None, 1e9
    return (W / 2 - _f * pc[1] / pc[0], H / 2 - _f * pc[2] / pc[0]), pc[0]


def _poly(d, pts, fill, outline=None):
    pr = [project(p)[0] for p in pts]
    if any(p is None for p in pr):
        return
    d.polygon(pr, fill=fill, outline=outline)


def _shade(c, k):
    return tuple(int(max(0, min(255, v * 255 * k))) for v in c)


def _box_faces(cx, cy, z0, sx, sy, z1):
    x0, x1, y0, y1 = cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2
    v = lambda x, y, z: (x, y, z)
    return [  # (corners, normal)
        ([v(x0, y0, z1), v(x1, y0, z1), v(x1, y1, z1), v(x0, y1, z1)], (0, 0, 1)),
        ([v(x1, y0, z0), v(x1, y1, z0), v(x1, y1, z1), v(x1, y0, z1)], (1, 0, 0)),
        ([v(x0, y0, z0), v(x0, y1, z0), v(x0, y1, z1), v(x0, y0, z1)], (-1, 0, 0)),
        ([v(x0, y1, z0), v(x1, y1, z0), v(x1, y1, z1), v(x0, y1, z1)], (0, 1, 0)),
        ([v(x0, y0, z0), v(x1, y0, z0), v(x1, y0, z1), v(x0, y0, z1)], (0, -1, 0)),
    ]


def _draw_box(d, cx, cy, z0, sx, sy, z1, color):
    shade = {(0, 0, 1): 1.0, (1, 0, 0): 0.85, (-1, 0, 0): 0.55, (0, 1, 0): 0.65, (0, -1, 0): 0.78}
    faces = _box_faces(cx, cy, z0, sx, sy, z1)
    for pts, n in faces:
        center = np.mean(pts, axis=0)
        if np.dot(np.array(n), _C - center) <= 0:      # back-face
            continue
        _poly(d, pts, _shade(color, shade[n]), (20, 20, 20))


def _draw_cyl(d, cx, cy, z0, r, z1, color):
    ang = np.linspace(0, 2 * math.pi, 28)
    ring = lambda z: [(cx + r * math.cos(a), cy + r * math.sin(a), z) for a in ang]
    pts = [project(p)[0] for p in ring(z0) + ring(z1)]
    if any(p is None for p in pts):
        return
    hull = _hull(pts)
    d.polygon(hull, fill=_shade(color, 0.75), outline=(20, 20, 20))
    top = [project(p)[0] for p in ring(z1)]
    d.polygon(top, fill=_shade(color, 1.0), outline=(20, 20, 20))


def _hull(points):
    pts = sorted(set((round(x, 1), round(y, 1)) for x, y in points))
    if len(pts) <= 2:
        return pts
    cross = lambda o, a, b: (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    lo, up = [], []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0: lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(up) >= 2 and cross(up[-2], up[-1], p) <= 0: up.pop()
        up.append(p)
    return lo[:-1] + up[:-1]


def _draw_obj(d, o, pos):
    x, y, zc = pos
    if o.shape == "box":
        _draw_box(d, x, y, zc - o.half_h, o.size[0], o.size[1], zc + o.half_h, o.color)
    else:
        _draw_cyl(d, x, y, zc - o.half_h, o.size[0], zc + o.half_h, o.color)


def render(q, poses: dict, held: str | None = None) -> bytes:
    img = Image.new("RGB", (W, H), (200, 205, 212))
    d = ImageDraw.Draw(img)
    # floor gradient
    for y in range(H):
        k = 0.78 + 0.22 * y / H
        d.line([(0, y), (W, y)], fill=(int(176 * k), int(182 * k), int(190 * k)))
    # table top + legs hint
    tx, ty, _ = S.TABLE_SIZE
    cx, cy = S.TABLE_CENTER
    _draw_box(d, cx, cy, -0.04, tx, ty, 0.0, (0.55, 0.40, 0.27))
    # robot
    frames = K.fk_chain(q)
    d.ellipse([0, 0, 0, 0])
    pts = [f[:3, 3] for f in frames]
    base_top = pts[1]
    _draw_cyl(d, 0, 0, 0, 0.07, base_top[2], (0.25, 0.28, 0.33))
    cols = [(0.18, 0.45, 0.80)] * 3 + [(0.65, 0.68, 0.72)] * 3
    for i in range(len(pts) - 1):
        a, b = project(pts[i])[0], project(pts[i + 1])[0]
        if a and b:
            w = [16, 15, 13, 10, 8, 7][min(i, 5)] if i < 6 else 6
            col = _shade(cols[min(i, 5)], 1.0)
            d.line([a, b], fill=(25, 25, 25), width=w + 3)
            d.line([a, b], fill=col, width=w)
            d.ellipse([a[0] - w / 2, a[1] - w / 2, a[0] + w / 2, a[1] + w / 2], fill=col, outline=(25, 25, 25))
    # basket
    bx, by = S.BASKET_CENTER
    i_, t, h = S.BASKET_INNER, S.BASKET_WALL_T, S.BASKET_WALL_H
    outer = i_ + 2 * t
    wc = (0.55, 0.38, 0.20)
    _draw_box(d, bx, by, 0, outer, outer, S.BASKET_FLOOR_T, wc)
    walls = [(bx, by - (i_ + t) / 2, outer, t), (bx, by + (i_ + t) / 2, outer, t),
             (bx - (i_ + t) / 2, by, t, i_), (bx + (i_ + t) / 2, by, t, i_)]
    walls.sort(key=lambda w: -np.linalg.norm(np.array([w[0], w[1], 0.05]) - _C))
    # draw walls + objects together, far to near
    items = [("wall", w, np.linalg.norm(np.array([w[0], w[1], 0.05]) - _C)) for w in walls]
    for o in S.OBJECTS:
        p = poses[o.name]
        items.append(("obj", o, np.linalg.norm(np.array(p) - _C) - (0.3 if o.name == held else 0)))
    items.sort(key=lambda it: -it[2])
    for kind, it, _ in items:
        if kind == "wall":
            _draw_box(d, it[0], it[1], S.BASKET_FLOOR_T, it[2], it[3], S.BASKET_FLOOR_T + h, wc)
        else:
            _draw_obj(d, it, poses[it.name])
    # suction cup marker
    tcp = pts[-1]
    a = project(tcp)[0]
    if a:
        d.ellipse([a[0] - 4, a[1] - 4, a[0] + 4, a[1] + 4], fill=(30, 30, 30))
    buf = io.BytesIO()
    img.save(buf, "JPEG", quality=82)
    return buf.getvalue()
