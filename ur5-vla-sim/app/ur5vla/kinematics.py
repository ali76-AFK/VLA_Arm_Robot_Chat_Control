"""UR5 kinematics (standard universal_robot / ur_description UR5 parameters).

All poses are expressed in the robot *base frame*: x forward, y left, z up,
origin at the centre of the robot base on the table surface.
The same joint table is used to generate the Gazebo SDF, so the simulated arm
and this solver can never disagree.
"""
from __future__ import annotations
import math
import numpy as np

PI = math.pi
HP = PI / 2

# name, origin xyz, origin rpy, axis (in child frame)   -- from the UR5 URDF
JOINTS = [
    ("shoulder_pan_joint",  (0.0,  0.0,      0.089159), (0.0, 0.0, 0.0), (0, 0, 1)),
    ("shoulder_lift_joint", (0.0,  0.13585,  0.0),      (0.0, HP,  0.0), (0, 1, 0)),
    ("elbow_joint",         (0.0, -0.1197,   0.425),    (0.0, 0.0, 0.0), (0, 1, 0)),
    ("wrist_1_joint",       (0.0,  0.0,      0.39225),  (0.0, HP,  0.0), (0, 1, 0)),
    ("wrist_2_joint",       (0.0,  0.093,    0.0),      (0.0, 0.0, 0.0), (0, 0, 1)),
    ("wrist_3_joint",       (0.0,  0.0,      0.09465),  (0.0, 0.0, 0.0), (0, 1, 0)),
]
JOINT_NAMES = [j[0] for j in JOINTS]
LINK_NAMES = ["base_link", "shoulder_link", "upper_arm_link", "forearm_link",
              "wrist_1_link", "wrist_2_link", "wrist_3_link"]
# flange (tool0) relative to wrist_3_link; tool z axis == wrist_3 rotation axis
TOOL0_XYZ = (0.0, 0.0823, 0.0)
TOOL0_RPY = (-HP, 0.0, 0.0)
CUP_LENGTH = 0.12          # suction cup: flange -> contact point along tool z
JOINT_LIMIT = 2 * PI
HOME_Q = np.array([0.0, -1.9, 1.9, -1.57, -1.57, 0.0])


def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                     [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
                     [-sp,   cp*sr,            cp*cr]])


def R_to_rpy(R):
    sp = -R[2, 0]
    p = math.asin(max(-1.0, min(1.0, sp)))
    if abs(sp) < 0.999999:
        return math.atan2(R[2, 1], R[2, 2]), p, math.atan2(R[1, 0], R[0, 0])
    return math.atan2(-R[1, 2], R[1, 1]), p, 0.0


def _T(xyz, R):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = xyz
    return T


def _axis_R(axis, q):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * (K @ K)


_ORIGINS = [_T(xyz, rpy_to_R(*rpy)) for _, xyz, rpy, _ in JOINTS]
_TOOL0 = _T(TOOL0_XYZ, rpy_to_R(*TOOL0_RPY))
_CUP = _T((0, 0, CUP_LENGTH), np.eye(3))


def fk_chain(q):
    """Return list of 4x4 frames: base, then each link frame, then tcp."""
    T = np.eye(4)
    frames = [T.copy()]
    for (name, _, _, axis), O, qi in zip(JOINTS, _ORIGINS, q):
        T = T @ O @ _T((0, 0, 0), _axis_R(axis, qi))
        frames.append(T.copy())
    frames.append(T @ _TOOL0 @ _CUP)
    return frames


def fk(q):
    return fk_chain(q)[-1]


def _rot_err(Rt, Rc):
    Re = Rt @ Rc.T
    c = (np.trace(Re) - 1) / 2
    ang = math.acos(max(-1.0, min(1.0, c)))
    if ang < 1e-9:
        return np.zeros(3)
    v = np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
    s = np.linalg.norm(v)
    return np.zeros(3) if s < 1e-12 else v / s * ang


def _jac(q, eps=1e-6):
    T0 = fk(q)
    J = np.zeros((6, 6))
    for i in range(6):
        dq = np.array(q, float)
        dq[i] += eps
        T1 = fk(dq)
        J[:3, i] = (T1[:3, 3] - T0[:3, 3]) / eps
        J[3:, i] = _rot_err(T1[:3, :3], T0[:3, :3]) / eps
    return J


def ik(T_target, q0, iters=200, damping=0.03, tol_p=3e-4, tol_r=2e-3):
    q = np.array(q0, float)
    for _ in range(iters):
        T = fk(q)
        e = np.concatenate([T_target[:3, 3] - T[:3, 3], _rot_err(T_target[:3, :3], T[:3, :3])])
        if np.linalg.norm(e[:3]) < tol_p and np.linalg.norm(e[3:]) < tol_r:
            return q
        J = _jac(q)
        dq = J.T @ np.linalg.solve(J @ J.T + (damping ** 2) * np.eye(6), e)
        n = np.max(np.abs(dq))
        if n > 0.3:
            dq *= 0.3 / n
        q = np.clip(q + dq, -JOINT_LIMIT, JOINT_LIMIT)
    return None


def down_pose(xyz, yaw=0.0):
    """TCP pose with the suction cup pointing straight down."""
    R = rpy_to_R(0, 0, yaw) @ np.diag([1.0, -1.0, -1.0])
    return _T(xyz, R)


_YAWS = [0.0, 0.4, -0.4, 0.8, -0.8, 1.2, -1.2, 1.57, -1.57, 2.0, -2.0, 2.6, -2.6, 3.14]


def solve_down(xyz, q_seed, seeds=()):
    """Joint angles putting the TCP at xyz pointing down, closest to q_seed
    (smooth motion). None if unreachable."""
    best, best_cost = None, 1e9
    q_seed = np.asarray(q_seed, float)
    starts = [q_seed] + [np.asarray(s, float) for s in seeds]
    for yaw in _YAWS:
        for s in starts:
            q = ik(down_pose(xyz, yaw), s)
            if q is None:
                continue
            d = (q - q_seed + PI) % (2 * PI) - PI   # keep joints within +-pi of seed
            q = q_seed + d
            if np.any(np.abs(q) > JOINT_LIMIT):
                continue
            fr = fk_chain(q)
            if min(f[2, 3] for f in fr[2:6]) < 0.02:   # elbow/wrist below table
                continue
            cost = float(np.sum(np.abs(q - q_seed)))
            if cost < best_cost:
                best, best_cost = q, cost
        if best is not None and best_cost < 2.0:
            break
    return best


def tcp_xyz(q):
    return fk(q)[:3, 3]
