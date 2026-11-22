"""Motion primitives: turn "pick X, place in Y" into joint-space trajectories."""
from __future__ import annotations
from dataclasses import dataclass, field
import numpy as np
from . import kinematics as K
from . import scene as S

HOVER_Z = 0.26        # TCP height while travelling with / without an object
APPROACH_Z = 0.14     # slow-down height above grasp point
RELEASE_CLEARANCE = 0.05   # object bottom above basket floor when released


@dataclass
class Waypoint:
    q: np.ndarray
    phase: str                    # human-readable phase for the GUI
    event: str | None = None      # "attach" | "detach" | None
    dwell: float = 0.3            # seconds to wait after arriving
    speed: float = 1.0            # rad/s peak joint speed


class Unreachable(Exception):
    pass


def _solve(xyz, q_prev, what):
    q = K.solve_down(xyz, q_prev, seeds=[K.HOME_Q])
    if q is None:
        raise Unreachable(f"{what} at {tuple(round(float(v), 2) for v in xyz)} is out of reach")
    return q


def home_q():
    q = K.solve_down((0.38, 0.0, 0.30), K.HOME_Q, seeds=[K.HOME_Q])
    return q if q is not None else K.HOME_Q.copy()


def plan_pick_place(obj: S.Obj, obj_xy, q_start, slot_index=0, target="basket", held=False):
    """Return a list of Waypoints. obj_xy is the object's current (x, y)."""
    wps: list[Waypoint] = []
    q = np.asarray(q_start, float)
    ox, oy = obj_xy
    top = obj.height
    # ---- pick
    q = _add(wps, q, (ox, oy, HOVER_Z), "Moving above " + obj.label, speed=1.2)
    q = _add(wps, q, (ox, oy, top + APPROACH_Z / 2), "Approaching " + obj.label, speed=0.8)
    q = _add(wps, q, (ox, oy, top + 0.004), "Touching " + obj.label, speed=0.35, dwell=0.5)
    wps.append(Waypoint(q, "Suction on: grasping " + obj.label, event="attach", dwell=0.5))
    q = _add(wps, q, (ox, oy, HOVER_Z), "Lifting " + obj.label, speed=0.6)
    # ---- place
    if target == "basket":
        px, py = S.slot_xy(slot_index)
        z_rel = S.basket_floor_top() + RELEASE_CLEARANCE + obj.height
    else:  # put back on the table, in the free front-centre area
        px, py = 0.40, 0.0
        z_rel = obj.height + 0.01
    q = _add(wps, q, (px, py, HOVER_Z + 0.04), "Carrying to " + target, speed=1.2)
    q = _add(wps, q, (px, py, z_rel), "Lowering over " + target, speed=0.5, dwell=0.4)
    wps.append(Waypoint(q, "Releasing " + obj.label, event="detach", dwell=0.7))
    q = _add(wps, q, (px, py, HOVER_Z + 0.04), "Retreating", speed=1.0)
    return wps


def _add(wps, q_prev, xyz, phase, speed=1.0, dwell=0.15):
    q = _solve(xyz, q_prev, phase)
    wps.append(Waypoint(q, phase, dwell=dwell, speed=speed))
    return q


def min_jerk(q0, q1, speed):
    """Yield (t, q) samples at 50 Hz for a minimum-jerk move."""
    d = float(np.max(np.abs(np.asarray(q1) - np.asarray(q0))))
    T = max(0.5, 1.875 * d / max(speed, 1e-3))
    n = int(T * 50)
    for i in range(1, n + 1):
        s = i / n
        k = 10 * s**3 - 15 * s**4 + 6 * s**5
        yield i / 50.0, np.asarray(q0) + (np.asarray(q1) - np.asarray(q0)) * k
