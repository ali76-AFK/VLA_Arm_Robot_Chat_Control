"""Single source of truth for the demo scene. Used by the SDF generator, the
mock backend, the planner and the skills. Base frame: robot base on the table
surface, x forward, y left, z up."""
from __future__ import annotations
from dataclasses import dataclass
import math

TABLE_TOP_Z = 0.70                 # world height of the table surface
BASE_WORLD = (0.0, 0.0, TABLE_TOP_Z)
TABLE_CENTER = (0.30, 0.0)         # base frame (x, y)
TABLE_SIZE = (1.60, 1.00, TABLE_TOP_Z)   # x, y, height (top at TABLE_TOP_Z)

BASKET_CENTER = (0.32, -0.40)
BASKET_INNER = 0.26
BASKET_WALL_T = 0.012
BASKET_WALL_H = 0.09
BASKET_FLOOR_T = 0.012
BASKET_SLOTS = [(-0.05, -0.05), (0.05, -0.05), (-0.05, 0.05), (0.05, 0.05)]

# camera (base frame position, looks towards -x and down)
CAM_POS = (1.25, 0.0, 0.95)
CAM_PITCH = 0.66
CAM_YAW = math.pi
CAM_FOV = 1.25
CAM_W, CAM_H = 480, 360


@dataclass(frozen=True)
class Obj:
    name: str            # id used everywhere
    label: str           # human name
    color: tuple         # rgb 0..1
    shape: str           # box | cylinder
    size: tuple          # box: (sx,sy,sz); cylinder: (radius, height, 0)
    pos: tuple           # initial (x, y) base frame
    mass: float = 0.08

    @property
    def height(self):
        return self.size[2] if self.shape == "box" else self.size[1]

    @property
    def half_h(self):
        return self.height / 2

    @property
    def color_name(self):
        return self.name.split("_")[0]


OBJECTS = [
    Obj("red_cube",       "red cube",       (0.85, 0.12, 0.12), "box",      (0.05, 0.05, 0.05), (0.45, -0.05)),
    Obj("blue_cube",      "blue cube",      (0.12, 0.30, 0.85), "box",      (0.05, 0.05, 0.05), (0.35, 0.22)),
    Obj("green_cylinder", "green cylinder", (0.15, 0.70, 0.20), "cylinder", (0.025, 0.07, 0.0), (0.58, 0.18)),
    Obj("yellow_box",     "yellow box",     (0.95, 0.80, 0.10), "box",      (0.08, 0.04, 0.04), (0.50, 0.38)),
]
OBJ_BY_NAME = {o.name: o for o in OBJECTS}


def initial_z(o: Obj) -> float:
    return o.half_h + 0.001


def basket_floor_top() -> float:
    return BASKET_FLOOR_T


def slot_xy(i: int):
    dx, dy = BASKET_SLOTS[i % len(BASKET_SLOTS)]
    return BASKET_CENTER[0] + dx, BASKET_CENTER[1] + dy


def in_basket(p) -> bool:
    h = BASKET_INNER / 2 + 0.01
    return abs(p[0] - BASKET_CENTER[0]) < h and abs(p[1] - BASKET_CENTER[1]) < h and p[2] < 0.2
