"""Mock backend: kinematic stand-in for Gazebo (no physics). Lets you try the
GUI / planner / chat without any simulator or GPU."""
from __future__ import annotations
import threading
import numpy as np
from .base import Backend
from .. import scene as S, kinematics as K, render


class MockBackend(Backend):
    name = "mock"

    def __init__(self):
        self.q = np.zeros(6)
        self.lock = threading.Lock()
        self.held = None
        self.pos = {}
        self.reset()

    def reset(self):
        with self.lock:
            self.held = None
            self.pos = {o.name: (o.pos[0], o.pos[1], S.initial_z(o)) for o in S.OBJECTS}
            self.q = np.zeros(6)

    def set_joints(self, q):
        with self.lock:
            self.q = np.array(q, float)

    def get_joints(self):
        with self.lock:
            return self.q.copy()

    def object_poses(self):
        with self.lock:
            out = dict(self.pos)
            if self.held:
                t = K.fk(self.q)[:3, 3]
                o = S.OBJ_BY_NAME[self.held]
                out[self.held] = (t[0], t[1], t[2] - o.half_h)
            return out

    def attach(self, name):
        with self.lock:
            self.held = name

    def detach(self, name, drop_xy=None):
        with self.lock:
            if self.held != name:
                return
            t = K.fk(self.q)[:3, 3]
            o = S.OBJ_BY_NAME[name]
            x, y = (t[0], t[1])
            floor = S.basket_floor_top() if S.in_basket((x, y, 0.0)) else 0.0
            self.pos[name] = (x, y, floor + o.half_h)
            self.held = None

    def frame_jpeg(self):
        with self.lock:
            q = self.q.copy()
            held = self.held
        return render.render(q, self.object_poses(), held)

    def sim_info(self):
        return {"rtf": 1.0}
