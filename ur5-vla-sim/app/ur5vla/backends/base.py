from __future__ import annotations
import numpy as np


class Backend:
    """Interface between the agent and a simulator. Positions are in the robot
    base frame (see scene.py)."""
    name = "base"

    def start(self): ...
    def stop(self): ...
    def ready(self) -> bool: return True
    def status(self) -> str: return "ok"
    def set_joints(self, q: np.ndarray): raise NotImplementedError
    def get_joints(self):               # actual joint angles or None
        return None
    def object_poses(self) -> dict:     # name -> (x, y, z)
        raise NotImplementedError
    def attach(self, name: str): raise NotImplementedError
    def detach(self, name: str, drop_xy=None): raise NotImplementedError
    def reset(self): raise NotImplementedError
    def frame_jpeg(self) -> bytes | None: return None
    def sim_info(self) -> dict: return {}
