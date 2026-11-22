"""Gazebo Harmonic backend using the gz-transport Python bindings.

Topics (all created by sim/gen_world.py):
  cmd   /ur5/cmd/<joint>                gz.msgs.Double   (JointPositionController)
  state /ur5/joint_states               gz.msgs.Model
  poses /world/demo/dynamic_pose/info   gz.msgs.Pose_V   (object poses)
  image /camera/image                   gz.msgs.Image
  grasp /ur5/attach|detach/<object>     gz.msgs.Empty    (DetachableJoint = suction)
  reset /world/demo/set_pose            service gz.msgs.Pose -> gz.msgs.Boolean
"""
from __future__ import annotations
import io, threading, time
import numpy as np
from PIL import Image as PILImage
from .base import Backend
from .. import scene as S, kinematics as K

WORLD = "demo"
BX, BY, BZ = S.BASE_WORLD


class GazeboBackend(Backend):
    name = "gazebo"

    def __init__(self, grasp_mode: str = "detachable"):
        from gz.transport13 import Node
        from gz.msgs10.double_pb2 import Double
        from gz.msgs10.empty_pb2 import Empty
        from gz.msgs10.image_pb2 import Image
        from gz.msgs10.model_pb2 import Model
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.msgs10.pose_pb2 import Pose
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.world_stats_pb2 import WorldStatistics
        self._m = dict(Double=Double, Empty=Empty, Pose=Pose, Boolean=Boolean)
        self.node = Node()
        self.lock = threading.Lock()
        self.grasp_mode = grasp_mode
        self._poses = {}
        self._q = None
        self._q_t = 0.0
        self._rtf = None
        self._img = None
        self._img_jpeg = None
        self._img_t = 0.0
        self._pose_t = 0.0
        self._held = None
        self.cmd_pub = {j: self.node.advertise(f"/ur5/cmd/{j}", Double) for j in K.JOINT_NAMES}
        self.att_pub = {o.name: self.node.advertise(f"/ur5/attach/{o.name}", Empty) for o in S.OBJECTS}
        self.det_pub = {o.name: self.node.advertise(f"/ur5/detach/{o.name}", Empty) for o in S.OBJECTS}
        self.node.subscribe(Pose_V, f"/world/{WORLD}/dynamic_pose/info", self._on_pose)
        self.node.subscribe(Model, "/ur5/joint_states", self._on_joint)
        self.node.subscribe(Model, f"/world/{WORLD}/model/ur5/joint_state", self._on_joint)
        self.node.subscribe(Image, "/camera/image", self._on_image)
        self.node.subscribe(WorldStatistics, f"/world/{WORLD}/stats", self._on_stats)

    # ---- callbacks (transport threads)
    def _on_pose(self, msg):
        names = {o.name for o in S.OBJECTS}
        with self.lock:
            for p in msg.pose:
                if p.name in names:
                    self._poses[p.name] = (p.position.x - BX, p.position.y - BY, p.position.z - BZ)
            self._pose_t = time.time()

    def _on_joint(self, msg):
        pos = {}
        for j in msg.joint:
            if j.name in K.JOINT_NAMES:
                pos[j.name] = j.axis1.position
        if len(pos) == 6:
            with self.lock:
                self._q = np.array([pos[n] for n in K.JOINT_NAMES])
                self._q_t = time.time()

    def _on_image(self, msg):
        try:
            if msg.width and msg.height and len(msg.data) >= msg.width * msg.height * 3:
                im = PILImage.frombytes("RGB", (msg.width, msg.height), bytes(msg.data)[: msg.width * msg.height * 3])
                with self.lock:
                    self._img = im
                    self._img_jpeg = None
                    self._img_t = time.time()
        except Exception:
            pass

    def _on_stats(self, msg):
        with self.lock:
            self._rtf = float(msg.real_time_factor)

    # ---- Backend API
    def ready(self):
        with self.lock:
            return len(self._poses) == len(S.OBJECTS)

    def status(self):
        with self.lock:
            if len(self._poses) < len(S.OBJECTS):
                return "waiting for Gazebo"
            if self._img is None:
                return "ok (no camera image yet)"
        return "ok"

    def set_joints(self, q):
        Double = self._m["Double"]
        for name, v in zip(K.JOINT_NAMES, q):
            self.cmd_pub[name].publish(Double(data=float(v)))

    def get_joints(self):
        with self.lock:
            if self._q is not None and time.time() - self._q_t < 1.0:
                return self._q.copy()
        return None

    def object_poses(self):
        with self.lock:
            return dict(self._poses)

    def attach(self, name):
        self._held = name
        self.att_pub[name].publish(self._m["Empty"]())

    def detach(self, name, drop_xy=None):
        self._held = None
        self.det_pub[name].publish(self._m["Empty"]())

    def _set_pose(self, name, x, y, z):
        Pose = self._m["Pose"]
        req = Pose()
        req.name = name
        req.position.x, req.position.y, req.position.z = x + BX, y + BY, z + BZ
        req.orientation.w = 1.0
        ok, rep = self.node.request(f"/world/{WORLD}/set_pose", req, Pose, self._m["Boolean"], 1500)
        return ok

    def reset(self):
        for o in S.OBJECTS:
            self.det_pub[o.name].publish(self._m["Empty"]())
        time.sleep(0.2)
        for o in S.OBJECTS:
            self._set_pose(o.name, o.pos[0], o.pos[1], S.initial_z(o) + 0.01)

    def frame_jpeg(self):
        with self.lock:
            if self._img is None:
                return None
            if self._img_jpeg is None:
                buf = io.BytesIO()
                self._img.save(buf, "JPEG", quality=80)
                self._img_jpeg = buf.getvalue()
            return self._img_jpeg

    def sim_info(self):
        with self.lock:
            return {"rtf": self._rtf}
