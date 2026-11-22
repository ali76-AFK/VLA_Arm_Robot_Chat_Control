"""Exercises backends/gz.py against a *stub* of the gz-transport/gz-msgs Python API
(shape taken from gz-transport13 / gz-msgs10). Catches logic/attribute bugs in our code;
it cannot prove the real Gazebo behaves the same -> use `make check` on your machine."""
import os, sys, types, unittest, time
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "app"))


class NS:  # attribute bag
    def __init__(self, **kw): self.__dict__.update(kw)


def install_stub():
    published, requests, subs = [], [], {}
    def msg(name, fields):
        def init(self, **kw):
            for k, v in fields().items(): setattr(self, k, v)
            for k, v in kw.items(): setattr(self, k, v)
        return type(name, (), {"__init__": init})
    Double = msg("Double", lambda: {"data": 0.0})
    Empty = msg("Empty", lambda: {})
    Boolean = msg("Boolean", lambda: {"data": True})
    Pose = msg("Pose", lambda: {"name": "", "position": NS(x=0, y=0, z=0), "orientation": NS(x=0, y=0, z=0, w=1)})
    Pose_V = msg("Pose_V", lambda: {"pose": []})
    Image = msg("Image", lambda: {"width": 0, "height": 0, "data": b""})
    Model = msg("Model", lambda: {"joint": []})
    WorldStatistics = msg("WorldStatistics", lambda: {"real_time_factor": 1.0})

    class Pub:
        def __init__(s, topic): s.topic = topic
        def publish(s, m): published.append((s.topic, m))

    class Node:
        def subscribe(s, cls, topic, cb): subs[topic] = cb; return True
        def advertise(s, topic, cls): return Pub(topic)
        def request(s, svc, req, rq, rp, timeout): requests.append((svc, req)); return True, Boolean()

    def mod(name, **attrs):
        m = types.ModuleType(name); m.__dict__.update(attrs); sys.modules[name] = m; return m
    mod("gz"); mod("gz.transport13", Node=Node); mod("gz.msgs10")
    for n, cls in [("double_pb2", Double), ("empty_pb2", Empty), ("image_pb2", Image), ("model_pb2", Model),
                   ("pose_v_pb2", Pose_V), ("pose_pb2", Pose), ("boolean_pb2", Boolean), ("world_stats_pb2", WorldStatistics)]:
        mod(f"gz.msgs10.{n}", **{cls.__name__: cls})
    return published, requests, subs, dict(Pose=Pose, Pose_V=Pose_V, Image=Image, Model=Model, WorldStatistics=WorldStatistics)


class TestGzBackend(unittest.TestCase):
    def test_flow(self):
        published, requests, subs, M = install_stub()
        from ur5vla.backends.gz import GazeboBackend
        from ur5vla import scene as S, kinematics as K
        be = GazeboBackend()
        self.assertFalse(be.ready())
        self.assertIn("waiting", be.status())
        # object poses (world frame -> base frame conversion)
        pv = M["Pose_V"](pose=[])
        for o in S.OBJECTS:
            p = M["Pose"](name=o.name); p.position.x, p.position.y, p.position.z = o.pos[0], o.pos[1], S.TABLE_TOP_Z + S.initial_z(o)
            pv.pose.append(p)
        pv.pose.append(M["Pose"](name="link"))      # link entries must be ignored
        subs["/world/demo/dynamic_pose/info"](pv)
        self.assertTrue(be.ready())
        po = be.object_poses()
        self.assertEqual(set(po), {o.name for o in S.OBJECTS})
        self.assertAlmostEqual(po["red_cube"][2], S.initial_z(S.OBJ_BY_NAME["red_cube"]), places=6)
        # joint state
        ms = M["Model"](joint=[NS(name=n, axis1=NS(position=0.1 * i)) for i, n in enumerate(K.JOINT_NAMES)])
        self.assertIsNone(be.get_joints())
        subs["/ur5/joint_states"](ms)
        self.assertTrue(np.allclose(be.get_joints(), [0.1 * i for i in range(6)]))
        # camera + jpeg
        self.assertIsNone(be.frame_jpeg())
        subs["/camera/image"](M["Image"](width=8, height=6, data=bytes(8 * 6 * 3)))
        self.assertEqual(be.frame_jpeg()[:3], b"\xff\xd8\xff")
        # stats
        subs["/world/demo/stats"](M["WorldStatistics"](real_time_factor=0.8))
        self.assertAlmostEqual(be.sim_info()["rtf"], 0.8)
        # commands go to the right topics
        published.clear(); be.set_joints(np.arange(6) * 0.2)
        self.assertEqual({t for t, _ in published}, {f"/ur5/cmd/{j}" for j in K.JOINT_NAMES})
        self.assertAlmostEqual(dict(published)["/ur5/cmd/elbow_joint"].data, 0.4)
        # grasp topics
        published.clear(); be.attach("red_cube"); be.detach("red_cube")
        self.assertEqual([t for t, _ in published], ["/ur5/attach/red_cube", "/ur5/detach/red_cube"])
        # reset -> detach all + set_pose service for every object, with base offset applied
        published.clear(); requests.clear(); be.reset()
        self.assertEqual(len(requests), len(S.OBJECTS))
        svc, req = requests[0]
        self.assertEqual(svc, "/world/demo/set_pose")
        self.assertAlmostEqual(req.position.z, S.TABLE_TOP_Z + S.initial_z(S.OBJECTS[0]) + 0.01, places=6)


if __name__ == "__main__":
    unittest.main()
