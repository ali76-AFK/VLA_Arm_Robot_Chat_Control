"""Validate the generated SDF against the IK solver's kinematics."""
import os, sys, subprocess, tempfile, unittest, math
import numpy as np
from lxml import etree
ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, os.path.join(ROOT, "app"))
from ur5vla import kinematics as K, scene as S


def build():
    out = os.path.join(tempfile.mkdtemp(), "demo.sdf")
    subprocess.check_call([sys.executable, os.path.join(ROOT, "sim", "gen_world.py"), out], stdout=subprocess.DEVNULL)
    return etree.parse(out).getroot()


def parse_pose(el):
    if el is None:
        return np.eye(4), None
    v = [float(x) for x in el.text.split()]
    T = np.eye(4); T[:3, :3] = K.rpy_to_R(*v[3:6]); T[:3, 3] = v[:3]
    return T, el.get("relative_to")


class TestSDF(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root = build()
        cls.world = cls.root.find("world")
        cls.robot = [m for m in cls.world.findall("model") if m.get("name") == "ur5"][0]

    def test_unique_names(self):
        names = [m.get("name") for m in self.world.findall("model")]
        self.assertEqual(len(names), len(set(names)))
        for need in ["table", "basket", "ur5", "overview_cam"] + [o.name for o in S.OBJECTS]:
            self.assertIn(need, names)

    def test_link_frames_match_fk(self):
        """Compose link poses via relative_to chain; must equal fk_chain(q=0)."""
        links = {l.get("name"): parse_pose(l.find("pose")) for l in self.robot.findall("link")}
        world = {}
        def resolve(n):
            if n in world: return world[n]
            T, rel = links[n]
            world[n] = (resolve(rel) @ T) if rel else T
            return world[n]
        fr = K.fk_chain(np.zeros(6))
        for i, n in enumerate(K.LINK_NAMES):
            self.assertTrue(np.allclose(resolve(n), fr[i], atol=1e-5), n)

    def test_joints_consistent(self):
        links = {l.get("name") for l in self.robot.findall("link")}
        joints = {j.get("name"): j for j in self.robot.findall("joint")}
        for i, (jn, _, _, axis) in enumerate(K.JOINTS):
            j = joints[jn]
            self.assertEqual(j.find("parent").text, K.LINK_NAMES[i])
            self.assertEqual(j.find("child").text, K.LINK_NAMES[i + 1])
            self.assertEqual([float(x) for x in j.find("axis/xyz").text.split()], [float(a) for a in axis])
            self.assertIn(j.find("child").text, links)

    def test_plugins(self):
        plug = [(p.get("name"), p) for p in self.robot.findall("plugin")]
        self.assertEqual(sum(n.endswith("JointPositionController") for n, _ in plug), 6)
        det = [p for n, p in plug if n.endswith("DetachableJoint")]
        self.assertEqual(len(det), len(S.OBJECTS))
        model_names = {m.get("name") for m in self.world.findall("model")}
        for p in det:
            self.assertIn(p.find("child_model").text, model_names)
        topics = {p.find("topic").text for n, p in plug if n.endswith("JointPositionController")}
        self.assertEqual(topics, {f"/ur5/cmd/{j}" for j in K.JOINT_NAMES})

    def test_objects_on_table(self):
        for o in S.OBJECTS:
            m = [m for m in self.world.findall("model") if m.get("name") == o.name][0]
            z = parse_pose(m.find("pose"))[0][2, 3]
            self.assertAlmostEqual(z, S.TABLE_TOP_Z + o.half_h + 0.001, places=4)

    def test_camera_and_systems(self):
        self.assertIsNotNone(self.world.xpath("//sensor[@type='camera']/topic[text()='/camera/image']") or None)
        files = {p.get("filename") for p in self.world.findall("plugin")}
        for f in ["gz-sim-physics-system", "gz-sim-user-commands-system", "gz-sim-scene-broadcaster-system", "gz-sim-sensors-system"]:
            self.assertIn(f, files)


if __name__ == "__main__":
    unittest.main()
