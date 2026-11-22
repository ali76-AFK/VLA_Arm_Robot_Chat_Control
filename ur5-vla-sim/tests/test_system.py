"""End-to-end tests: mock simulator + HTTP API + planner (incl. fake VLM server)."""
import json, os, sys, threading, time, unittest, urllib.request, urllib.error
from http.server import BaseHTTPRequestHandler, HTTPServer, ThreadingHTTPServer
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "app"))
from ur5vla import scene as S, kinematics as K, skills as SK
from ur5vla.agent import Agent
from ur5vla.planner import Planner, VLMClient, rule_plan
from ur5vla.backends.mock import MockBackend
from ur5vla.server import make_handler

POSES = {o.name: (o.pos[0], o.pos[1], S.initial_z(o)) for o in S.OBJECTS}


def wait_idle(a, timeout=60):
    t = time.time()
    while time.time() - t < timeout:
        if a.ready and not a.busy and a.q.empty():
            return True
        time.sleep(0.1)
    return False


class FakeVLM(BaseHTTPRequestHandler):
    reply = {"action": "pick_place", "object": "blue_cube", "target": "basket", "reply": "Moving the blue cube."}
    seen = []
    def log_message(self, *a): pass
    def do_GET(self):
        self.send_response(200); self.send_header("Content-Length", "2"); self.end_headers(); self.wfile.write(b"ok")
    def do_POST(self):
        body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
        FakeVLM.seen.append(body)
        out = json.dumps({"choices": [{"message": {"content": json.dumps(FakeVLM.reply)}}]}).encode()
        self.send_response(200); self.send_header("Content-Length", str(len(out))); self.end_headers(); self.wfile.write(out)


class TestKinematicsSkills(unittest.TestCase):
    def test_all_picks_reachable_and_safe(self):
        q = SK.home_q()
        for o in S.OBJECTS:
            for slot in range(4):
                cur = q
                for w in SK.plan_pick_place(o, o.pos, q, slot):
                    self.assertLess(float(np.max(np.abs(w.q - cur))), 2.2)
                    for _, qq in SK.min_jerk(cur, w.q, w.speed):
                        fr = K.fk_chain(qq)
                        self.assertGreater(min(f[2, 3] for f in fr[2:7]), 0.03)   # links above table
                    cur = w.q
                    tcp = K.fk(w.q)
                    self.assertTrue(np.allclose(tcp[:3, 2], [0, 0, -1], atol=0.01))  # cup points down

    def test_unreachable(self):
        far = S.Obj("x", "x", (1, 1, 1), "box", (.05, .05, .05), (1.5, 0))
        with self.assertRaises(SK.Unreachable):
            SK.plan_pick_place(far, far.pos, SK.home_q())


class TestPlanner(unittest.TestCase):
    def test_rules(self):
        f = lambda t: [(s.action, s.object, s.target) for s in rule_plan(t, POSES).steps]
        self.assertEqual(f("put the red cube in the basket"), [("pick_place", "red_cube", "basket")])
        self.assertEqual(len(f("tidy up")), 4)
        self.assertEqual(f("go home"), [("home", None, "basket")])
        self.assertEqual(f("hello")[0][0], "say")

    def test_vlm_used_and_guarded(self):
        srv = HTTPServer(("127.0.0.1", 0), FakeVLM); threading.Thread(target=srv.serve_forever, daemon=True).start()
        pl = Planner("vlm", VLMClient(f"http://127.0.0.1:{srv.server_port}"))
        FakeVLM.reply = {"action": "pick_place", "object": "blue_cube", "target": "basket", "reply": "ok"}
        p = pl.plan("grab the thing on the right", POSES, b"\xff\xd8jpeg")
        self.assertEqual((p.source, p.steps[0].object), ("vlm", "blue_cube"))
        self.assertTrue(pl.vlm_ok)
        content = FakeVLM.seen[-1]["messages"][1]["content"]
        self.assertEqual(content[0]["type"], "image_url")                     # image really sent
        self.assertIn("enum", json.dumps(FakeVLM.seen[-1]["response_format"]))  # schema constrained
        p = pl.plan("put the red cube in the basket", POSES, None)            # VLM disagrees with explicit words
        self.assertEqual(p.steps[0].object, "red_cube")
        self.assertTrue(any("you named" in n for n in p.notes))
        FakeVLM.reply = {"action": "say", "object": "none", "target": "basket", "reply": "I see 4 objects."}
        p = pl.plan("what do you see", POSES, None)
        self.assertEqual(p.steps[0].action, "say")
        srv.shutdown()

    def test_vlm_down_falls_back(self):
        pl = Planner("vlm", VLMClient("http://127.0.0.1:9", timeout=1))
        p = pl.plan("pick up the green cylinder", POSES, None)
        self.assertEqual(p.steps[0].object, "green_cylinder")
        self.assertFalse(pl.vlm_ok)


class TestHTTP(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.agent = Agent(MockBackend(), Planner("rules", None), time_scale=30)
        cls.agent.start()
        cls.srv = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(cls.agent))
        threading.Thread(target=cls.srv.serve_forever, daemon=True).start()
        cls.base = f"http://127.0.0.1:{cls.srv.server_port}"
        assert wait_idle(cls.agent)

    @classmethod
    def tearDownClass(cls):
        cls.srv.shutdown()

    def get(self, p):
        return urllib.request.urlopen(self.base + p, timeout=10)

    def post(self, p, d):
        r = urllib.request.Request(self.base + p, json.dumps(d).encode(), {"Content-Type": "application/json"})
        return json.loads(urllib.request.urlopen(r, timeout=10).read())

    def state(self):
        return json.loads(self.get("/api/state").read())

    def test_page_and_frame(self):
        self.assertIn(b"UR5", self.get("/").read())
        r = self.get("/api/frame.jpg"); self.assertEqual(r.headers["Content-Type"], "image/jpeg")
        self.assertTrue(r.read(3) == b"\xff\xd8\xff")

    def test_chat_pick_place_stop_reset(self):
        self.post("/api/reset", {}); self.assertTrue(wait_idle(self.agent))
        self.post("/api/chat", {"text": "put the red cube and the blue cube in the basket"})
        time.sleep(0.3); self.assertTrue(wait_idle(self.agent))
        s = self.state()
        where = {o["name"]: o["where"] for o in s["objects"]}
        self.assertEqual((where["red_cube"], where["blue_cube"], where["green_cylinder"]), ("basket", "basket", "table"))
        self.assertEqual(s["in_basket"], 2)
        self.assertIn("Done", s["chat"][-1]["text"])
        # stop mid-motion (real-time speed)
        self.agent.ts = 1.0
        self.post("/api/chat", {"text": "take the green cylinder"}); time.sleep(1.5)
        self.assertTrue(self.state()["busy"])
        self.post("/api/stop", {}); self.assertTrue(wait_idle(self.agent, 20))
        self.agent.ts = 30
        # reset, then pronoun resolution
        self.post("/api/reset", {}); self.assertTrue(wait_idle(self.agent))
        self.assertEqual(self.state()["in_basket"], 0)
        self.post("/api/chat", {"text": "pick up the yellow box"}); time.sleep(0.3); wait_idle(self.agent)
        self.post("/api/chat", {"text": "put it back on the table"}); time.sleep(0.3); wait_idle(self.agent)
        self.assertEqual({o["name"]: o["where"] for o in self.state()["objects"]}["yellow_box"], "table")

    def test_bad_json(self):
        r = urllib.request.Request(self.base + "/api/chat", b"{nope", {"Content-Type": "application/json"})
        with self.assertRaises(urllib.error.HTTPError) as c:
            urllib.request.urlopen(r)
        self.assertEqual(c.exception.code, 400)


if __name__ == "__main__":
    unittest.main(verbosity=2)
