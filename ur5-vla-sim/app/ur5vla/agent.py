"""Executor + state for the GUI: chat -> plan -> waypoints -> simulator."""
from __future__ import annotations
import collections, io, queue, re, threading, time
import numpy as np
from PIL import Image
from . import kinematics as K, scene as S, skills as SK
from .planner import Planner, Plan, Step, describe_scene, mentioned_objects


class Aborted(Exception):
    pass


class Agent:
    def __init__(self, backend, planner: Planner, time_scale: float = 1.0):
        self.be = backend
        self.planner = planner
        self.ts = max(0.05, time_scale)
        self.lock = threading.RLock()
        self.q_cmd = np.zeros(6)
        self.status = "starting"
        self.phase = ""
        self.progress = {"step": 0, "of": 0}
        self.held = None
        self.last_obj = None
        self.chat = collections.deque(maxlen=60)
        self.log = collections.deque(maxlen=120)
        self.last_plan = {}
        self.q = queue.Queue()
        self.stop_flag = threading.Event()
        self.busy = False
        self.ready = False
        self._t = None

    # ---------------------------------------------------------------- public
    def start(self):
        self.be.start()
        self._t = threading.Thread(target=self._worker, daemon=True)
        self._t.start()

    def submit(self, text: str):
        text = text.strip()
        if not text:
            return
        self._say("user", text)
        self.q.put(("chat", text))

    def stop(self):
        self.stop_flag.set()
        self._log("Stop requested")

    def reset_scene(self):
        self.stop_flag.set()
        self.q.put(("reset", None))

    def set_planner_mode(self, mode):
        if mode in ("rules", "vlm"):
            self.planner.mode = mode
            self._log(f"Planner mode -> {mode}")

    def snapshot(self) -> dict:
        poses = self._poses()
        q = self.be.get_joints()
        qv = self.q_cmd if q is None else q
        tcp = K.fk(qv)[:3, 3]
        objs = []
        for o in S.OBJECTS:
            p = poses.get(o.name)
            where = "?" if p is None else ("held" if self.held == o.name else ("basket" if S.in_basket(p) else "table"))
            objs.append({"name": o.name, "label": o.label, "color": "#%02x%02x%02x" % tuple(int(c * 255) for c in o.color),
                         "pos": None if p is None else [round(float(v), 3) for v in p], "where": where})
        info = self.be.sim_info()
        return {
            "status": self.status, "phase": self.phase, "busy": self.busy, "ready": self.ready,
            "progress": self.progress,
            "backend": {"name": self.be.name, "status": self.be.status(), "rtf": info.get("rtf")},
            "planner": {"mode": self.planner.mode, "vlm_ok": self.planner.vlm_ok, "error": self.planner.last_error,
                        "last": self.last_plan},
            "joints_deg": [round(float(np.degrees(v)), 1) for v in qv],
            "joint_names": K.JOINT_NAMES,
            "tcp": [round(float(v), 3) for v in tcp],
            "gripper": f"holding {S.OBJ_BY_NAME[self.held].label}" if self.held else "empty",
            "objects": objs,
            "in_basket": sum(1 for o in objs if o["where"] == "basket"),
            "chat": list(self.chat), "log": list(self.log)[-40:],
        }

    # --------------------------------------------------------------- helpers
    def _say(self, role, text):
        self.chat.append({"role": role, "text": text, "t": time.strftime("%H:%M:%S")})

    def _log(self, text):
        self.log.append({"t": time.strftime("%H:%M:%S"), "text": text})

    def _poses(self):
        try:
            return self.be.object_poses()
        except Exception:
            return {}

    def _vlm_image(self):
        jpg = self.be.frame_jpeg()
        if not jpg:
            return None
        im = Image.open(io.BytesIO(jpg)).convert("RGB")
        im.thumbnail((384, 288))
        buf = io.BytesIO()
        im.save(buf, "JPEG", quality=85)
        return buf.getvalue()

    def _sleep(self, s):
        end = time.time() + s / self.ts
        while time.time() < end:
            if self.stop_flag.is_set():
                raise Aborted()
            time.sleep(0.02)

    # ---------------------------------------------------------------- worker
    def _worker(self):
        self.status = "waiting for simulator"
        while not self.be.ready():
            time.sleep(0.5)
        self._log(f"Simulator connected ({self.be.name})")
        try:
            self.status = "initialising"
            self.stop_flag.clear()
            self._goto_home()
            self.ready = True
            self.status = "idle"
            self._say("robot", "Ready. Tell me what to do, e.g. “put the red cube in the basket”.")
        except Exception as e:
            self._log(f"init error: {e}")
        while True:
            kind, payload = self.q.get()
            self.busy = True
            self.stop_flag.clear()
            try:
                if kind == "chat":
                    self._handle(payload)
                elif kind == "reset":
                    self._do_reset()
            except Aborted:
                self._log("Motion aborted")
                self._say("robot", "Stopped.")
                if self.held:
                    self.be.detach(self.held)
                    self._log(f"Released {self.held} (stop)")
                    self.held = None
            except SK.Unreachable as e:
                self._say("robot", f"I can't do that: {e}.")
                self._log(str(e))
            except Exception as e:  # keep the loop alive no matter what
                self._log(f"error: {type(e).__name__}: {e}")
                self._say("robot", "Something went wrong, see the log.")
            finally:
                self.busy = False
                self.phase = ""
                self.progress = {"step": 0, "of": 0}
                self.status = "idle"

    def _do_reset(self):
        self.status = "resetting"
        if self.held:
            self.be.detach(self.held)
            self.held = None
        self.be.reset()
        self._goto_home()
        self._say("robot", "Scene reset.")
        self._log("Scene reset")

    def _handle(self, text):
        poses = self._poses()
        # resolve "it/that" to the last handled object
        if self.last_obj and not mentioned_objects(text) and re.search(r"\b(it|that|this|same one)\b", text.lower()):
            text = re.sub(r"\b(it|that|this|same one)\b", S.OBJ_BY_NAME[self.last_obj].label, text, count=1, flags=re.I)
        self.status = "thinking" if self.planner.mode == "vlm" else "planning"
        plan: Plan = self.planner.plan(text, poses, self._vlm_image() if self.planner.mode == "vlm" else None)
        self.last_plan = {"source": plan.source, "latency": round(plan.latency, 2),
                          "steps": [{"action": s.action, "object": s.object, "target": s.target} for s in plan.steps],
                          "notes": plan.notes}
        for n in plan.notes:
            self._log("planner: " + n)
        self._log(f"Plan ({plan.source}, {plan.latency:.1f}s): " +
                  (", ".join(f"{s.action}:{s.object or '-'}→{s.target}" for s in plan.steps) or "no motion"))
        if plan.reply:
            self._say("robot", plan.reply)
        acts = [s for s in plan.steps if s.action != "say"]
        for i, st in enumerate(acts):
            self.progress = {"step": i + 1, "of": len(acts)}
            if st.action == "home":
                self.status = "moving"
                self._goto_home()
            elif st.action == "pick_place":
                self._pick_place(st)
        if any(s.action == "pick_place" for s in acts):
            self.status = "moving"
            self.phase = "Returning home"
            self._goto_home()
            self._say("robot", f"Done. {self.snapshot()['in_basket']} of {len(S.OBJECTS)} objects are in the basket.")

    # --------------------------------------------------------------- motion
    def _goto_home(self):
        self.phase = "Going home"
        self._move(SK.home_q(), 0.9)

    def _move(self, q1, speed):
        q0 = self.q_cmd.copy()
        for _, q in SK.min_jerk(q0, q1, speed):
            if self.stop_flag.is_set():
                raise Aborted()
            self.q_cmd = q
            self.be.set_joints(q)
            time.sleep(0.02 / self.ts)
        self.q_cmd = np.asarray(q1, float)
        self.be.set_joints(self.q_cmd)

    def _settle(self, dwell):
        """wait until the simulated arm reached the command (if we can see it)"""
        t_end = time.time() + 5.0
        while time.time() < t_end and self.be.get_joints() is not None:
            if self.stop_flag.is_set():
                raise Aborted()
            if np.max(np.abs(self.be.get_joints() - self.q_cmd)) < 0.015:
                break
            time.sleep(0.05)
        self._sleep(dwell)

    def _pick_place(self, st: Step):
        obj = S.OBJ_BY_NAME[st.object]
        poses = self._poses()
        if obj.name not in poses:
            raise SK.Unreachable(f"I can't see the {obj.label}")
        slot = sum(1 for o in S.OBJECTS if o.name in poses and S.in_basket(poses[o.name]))
        wps = SK.plan_pick_place(obj, poses[obj.name][:2], self.q_cmd, slot, st.target)
        self.status = "moving"
        self._log(f"{obj.label}: {len(wps)} waypoints")
        for w in wps:
            self.phase = w.phase
            self._move(w.q, w.speed)
            self._settle(w.dwell)
            if w.event == "attach":
                self.be.attach(obj.name)
                self.held = obj.name
                self._log(f"Suction on: {obj.label}")
            elif w.event == "detach":
                self.be.detach(obj.name)
                self.held = None
                self._log(f"Released: {obj.label}")
        self.last_obj = obj.name
        self.phase = "Checking result"
        self._sleep(1.0)
        p = self._poses().get(obj.name)
        ok = p is not None and (S.in_basket(p) == (st.target == "basket"))
        self._log(f"{obj.label} {'is in the ' + st.target if ok else 'did NOT end up in the ' + st.target}")
        if not ok:
            self._say("robot", f"The {obj.label} didn't end up in the {st.target}. Tell me to try again.")
