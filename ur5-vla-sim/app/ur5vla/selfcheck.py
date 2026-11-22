"""Staged self-check of the live simulator (run via `make check`).
Each stage prints PASS/FAIL/WARN so a failure points at ONE specific thing."""
from __future__ import annotations
import os, sys, time
import numpy as np
from . import scene as S, kinematics as K, skills as SK
from .backends import make_backend
from .agent import Agent
from .planner import Planner

results = []


def stage(name, ok, detail="", warn=False):
    tag = "PASS" if ok else ("WARN" if warn else "FAIL")
    results.append((name, tag))
    print(f"[{tag}] {name}" + (f" — {detail}" if detail else ""), flush=True)
    return ok


def wait(cond, timeout):
    t = time.time()
    while time.time() - t < timeout:
        if cond():
            return True
        time.sleep(0.25)
    return False


def main():
    kind = os.environ.get("SIM_BACKEND", "gazebo")
    speed = float(os.environ.get("SIM_SPEED", "1"))
    print(f"== selfcheck, backend={kind} ==", flush=True)
    be = make_backend(kind)
    be.start()
    stage("1 python bindings + backend created", True)
    ok = wait(be.ready, 90)
    stage("2 Gazebo up, object poses streaming", ok, f"status={be.status()}")
    if not ok:
        return finish()
    ok = wait(lambda: be.frame_jpeg() is not None, 40)
    stage("3 camera image received", ok, "no image: GUI camera panel will be empty, sim itself still works" if not ok else "", warn=True)
    # 4: joint control
    q0 = SK.home_q()
    steps = 60
    cur = np.zeros(6)
    for i in range(1, steps + 1):
        be.set_joints(cur + (q0 - cur) * i / steps)
        time.sleep(0.03 / speed)
    time.sleep(3.0 / speed)
    q = be.get_joints()
    if q is None:
        stage("4 joint control (joint_states)", False, "no joint state received (topic name / plugin)", warn=True)
    else:
        err = float(np.max(np.abs(q - q0)))
        stage("4 joint control tracks commands", err < 0.08, f"max joint error {err:.3f} rad")
    # 5: full pick & place through the real agent
    ag = Agent(be, Planner("rules", None), time_scale=speed)
    ag.start()
    ok = wait(lambda: ag.ready and not ag.busy, 60)
    stage("5a agent initialised (arm at home)", ok)
    if ok:
        ag.submit("put the red cube in the basket")
        time.sleep(1)
        wait(lambda: not ag.busy and ag.q.empty(), 180)
        p = be.object_poses().get("red_cube")
        inb = p is not None and S.in_basket(p)
        stage("5b pick & place: red cube ends in basket", inb, f"red_cube at {None if p is None else tuple(round(float(v), 3) for v in p)}")
        if not inb:
            print("    last agent log:", *[l["text"] for l in list(ag.log)[-6:]], sep="\n      ")
    return finish()


def finish():
    bad = [r for r in results if r[1] == "FAIL"]
    print("\n==== SUMMARY ====")
    for n, t in results:
        print(f"  {t}  {n}")
    print("ALL CRITICAL STAGES PASSED" if not bad else f"{len(bad)} CRITICAL STAGE(S) FAILED")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
