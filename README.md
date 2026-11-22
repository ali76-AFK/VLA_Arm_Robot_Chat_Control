# UR5 · VLM pick-and-place demo (Gazebo Harmonic, Docker)

A UR5 arm on a table with a basket and four objects (red cube, blue cube, green cylinder, yellow box).
You talk to it in a web GUI; a small vision-language model looks at the camera image and decides what to
do; a motion layer (IK + trajectories) executes it in Gazebo. Everything runs in Docker.

```
 browser :8080  ──chat/state──►  ┌──────────── sim container ─────────────┐        ┌── vlm container ──┐
 (camera, status, joints,        │ web GUI + agent ──► planner ───────────┼──HTTP─►│ llama.cpp server  │
  objects, chat, log)            │      │  IK / trajectories              │        │ SmolVLM-500M (CPU)│
                                 │      ▼                                 │        └───────────────────┘
 Gazebo window (X11, Arc GPU) ◄──│ Gazebo Harmonic: UR5, table, basket,   │
                                 │ objects, camera  (gz-transport)        │
                                 └────────────────────────────────────────┘
```

## Run it

```bash
make check      # FIRST: staged self-test of Gazebo inside the container (≈5 min the first time: image build)
make up         # Gazebo window + web GUI + small VLM   (needs ~3.5 GB free RAM)
# then open  http://localhost:8080
```

| command | what | RAM needed (estimate) |
|---|---|---|
| `make up` | Gazebo + web GUI + SmolVLM-500M | ~3.5 GB free |
| `make up-lite` | Gazebo + web GUI, rule-based planner (no VLM) | ~2.5 GB |
| `make up-mock` | **no Gazebo**: GUI + planner on a built-in kinematic mock | ~0.5 GB |
| `make up-software` | like `up`, forces CPU rendering (if the Gazebo window is black) | +CPU load |
| `make test` | Python tests, no Docker needed | – |
| `make down` / `make clean` | stop / remove everything incl. model cache | |

`GZ_GUI=0 make up` skips the Gazebo window (the web GUI still shows the simulated camera) and saves ~1 GB RAM.

**Your laptop (from the output you sent):** Core Ultra 5 125H, Arc iGPU (Mesa 25.2, OpenGL 4.6), Ubuntu 24.04 on
Wayland, 15 GB RAM. When you ran the commands only ~1.2 GB was free and swap was full — **close the browser/heavy apps
before `make up`**; `scripts/preflight.sh` warns (and refuses) when free RAM is too low (`FORCE=1` overrides).
The Gazebo window runs through XWayland (`xhost +local:` is done for you).

## Example commands to type
`Put the red cube in the basket` · `Pick up the blue cube and the yellow box` · `Tidy up everything` ·
`What do you see?` · `Put it back on the table` · `Go home`. **Stop** aborts mid-motion (a held object is dropped).

## How the "VLA" part works (honest description)
This is a **VLM planner + scripted skills**, not an end-to-end VLA (those need a GPU and robot-specific training).
1. Chat text + a 384×288 camera frame + the list of objects go to SmolVLM-500M.
2. The model must answer JSON constrained by a schema (`action`, `object ∈ known ids`, `target`, `reply`), so it can
   only emit valid actions.
3. **Grounding guard:** if you explicitly name an object and the tiny model picks another one, your words win (shown in
   the log). Multi-object requests ("tidy up") are expanded by rules. If the VLM is down/slow, it falls back to the rule
   parser automatically (GUI pill turns red).
4. Skills: damped-least-squares IK (cup pointing down) → min-jerk joint trajectories → suction attach/release.
Object *positions* come from the simulator, not from pixels; the VLM chooses *what* to do.
Expect ~5–20 s per VLM answer on CPU (estimate). Switch planner live in the GUI (VLM / Rules).

Other models: `VLM_REPO=ggml-org/SmolVLM-256M-Instruct-GGUF make up` (lighter, dumber) or
`ggml-org/SmolVLM2-2.2B-Instruct-GGUF` (smarter, ~2.5 GB more RAM; not for your current free memory). Check the repo names exist on Hugging Face.

## What was tested, and what was NOT
Tested in my sandbox (no Docker/Gazebo/network there) — `make test`, 15 tests:
- UR5 FK matches the known zero-pose (0.8172 m), IK reaches all pick/place waypoints, cup points down, no link under the table, carries clear basket walls.
- **The generated Gazebo SDF encodes exactly the same robot as the IK solver** (link frames, joint axes, plugin topics checked by parsing it back) and is well-formed XML.
- Full pipeline on the mock backend: chat → plan → IK → motion → objects in basket; stop, reset, pronouns ("put *it* back"); HTTP API; GUI JavaScript executed against real state.
- VLM client against a fake llama.cpp server (image sent, schema sent, disagreement guard, fallback when down).
- `backends/gz.py` against a *stub* of the gz-transport Python API (catches our own bugs, not Gazebo's behaviour).

**Not tested (impossible here): the real Gazebo run, Docker build, GPU rendering, the real SmolVLM.**
`make check` tests these on your machine in stages, each printing PASS/FAIL/WARN:
`gz sdf -k` valid → Gazebo up & poses streaming → camera image → joint control tracks commands → full pick-and-place ends with the red cube in the basket.
Likeliest spots to need a tweak (and where to look):
| symptom | likely cause | fix |
|---|---|---|
| `make check` stage 2 fails | world not loading | read the printed Gazebo log tail; `gz sdf -k` output |
| stage 3 WARN (no camera) | headless EGL rendering issue | `make up-software`, or `GZ_GUI=1` (GUI context) |
| stage 4 WARN/FAIL | joint-state topic / controller gain | gains: `KP` in `sim/gen_world.py` |
| stage 5b FAIL (cube not in basket) | suction (DetachableJoint) behaviour/timing | see log lines in the GUI; tune dwell times in `skills.py` |
| black Gazebo window | Mesa/GPU path | `make up-software` |
| arm jitter | physics step / gains | lower `KP`, or `max_step_size` in `gen_world.py` |

## Notes & limitations
- Arm visuals are primitive shapes (cylinders/spheres) built from the **exact UR5 kinematic parameters**, not the CAD meshes
  (no ROS/mesh dependency, guaranteed consistent with the IK). Robot links have no collisions and gravity is off for them
  (so position control holds without tuning); paths are planned collision-free by construction.
- Grasping is a **suction cup** implemented with Gazebo's `DetachableJoint` (no finger physics).
- First start downloads: Gazebo image (~2–3 GB) and the VLM (~0.6 GB, cached in a Docker volume). Needs internet once.
- x86_64 only. Docker Compose ≥ 2.20.

## Layout
```
app/ur5vla/  kinematics.py scene.py skills.py planner.py agent.py server.py render.py selfcheck.py backends/{gz,mock}.py
app/static/index.html   web GUI          sim/gen_world.py   world+robot SDF generator (from scene.py/kinematics.py)
docker/sim.Dockerfile   docker-compose.yml   scripts/   tests/   Makefile
```
Change objects/positions in `app/ur5vla/scene.py` – SDF, mock renderer, planner and GUI all follow.
