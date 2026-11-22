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


## Example commands to type
`Put the red cube in the basket` · `Pick up the blue cube and the yellow box` · `Tidy up everything` ·
`What do you see?` · `Put it back on the table` · `Go home`. **Stop** aborts mid-motion (a held object is dropped).

## How the "VLA" part works
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



## Layout
```
app/ur5vla/  kinematics.py scene.py skills.py planner.py agent.py server.py render.py selfcheck.py backends/{gz,mock}.py
app/static/index.html   web GUI          sim/gen_world.py   world+robot SDF generator (from scene.py/kinematics.py)
docker/sim.Dockerfile   docker-compose.yml   scripts/   tests/   Makefile
```
Change objects/positions in `app/ur5vla/scene.py` – SDF, mock renderer, planner and GUI all follow.
