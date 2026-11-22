#!/usr/bin/env bash
# Starts Gazebo (server headless-rendered + optional GUI) and the Python controller/web GUI.
set -u
BACKEND="${SIM_BACKEND:-gazebo}"
pids=()
cleanup() { for p in "${pids[@]}"; do kill "$p" 2>/dev/null; done; }
trap cleanup EXIT INT TERM

if [ "$BACKEND" = "gazebo" ]; then
  python3 /opt/ur5/sim/gen_world.py /tmp/demo.sdf
  echo "[entrypoint] validating world with 'gz sdf -k'"
  gz sdf -k /tmp/demo.sdf || echo "[entrypoint] WARNING: SDF validation reported problems (continuing)"
  echo "[entrypoint] starting Gazebo server (headless rendering for the camera)"
  gz sim -s -r -v "${GZ_VERBOSE:-2}" --headless-rendering /tmp/demo.sdf 2>&1 | sed -u 's/^/[gz] /' &
  pids+=($!)
  if [ "${GZ_GUI:-1}" = "1" ]; then
    if [ -z "${DISPLAY:-}" ]; then
      echo "[entrypoint] GZ_GUI=1 but no DISPLAY; skipping Gazebo window (web GUI still works)"
    else
      echo "[entrypoint] starting Gazebo GUI on DISPLAY=$DISPLAY"
      ( sleep 8; gz sim -g -v 2 2>&1 | sed -u 's/^/[gz-gui] /' ) &
      pids+=($!)
      ( sleep 20; sh /tmp/demo_guicam.sh >/dev/null 2>&1 || true ) &
    fi
  fi
fi

python3 -m ur5vla.server &
pids+=($!)
# exit (so docker shows the failure) as soon as either main process dies
wait -n
code=$?
echo "[entrypoint] a main process exited (code $code); shutting down"
exit "$code"
