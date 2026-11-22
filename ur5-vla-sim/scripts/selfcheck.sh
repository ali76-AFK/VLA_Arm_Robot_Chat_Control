#!/usr/bin/env bash
# Runs inside the sim container: headless Gazebo + staged checks. `make check` calls this.
set -u
python3 /opt/ur5/sim/gen_world.py /tmp/demo.sdf
echo "== stage 0: gz sdf -k =="; gz sdf -k /tmp/demo.sdf && echo "PASS sdf valid" || echo "FAIL sdf invalid"
gz sim -s -r -v 2 --headless-rendering /tmp/demo.sdf > /tmp/gz-check.log 2>&1 &
GZ=$!
trap 'kill $GZ 2>/dev/null' EXIT
SIM_BACKEND=gazebo python3 -m ur5vla.selfcheck
rc=$?
if [ $rc -ne 0 ]; then echo; echo "---- last 40 lines of Gazebo log ----"; tail -40 /tmp/gz-check.log; fi
exit $rc
