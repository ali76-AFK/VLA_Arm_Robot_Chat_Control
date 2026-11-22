#!/usr/bin/env bash
# usage: preflight.sh [full|lite|mock]
mode="${1:-full}"; fail=0
need_mb=3500; [ "$mode" = lite ] && need_mb=2500; [ "$mode" = mock ] && need_mb=500
avail=$(awk '/MemAvailable/ {printf "%d", $2/1024}' /proc/meminfo)
swapused=$(awk '/SwapTotal/ {t=$2} /SwapFree/ {f=$2} END {printf "%d", (t-f)/1024}' /proc/meminfo)
echo "RAM available: ${avail} MB (mode '$mode' wants ~${need_mb} MB free) | swap in use: ${swapused} MB"
if [ "$avail" -lt "$need_mb" ]; then
  echo "!! Not enough free RAM. Close your browser / heavy apps first, or use:  make up-lite  (no VLM)  or  make up-mock"
  [ "${FORCE:-0}" = 1 ] || fail=1
fi
command -v docker >/dev/null || { echo "!! docker not found"; fail=1; }
docker compose version >/dev/null 2>&1 || { echo "!! 'docker compose' plugin not found"; fail=1; }
docker info >/dev/null 2>&1 || { echo "!! cannot talk to docker daemon (is your user in the docker group?)"; fail=1; }
if [ "$mode" != mock ]; then
  [ -e /dev/dri/renderD128 ] || echo "!! /dev/dri/renderD128 missing: Gazebo will fall back to slow software rendering"
  if [ -n "${DISPLAY:-}" ] && command -v xhost >/dev/null; then
    xhost +local: >/dev/null 2>&1 && echo "X11: allowed local containers (xhost +local:)  [session: ${XDG_SESSION_TYPE:-?}, DISPLAY=$DISPLAY]" \
      || echo "!! xhost failed; Gazebo window may not open (web GUI still works)"
  else
    echo "!! no DISPLAY/xhost: Gazebo window disabled (web GUI still works). Install: sudo apt install x11-xserver-utils"
  fi
fi
exit $fail
