#!/usr/bin/env bash
# usage: run.sh full|lite|mock
set -e
cd "$(dirname "$0")/.."
mode="${1:-full}"
bash scripts/preflight.sh "$mode"
case "$mode" in
  full) docker compose --profile vlm up --build ;;
  lite) PLANNER=rules docker compose up --build ;;
  mock) SIM_BACKEND=mock PLANNER=rules GZ_GUI=0 docker compose up --build ;;
  *) echo "usage: $0 full|lite|mock"; exit 2 ;;
esac
