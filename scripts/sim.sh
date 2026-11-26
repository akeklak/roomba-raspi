#!/usr/bin/env bash
# Run the Gazebo simulation in Docker.
#   ./scripts/sim.sh up       build the image (first time ~10 min) and start the sim
#   ./scripts/sim.sh teleop   drive with the keyboard (in a second terminal)
#   ./scripts/sim.sh shell    open a shell inside the running sim (ros2 topic list, etc.)
#   ./scripts/sim.sh down     stop it
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
COMPOSE=(docker compose -f docker/compose.yaml)

case "${1:-up}" in
  up)
    "${COMPOSE[@]}" up --build sim
    ;;
  teleop)
    "${COMPOSE[@]}" exec sim bash -ic "ros2 run teleop_twist_keyboard teleop_twist_keyboard"
    ;;
  shell)
    "${COMPOSE[@]}" exec sim bash
    ;;
  down)
    "${COMPOSE[@]}" down
    ;;
  *)
    echo "usage: $0 {up|teleop|shell|down}" >&2
    exit 1
    ;;
esac
