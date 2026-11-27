#!/usr/bin/env bash
# Run the Gazebo simulation in Docker.
#   ./scripts/sim.sh up       build the images (first time ~10 min), start the sim and the web UI
#                             at http://localhost:8080
#   ./scripts/sim.sh ui       only the web UI (for the real robot: http://localhost:8080/?ws=ws://roomba.local:8765)
#   ./scripts/sim.sh teleop   drive with the keyboard (in a second terminal)
#   ./scripts/sim.sh shell    open a shell inside the running sim (ros2 topic list, etc.)
#   ./scripts/sim.sh down     stop it
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
COMPOSE=(docker compose -f docker/compose.yaml)

case "${1:-up}" in
  up)
    echo "Starting the sim. Open http://localhost:8080 in your browser once it's up (about 30 s)."
    "${COMPOSE[@]}" up --build
    ;;
  ui)
    echo "Web UI at http://localhost:8080 (add ?ws=ws://roomba.local:8765 for the real robot)."
    "${COMPOSE[@]}" up --build ui
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
    echo "usage: $0 {up|ui|teleop|shell|down}" >&2
    exit 1
    ;;
esac
