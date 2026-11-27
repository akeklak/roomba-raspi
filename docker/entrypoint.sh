#!/bin/bash
# Builds the simulation packages from the mounted repo ($REPO_DIR, default /repo), then runs the given command.
# Build output stays inside the container (/ws) so nothing is written into your checkout.
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash"

REPO_DIR="${REPO_DIR:-/repo}"
if [[ -d "$REPO_DIR/ros2_ws/src" ]]; then
  cd "$REPO_DIR/ros2_ws"
  colcon --log-base /ws/log build --symlink-install --packages-select roomba_web roomba_nav roomba_sim \
    --build-base /ws/build --install-base /ws/install \
    --event-handlers console_cohesion- summary- > /tmp/colcon.log 2>&1 \
    || { cat /tmp/colcon.log; exit 1; }
  source /ws/install/setup.bash
fi

# Make interactive shells (docker compose exec ... bash) see the workspace too.
grep -q "/ws/install/setup.bash" /root/.bashrc 2>/dev/null || \
  echo "source /opt/ros/${ROS_DISTRO}/setup.bash; [ -f /ws/install/setup.bash ] && source /ws/install/setup.bash" >> /root/.bashrc

exec "$@"
