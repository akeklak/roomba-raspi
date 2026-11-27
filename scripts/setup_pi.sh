#!/usr/bin/env bash
# One-shot setup for a Raspberry Pi 4 running Ubuntu Server 24.04 (64-bit).
# Installs ROS 2 Jazzy, fetches the third-party drivers, builds the workspace and
# installs the USB device rules. Safe to re-run.
#
# Usage (on the Pi, from the repo root):   ./scripts/setup_pi.sh
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="$REPO_DIR/ros2_ws"
ROS_DISTRO=jazzy

log() { printf '\n\033[1;34m==> %s\033[0m\n' "$*"; }

. /etc/os-release
if [[ "${VERSION_CODENAME:-}" != "noble" ]]; then
  echo "This script expects Ubuntu 24.04 (noble); found ${PRETTY_NAME:-unknown}." >&2
  echo "Flash 'Ubuntu Server 24.04 LTS (64-bit)' with Raspberry Pi Imager first (see docs/bringup.md)." >&2
  exit 1
fi

log "Locale and base packages"
sudo apt-get update
sudo apt-get install -y locales software-properties-common curl git
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sudo add-apt-repository -y universe

if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  log "Adding the ROS 2 apt repository"
  ROS_APT_SOURCE_VERSION=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F '"tag_name"' | awk -F'"' '{print $4}')
  curl -fsSL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${VERSION_CODENAME}_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
fi

log "Installing ROS 2 ${ROS_DISTRO} (base, no GUI) and robot packages"
sudo apt-get update
sudo apt-get install -y \
  ros-${ROS_DISTRO}-ros-base \
  ros-dev-tools \
  ros-${ROS_DISTRO}-xacro \
  ros-${ROS_DISTRO}-robot-state-publisher \
  ros-${ROS_DISTRO}-teleop-twist-keyboard \
  ros-${ROS_DISTRO}-nav2-lifecycle-manager \
  ros-${ROS_DISTRO}-nav2-util \
  ros-${ROS_DISTRO}-diagnostic-updater \
  ros-${ROS_DISTRO}-slam-toolbox \
  python3-websockets \
  python3-pil \
  libudev-dev

# shellcheck disable=SC1090
source /opt/ros/${ROS_DISTRO}/setup.bash

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  log "Initialising rosdep"
  sudo rosdep init
fi
rosdep update

log "Fetching third-party drivers into ros2_ws/src/third_party"
mkdir -p "$WS_DIR/src"
vcs import --skip-existing "$WS_DIR/src" < "$WS_DIR/roomba.repos"
vcs pull "$WS_DIR/src/third_party" || true
git -C "$WS_DIR/src/third_party/ldrobot-lidar-ros2" submodule update --init --recursive

log "Installing remaining dependencies with rosdep"
# rviz2 is a desktop GUI; run it on your laptop instead of the Pi.
rosdep install --from-paths "$WS_DIR/src" --ignore-src -r -y --rosdistro ${ROS_DISTRO} --skip-keys "rviz2"

log "Building the workspace (takes 10 to 20 minutes on a Pi 4)"
# Two parallel jobs keeps a 2 GB or 4 GB Pi from running out of memory.
export MAKEFLAGS="-j2"
cd "$WS_DIR"
colcon build --symlink-install --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

log "Installing USB device rules (/dev/roomba, /dev/ldlidar)"
sudo cp "$REPO_DIR/scripts/99-roomba-raspi.rules" /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG dialout "$USER"

BASHRC_LINE="source $WS_DIR/install/setup.bash"
if ! grep -qF "$BASHRC_LINE" "$HOME/.bashrc"; then
  log "Adding ROS setup to ~/.bashrc"
  {
    echo ""
    echo "# Roomba Raspi"
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash"
    echo "$BASHRC_LINE"
  } >> "$HOME/.bashrc"
fi

log "Done. Log out and back in (for the dialout group), then follow docs/bringup.md."
