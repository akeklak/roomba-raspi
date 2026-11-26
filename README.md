# roomba-raspi

A new brain for an iRobot Roomba 675: a Raspberry Pi 4 on top runs its own mapping, localization
and navigation (ROS 2 Jazzy), with a web UI for the map, no-go zones and starting or stopping a clean,
plus cameras for video, obstacle spotting and a dirt map.

The Roomba itself only drives and reports its sensors over its serial port (the iRobot Open
Interface). Everything runs from the Roomba's own battery and charges on its dock.

## Plan

| Phase | What | Done when |
| --- | --- | --- |
| 1. Base and driving | Pi setup, Roomba driver, LD19 lidar, power from the Roomba | You drive it from a laptop and see scan, odometry, bumpers, cliffs and battery live |
| 2. Mapping and localization | slam_toolbox, Nav2 | You click a spot on the map and it drives there by itself |
| 3. Web UI and cleaning | Browser app, no-go zones, room coverage, docking | From your phone you start a clean, it covers a room in rows, skips no-go zones and docks |
| 4. Cameras | Video, obstacle detection, dirt heatmap | The UI shows video and the dirt map, and it avoids a cord on the floor |

## Layout

```
docs/hardware.md          parts, serial pinout, battery-tap wiring, mounting
docs/bringup.md           day-one bring-up checklist
scripts/setup_pi.sh       installs ROS 2 Jazzy + drivers on Ubuntu 24.04 and builds the workspace
scripts/99-roomba-raspi.rules   udev rules for /dev/roomba and /dev/ldlidar
systemd/                  optional service to start the robot at boot
ros2_ws/roomba.repos      third-party drivers (create_robot, libcreate, ldrobot-lidar-ros2)
ros2_ws/src/roomba_bringup
  launch/robot.launch.py  Roomba driver + lidar + robot model + power guard + Foxglove bridge
  config/                 driver, lidar and power-guard parameters
  urdf/roomba.urdf.xacro  robot model with lidar and camera frames
  roomba_bringup/power_guard.py   powers the Pi off before it drains the Roomba battery
```

## Quick start (on the Pi)

```bash
git clone https://github.com/akeklak/roomba-raspi.git ~/roomba-raspi
cd ~/roomba-raspi && ./scripts/setup_pi.sh
ros2 launch roomba_bringup robot.launch.py
```

See `docs/bringup.md` for the full first-time checklist.

## Topics (phase 1)

| Topic | Type | From |
| --- | --- | --- |
| `/cmd_vel` | geometry_msgs/Twist | you / Nav2 → Roomba |
| `/odom` | nav_msgs/Odometry | create_driver |
| `/scan` | sensor_msgs/LaserScan | LD19 lidar |
| `/bumper`, `/cliff` | create_msgs/Bumper, create_msgs/Cliff | create_driver |
| `/wheeldrop` | std_msgs/Empty | create_driver |
| `/battery/charge_ratio`, `/battery/charging_state` | std_msgs/Float32, create_msgs/ChargingState | create_driver |
| `/dock`, `/undock` | std_msgs/Empty | → create_driver (seek dock / leave dock) |

## Tests

The power-guard logic has plain pytest tests that run without ROS:

```bash
cd ros2_ws/src/roomba_bringup && python3 -m pytest test
```
