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
docs/simulation.md        running the Gazebo simulation, and making maps and navigating in it
docker/                   simulation image (ROS 2 Jazzy + Gazebo Harmonic) and compose file
web/                      the 3D web UI (React + three.js), served at http://localhost:8080
scripts/sim.sh            up / teleop / shell / down for the simulation and web UI
scripts/setup_pi.sh       installs ROS 2 Jazzy + drivers on Ubuntu 24.04 and builds the workspace
scripts/99-roomba-raspi.rules   udev rules for /dev/roomba and /dev/ldlidar
systemd/                  optional service to start the robot at boot
ros2_ws/roomba.repos      third-party drivers (create_robot, libcreate, ldrobot-lidar-ros2)
ros2_ws/src/roomba_bringup
  launch/robot.launch.py  Roomba driver + lidar + robot model + power guard + web bridge + mapping and navigation
  config/                 driver, lidar and power-guard parameters
  urdf/roomba.urdf.xacro  robot model with lidar and camera frames
  roomba_bringup/power_guard.py   powers the Pi off before it drains the Roomba battery
ros2_ws/src/roomba_web    web bridge: ROS topics to the browser, and drive commands, goals and map buttons back
ros2_ws/src/roomba_nav    mapping (slam_toolbox), localization (AMCL) and navigation (Nav2)
  roomba_nav/map_manager.py  switches between mapping and navigating on the saved map; saves and loads maps
  config/                 slam_toolbox and Nav2 settings
ros2_ws/src/roomba_sim    Gazebo robot model, apartment world, ROS bridge, world markers for the UI, smoke test
```

## Simulation (no hardware needed)

```bash
./scripts/sim.sh up      # Gazebo + the web UI in Docker; open http://localhost:8080
```

See `docs/simulation.md`. The simulated robot publishes the same topics as the real one.

## Web UI

A full 3D view in the browser: the robot, the room around it (in the sim), the map, the lidar
scan and everything it has seen so far, the path driven, and the camera, both in a window and
projected in 3D in front of the robot. Drive with W A S D, the arrow keys or the on-screen pad,
or click **Go to** and a spot on the map and it drives there by itself. The Map panel makes,
saves and switches maps. It talks to ROS through our web bridge (`ros2_ws/src/roomba_web`) on
port 8765, which both the sim and the robot launch. If the page stops sending drive commands
(tab closed, wifi dropped), the bridge stops the robot. Driving by hand stops short of what the
lidar sees in the way, and stops when the robot is pushing against something it can't see.

To use it with the real robot, run just the UI on your laptop (`./scripts/sim.sh ui`) and open
`http://localhost:8080/?ws=ws://roomba.local:8765`.
To work on the UI itself: `cd web && npm install && npm run dev` (hot reload at port 8080).

## Quick start (on the Pi)

```bash
git clone https://github.com/akeklak/roomba-raspi.git ~/roomba-raspi
cd ~/roomba-raspi && ./scripts/setup_pi.sh
ros2 launch roomba_bringup robot.launch.py
```

See `docs/bringup.md` for the full first-time checklist, then step 9 there to map the apartment.

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

## Topics (phase 2)

| Name | Type | From |
| --- | --- | --- |
| `/map` | nav_msgs/OccupancyGrid | slam_toolbox while mapping, map_server on the saved map |
| `map → odom` (tf) | | slam_toolbox while mapping, AMCL on the saved map |
| `/map_manager/state` | std_msgs/String (JSON) | map_manager: mapping or navigating, busy, saved map |
| `/map_manager/save_map`, `use_saved_map`, `new_map`, `extend_map` | std_srvs/Trigger | → map_manager |
| `/navigate_to_pose` | nav2_msgs/NavigateToPose (action) | → Nav2: drive to a pose on the map |
| `/plan` | nav_msgs/Path | Nav2's planned route |
| `/global_costmap/costmap` | nav_msgs/OccupancyGrid | Nav2: obstacles grown by the robot's size |
| `/initialpose` | geometry_msgs/PoseWithCovarianceStamped | → AMCL: where the robot really is |

## Tests

Plain pytest tests that run without ROS (power guard, sim world, web bridge messages, saved maps):

```bash
for p in roomba_bringup roomba_sim roomba_web roomba_nav; do (cd ros2_ws/src/$p && python3 -m pytest -q test); done
cd web && npm test
```

With the sim running, `ros2 run roomba_sim smoke_test` and `ros2 run roomba_nav nav_test` check it
end to end (CI runs both; see `docs/simulation.md`).
