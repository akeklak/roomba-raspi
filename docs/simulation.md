# Simulation

A simulated Roomba with the LD19 lidar and front camera, in a small apartment, running in
Gazebo Harmonic inside Docker. It publishes the **same topics as the real robot** (`/cmd_vel`,
`/odom`, `/tf`, `/scan`, `/camera/image_raw`, `/camera/camera_info`), so mapping, navigation and
the web UI can be built and tested here first and then run unchanged on the Pi.

## One-time setup (Mac)

1. Install [Docker Desktop](https://www.docker.com/products/docker-desktop/) (or OrbStack).
   In Docker Desktop settings, give it at least **4 CPUs and 6 GB of memory**.
2. Install the [Foxglove](https://foxglove.dev/download) desktop app (or use app.foxglove.dev in Chrome).
3. Clone the repo: `git clone https://github.com/akeklak/roomba-raspi.git && cd roomba-raspi`

## Run it

```bash
./scripts/sim.sh up        # first run builds the image, about 10 minutes
```

Wait for the log to settle (about 30 seconds), then in Foxglove choose
**Open connection → Foxglove WebSocket → `ws://localhost:8765`**.

A useful layout:

- **3D panel**: set the display frame to `odom`. Turn on `/scan`, `/tf` and `/robot_description`.
- **Image panel**: `/camera/image_raw`.
- **Teleop panel**: topic `/cmd_vel`, to drive with on-screen arrows.

Or drive from the keyboard in a second terminal:

```bash
./scripts/sim.sh teleop    # i = forward, , = back, j / l = turn, k = stop
```

Other commands:

```bash
./scripts/sim.sh shell     # shell inside the sim: ros2 topic list, ros2 topic hz /scan, ...
./scripts/sim.sh down      # stop
```

Code changes under `ros2_ws/src/roomba_sim` are picked up on the next `./scripts/sim.sh up`.

## What's in the world

`ros2_ws/src/roomba_sim/worlds/apartment.sdf`: about 8 m × 6 m with a living room, a bedroom
and a kitchen joined by 0.9 m doorways. A coffee table and dining table whose tops sit above
the lidar, so it only sees their legs (as the real one will). A couch, bed, counter and dresser.
The dock is the green block on the west wall; the robot starts just east of it, facing east.

## How it maps to the real robot

| | Real robot | Simulation |
| --- | --- | --- |
| Drive | create_driver over serial | Gazebo DiffDrive plugin, same 0.5 m/s limit |
| Lidar | LD19, 455 beams, 10 Hz | gpu_lidar, 455 beams, 10 Hz, 1 cm noise |
| Camera | OV5647 fisheye | 640×480 wide-angle camera (~140°) |
| Frames | `odom → base_footprint → base_link → ldlidar_link`, `camera_link`, `camera_optical_link` | same |

Not simulated yet: bumpers, cliff sensors, battery, docking and the vacuum. Those come with
phase 3.

## Linux server instead

The same `./scripts/sim.sh up` works on any Linux machine with Docker. On an Ubuntu 24.04 desktop
with ROS 2 Jazzy installed natively you can also see the Gazebo window:
`ros2 launch roomba_sim sim.launch.py headless:=false`.

## Automated check

CI runs `ros2 run roomba_sim smoke_test` against the sim on every pull request: it checks that
the lidar returns 455 beams that hit walls, that the camera publishes, and that the robot moves
about 0.9 m when driven at 0.3 m/s for 3 seconds.
