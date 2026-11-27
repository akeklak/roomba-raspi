# Day-one bring-up

Work through these in order. Each step has a check; don't move on until it passes.
Commands marked **(Pi)** run on the Pi over SSH, **(laptop)** on your computer.

## 1. Flash the Pi

1. In Raspberry Pi Imager choose **Other general-purpose OS → Ubuntu → Ubuntu Server 24.04 LTS (64-bit)**.
2. In the Imager settings set the hostname (`roomba`), your username, your wifi network, and enable
   SSH with your public key (password login also works, but keys are needed for Claude to help over SSH).
3. Boot the Pi, then **(laptop)** `ssh <user>@roomba.local`.

Check: `lsb_release -d` prints Ubuntu 24.04.

## 2. Install everything

**(Pi)**
```bash
git clone https://github.com/akeklak/roomba-raspi.git ~/roomba-raspi
cd ~/roomba-raspi && ./scripts/setup_pi.sh
```
Log out and back in afterwards.

Check: `ros2 pkg list | grep -E 'create_driver|ldlidar_component|roomba_bringup'` lists all three.

## 3. Identify the USB adapters

Plug in **only the Roomba cable**, then **(Pi)**
`udevadm info -q property -n /dev/ttyUSB0 | grep -E 'ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT'`.
Repeat with only the lidar plugged in. Compare with `scripts/99-roomba-raspi.rules` and fix the IDs
if they differ (see the comments in that file).

Check: with both plugged in, `ls -l /dev/roomba /dev/ldlidar` shows both links.

## 4. Talk to the Roomba (Pi on its wall charger)

Keep the Pi on its normal USB-C wall charger for now, next to the Roomba, with the serial cable
plugged into the Roomba's mini-DIN port. The Roomba should be **off the dock** and awake (press Clean once
then again to stop it, so it doesn't drive off).

**(Pi)** `ros2 launch roomba_bringup robot.launch.py use_lidar:=false`

Check, in a second SSH session:
- `ros2 topic echo --once /battery/charge_ratio` prints a number between 0 and 1.
- `ros2 topic echo /bumper` changes when you press the bumper.
- `ros2 topic echo /mode` shows FULL mode.

Then drive it. **Lift the Roomba off the floor or give it lots of room first.**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Use `i`/`,` for forward/back and `j`/`l` to turn; start with low speeds (`z` reduces speed).

Check: the wheels turn the right way and `/odom` changes as it moves.

## 5. Lidar

**(Pi)** `ros2 launch roomba_bringup robot.launch.py`

Check:
- `ros2 topic hz /scan` shows about 10 Hz.
- **(laptop)** In the repo, run `./scripts/sim.sh ui` and open
  `http://localhost:8080/?ws=ws://roomba.local:8765`. You should see the robot with the scan
  around it. Wave a hand near the lidar to confirm. (Foxglove at `ws://roomba.local:8765` works too.)

## 6. Power from the Roomba

Follow "Power: the battery tap" in `docs/hardware.md`. Then:

Check:
1. Off the dock, the Pi boots from the Roomba battery alone.
2. **Stop the stack**, put the Roomba on the dock, and confirm it charges (the Clean light pulses,
   and after restarting the stack `ros2 topic echo --once /battery/charging_state` shows a charging state).
   Leave it docked for an hour with the Pi running, then check `/battery/charge_ratio` went **up**.
3. power_guard: off the dock, temporarily set `shutdown_charge_ratio: 0.95`, `warn_charge_ratio: 0.95`
   and `grace_seconds: 10.0` in `config/power_guard.yaml`, relaunch, and confirm that after 10 seconds
   it logs "would power off now". Restore the values afterwards.

Allow power_guard to power off the Pi without a password, then set `dry_run: false` in
`config/power_guard.yaml`:
```bash
echo "$USER ALL=(root) NOPASSWD: /usr/bin/systemctl poweroff" | sudo tee /etc/sudoers.d/roomba-power-guard
sudo chmod 440 /etc/sudoers.d/roomba-power-guard
```

## 7. Mount and measure

Mount the lidar on top (highest point, clear view), the camera at the front, and the Pi and converter
low and centred. Measure the lidar and camera positions and update
`ros2_ws/src/roomba_bringup/urdf/roomba.urdf.xacro`, then rebuild:
`cd ~/roomba-raspi/ros2_ws && colcon build --symlink-install`.

## 8. Start at boot (optional)

Edit `User=` and paths in `systemd/roomba-bringup.service` if needed, then follow the comments at the
top of that file.

Phase 1 is done when you can drive it from your laptop and see the scan, odometry, bumpers, cliff
sensors and battery level live, running entirely on Roomba power.
