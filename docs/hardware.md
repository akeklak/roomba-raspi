# Hardware

Everything rides on the Roomba 675 and runs from the Roomba's own battery. There is no
extra battery: the dock charges the Roomba, and the Roomba powers the Pi, lidar and camera.

```
                 Roomba 675
 ┌──────────────────────────────────────────────┐
 │  battery (+) ──[3 A fuse]──► 5V converter ───┼──USB-C──► Raspberry Pi 4
 │  battery (−) ───────────────►  (GND)         │              │  │  │
 │                                              │              │  │  └─CSI ribbon── fisheye camera
 │  mini-DIN serial port ◄──── USB cable ───────┼──────────────┘  │
 │                                              │                 └─USB── LD19 lidar adapter ── LD19
 └──────────────────────────────────────────────┘
```

## Parts

| Part | Notes |
| --- | --- |
| Raspberry Pi 4 + heatsink/fan case + 64 GB microSD | Runs Ubuntu Server 24.04 and ROS 2 Jazzy. |
| EZSync USB to mini-DIN cable (EZSync018) | Serial link to the Roomba's Open Interface. Data only. |
| LD19 lidar (Waveshare / LDROBOT D500 kit) | Kit includes the USB adapter board. 230400 baud. |
| Arducam 5MP OV5647 fisheye | One CSI camera (the Pi 4 has a single camera port). |
| 12/24 V to 5 V 5 A USB-C step-down converter | Input range must cover at least 10 to 24 V. |
| Inline blade fuse holder + 3 A fuses, 18 AWG wire, lever connectors | For the battery tap. |
| Multimeter | Check polarity and the 5 V output before connecting the Pi. |

## Serial port (Open Interface)

The 7-pin mini-DIN port is on the rear right of the robot under a snap-off cover. From iRobot's
[Create 2 / Roomba 600 Open Interface spec](https://cdn.hackaday.io/files/1835247851890816/2021-04-13_iRobot_Roomba_600_Open_Interface_Spec-3.pdf):

| Pin | Name | Function |
| --- | --- | --- |
| 1, 2 | Vpwr | Battery +, unregulated, **behind a 200 mA resettable fuse (500 mA peak)** |
| 3 | RXD | Serial into the Roomba, 0 to 5 V |
| 4 | TXD | Serial out of the Roomba, 0 to 5 V |
| 5 | BRC | Baud rate change; a low pulse also wakes a sleeping Roomba |
| 6, 7 | GND | Battery ground |

- 115200 baud, 8N1, no flow control. The driver uses the `CREATE_2` model for the 600 series.
- Vpwr can supply about 2.5 W, which is far too little for a Pi 4 (about 6 to 8 W with the lidar
  and camera). That is why the Pi is powered from a direct battery tap instead.

## Power: the battery tap

Goal: battery + and − go to the 5 V converter through a 3 A fuse, so the Pi is powered whenever
the battery is, and charges with the Roomba on the dock.

1. Do this with the Roomba **off the dock**. Flip it over and remove the battery cover.
   Take photos of what you see before changing anything (post them in the project thread and
   we'll pick the exact connection point together).
2. Identify battery + and − with the multimeter (about 14 to 17 V depending on charge).
3. Connect a red wire to + **through the inline fuse holder, with the fuse as close to the
   battery as possible**, and a black wire to −. Use lever connectors or solder and heat-shrink;
   nothing should be able to short against the chassis.
4. Route the wires out to the converter without pinching them under the battery cover.
5. Before plugging in the Pi, measure the converter's USB-C output: it must read 5.0 to 5.2 V.
6. Plug in the Pi. It should boot with the Roomba both on and off the dock.

Things to know:

- **The Pi drains the battery whenever the robot is off the dock**, even if the Roomba looks
  "off". `power_guard` (in `roomba_bringup`) powers the Pi off below 12% charge. Keep its
  `dry_run: true` default until you've seen it log correctly once, then turn it off.
- **The driver keeps the Roomba in Full mode, and the Roomba does not charge in Safe or Full
  mode.** When parking it on the dock, stop the stack (`sudo systemctl stop roomba-bringup` or
  Ctrl+C) or publish to `/dock`, which switches to Passive mode. Phase 3 replaces this with an
  automatic dock supervisor.
- Untested assumption: that the Roomba's charger copes with a constant ~0.5 A load while
  docked. We check this on day one (see docs/bringup.md, step 6). If it misbehaves, the fallback
  is a USB-C power bank in front of the Pi.

## Mounting

- The **lidar must be the highest thing** on the robot with a clear 360° view. Anything that
  blocks part of the view can be cropped out in `config/ldlidar.yaml`.
- Keep the Pi, converter and wiring low and centred so the robot stays balanced, and keep the
  top **Clean button** and the front bumper free to move.
- The camera goes at the front, tilted slightly down at the floor.
- After mounting, measure where the lidar and camera sit relative to the centre of the robot
  and put the numbers in `ros2_ws/src/roomba_bringup/urdf/roomba.urdf.xacro`.
- The extra height means the robot may no longer fit under some furniture.
