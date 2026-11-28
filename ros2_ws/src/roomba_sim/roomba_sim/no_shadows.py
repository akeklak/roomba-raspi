"""Turns off shadows in the simulated camera's picture.

A world may have lights that cast shadows (the kitchen's cooking_area light does). In the robot's
camera they rendered as dark smudges, which looked wrong, and the shadow pass was the most
expensive part of rendering the camera (see realtime_pacer.py). This asks Gazebo once, at
startup, to stop every light in the world from casting shadows, without changing the world file.
The web UI's 3D view draws no shadows either.

Runs next to Gazebo in the sim only (sim.launch.py); it talks to Gazebo directly, not over ROS.
"""

import re
import sys
import time


def log(text: str) -> None:
    print(f'[no_shadows] {text}', flush=True)


def main() -> None:
    del sys.argv[1:]  # launch may pass --ros-args; there is nothing to configure
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.empty_pb2 import Empty
    from gz.msgs10.light_pb2 import Light
    from gz.msgs10.scene_pb2 import Scene
    from gz.transport13 import Node

    node = Node()
    world = None
    while world is None:
        for service in node.service_list():
            m = re.fullmatch(r'/world/([^/]+)/scene/info', service)
            if m:
                world = m.group(1)
        if world is None:
            time.sleep(1.0)
    ok, scene = False, None
    while not ok:
        ok, scene = node.request(f'/world/{world}/scene/info', Empty(), Empty, Scene, 5000)
        if not ok:
            time.sleep(1.0)
    changed = []
    for light in scene.light:
        if not light.cast_shadows:
            continue
        # light_config replaces the whole light, so send it back as it is, shadows off.
        light.cast_shadows = False
        done, reply = node.request(f'/world/{world}/light_config', light, Light, Boolean, 5000)
        if done and reply.data:
            changed.append(light.name)
        else:
            log(f'could not turn off the shadows of light "{light.name}"')
    if changed:
        log(f'turned off shadows for {", ".join(changed)}')


if __name__ == '__main__':
    main()
