"""ROS 2 node that powers the Pi off before it drains the Roomba battery.

Subscribes to create_driver's battery topics and runs PowerPolicy once a second.
"""

import subprocess

import rclpy
from create_msgs.msg import ChargingState
from rclpy.node import Node
from std_msgs.msg import Float32

from roomba_bringup.power_policy import Action, PowerPolicy


class PowerGuard(Node):

    def __init__(self) -> None:
        super().__init__('power_guard')
        self.policy = PowerPolicy(
            shutdown_charge_ratio=self.declare_parameter('shutdown_charge_ratio', 0.12).value,
            warn_charge_ratio=self.declare_parameter('warn_charge_ratio', 0.25).value,
            grace_seconds=self.declare_parameter('grace_seconds', 120.0).value,
            stale_seconds=self.declare_parameter('stale_seconds', 0.0).value,
        )
        self.dry_run = self.declare_parameter('dry_run', True).value
        self.shutdown_command = list(self.declare_parameter(
            'shutdown_command', ['sudo', '-n', '/usr/bin/systemctl', 'poweroff']).value)

        self.create_subscription(Float32, 'battery/charge_ratio', self._on_ratio, 10)
        self.create_subscription(ChargingState, 'battery/charging_state', self._on_state, 10)
        self.create_timer(1.0, self._tick)
        self._last_action = Action.OK
        self._shutdown_sent = False
        self.get_logger().info(
            f'power_guard running (dry_run={self.dry_run}, '
            f'shutdown below {self.policy.shutdown_charge_ratio:.0%} '
            f'for {self.policy.grace_seconds:.0f} s off the dock)')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_ratio(self, msg: Float32) -> None:
        self.policy.on_charge_ratio(self._now(), msg.data)

    def _on_state(self, msg: ChargingState) -> None:
        self.policy.on_charging_state(self._now(), msg.state)

    def _tick(self) -> None:
        action = self.policy.evaluate(self._now())
        if action != self._last_action:
            self.get_logger().info(f'battery status: {action.value}')
        self._last_action = action

        if action == Action.WARN:
            self.get_logger().warn('Roomba battery low, dock soon', throttle_duration_sec=60.0)
        elif action == Action.SHUTDOWN and not self._shutdown_sent:
            self._shutdown_sent = True
            if self.dry_run:
                self.get_logger().error(
                    'Battery critically low: would power off now (dry_run is true, so not doing it)')
                return
            self.get_logger().error('Battery critically low: powering off the Pi')
            try:
                subprocess.run(self.shutdown_command, check=True, timeout=30)
            except (OSError, subprocess.SubprocessError) as exc:
                self.get_logger().error(f'Power-off command failed: {exc}')
                self._shutdown_sent = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PowerGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
