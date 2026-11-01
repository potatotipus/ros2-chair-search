#!/usr/bin/env python3
"""Room-specific wrapper for the previously tested chair-search behavior."""
import json
from pathlib import Path
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from irobot_create_msgs.msg import DockStatus

from search_chair import ChairSearch


class DemoSearch(ChairSearch):
    def __init__(self):
        super().__init__()
        self._status_pub = self.create_publisher(
            String, '/demo/task_status', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._dock_state = None
        self._dock_wait_start = time.monotonic()
        self._dock_subscription = self.create_subscription(
            DockStatus, '/dock_status', self.dock_status, qos_profile_sensor_data)
        self.announce('WAITING')

    def dock_status(self, message):
        self._dock_state = message.is_docked

    def announce(self, state):
        self._status_pub.publish(String(data=state))

    def tick(self):
        if self.state == 'waiting' and not self.stop_reason and not self.done:
            if self._dock_state is not False:
                if time.monotonic() - self._dock_wait_start > 30:
                    self.finish('ERROR: dock status missing or robot still docked. Run bash demo.sh undock.')
                return
        super().tick()

    def on_detection(self, message):
        # Do not report completion before receiving a current dock status.
        if self._dock_state is not False:
            return
        super().on_detection(message)

    def send(self, client, goal, state):
        super().send(client, goal, state)
        self.announce('MOVING' if state == 'navigating' else 'LOOKING AROUND')

    def completed(self, future):
        super().completed(future)
        if not self.done and self.state == 'verifying':
            self.announce('VERIFYING CHAIR')

    def request_stop(self, reason):
        if reason.startswith('FOUND'):
            self.announce('CHAIR DETECTED - STOPPING')
        super().request_stop(reason)

    def finish(self, message):
        super().finish(message)
        state = ('FOUND - TASK COMPLETE' if message.startswith('FOUND') else
                 'NOT FOUND' if message.startswith('NOT FOUND') else 'ERROR - CHECK TERMINAL')
        self.announce(state)
        output = Path(__file__).resolve().parent / 'runtime' / 'last_result.json'
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps({'state': state, 'message': message,
                                      'simulation_time': self.seconds()}, indent=2), encoding='utf-8')


def main():
    rclpy.init()
    node = DemoSearch()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        node.request_stop('Interrupted by user.')
        deadline = time.monotonic() + 5
        try:
            while rclpy.ok() and not node.done and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
        except KeyboardInterrupt:
            pass
        if not node.done:
            node.get_logger().warning('Action end unconfirmed; cancel the current goal in RViz.')
    finally:
        # Allow the final status to reach the still-running image overlay.
        for _ in range(3):
            if rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
