#!/usr/bin/env python3
"""One-waypoint chair search using Nav2 and camera_detector.py JSON output."""
import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose, Spin
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


class ChairSearch(Node):
    def __init__(self):
        super().__init__('chair_search')
        self.declare_parameter('x', -3.8193917274475098)
        self.declare_parameter('y', 1.2262970209121704)
        self.declare_parameter('confidence', 0.5)
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.spin_client = ActionClient(self, Spin, '/spin')
        self.subscription = self.create_subscription(
            String, '/ai/detections', self.on_detection, 10)
        self.state = 'waiting'
        self._search_goal_handle = None
        self.pending = False
        self._cancel_requested = False
        self._verify_requested = False
        self._verification_samples = 0
        self._verification_hits = 0
        self._scan_remaining = 2.0 * math.pi
        self._scan_progress = 0.0
        self.stop_reason = None
        self.done = False
        self.last_stamp = None
        self.last_received = None
        self.votes = 0
        self.received = 0
        # ROS simulation time can remain zero until the first /clock message.
        self.started = None
        self.phase_started = None
        self.last_notice = 0.0
        self.timer = self.create_timer(0.5, self.tick)
        self.get_logger().info('Waiting for fresh AI detections and Nav2.')

    def seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_detection(self, message):
        if self.done or self.stop_reason:
            return
        try:
            payload = json.loads(message.data)
            stamp = payload['stamp']
            key = (int(stamp['sec']), int(stamp['nanosec']))
            age = self.seconds() - (key[0] + key[1] / 1e9)
            if age < -0.5 or age > 5.0:
                return
            if self.last_stamp is not None and key <= self.last_stamp:
                return
            detections = payload['detections']
            if not isinstance(detections, list):
                return
            found = any(d['label'] == 'chair' and
                        float(d['confidence']) >= self.get_parameter('confidence').value
                        for d in detections)
        except (ValueError, TypeError, KeyError):
            return
        if self.last_received is not None and self.seconds() - self.last_received > 5.0:
            self.votes = 0
        self.last_stamp = key
        self.last_received = self.seconds()
        self.received += 1
        if self.state == 'verifying':
            self._verification_samples += 1
            self._verification_hits += int(found)
            self.get_logger().info(
                f'Stationary confirmation: {self._verification_hits}/3 hits '
                f'in {self._verification_samples}/5 frames')
            if self._verification_hits >= 3:
                self.request_stop('FOUND: chair confirmed in at least 3 of 5 stationary frames.')
            elif self._verification_samples >= 5:
                self.get_logger().info('Chair not confirmed; resuming remaining turn.')
                self.start_scan()
            return
        self.votes = self.votes + 1 if found else 0
        if found:
            self.get_logger().info(f'Chair evidence: {self.votes}/3 frames')
        if self.votes >= 3:
            self.request_stop('FOUND: chair detected in 3 consecutive fresh frames.')
        elif found and self.state == 'scanning' and not self._verify_requested:
            self._verify_requested = True
            self.get_logger().info('Chair candidate detected; pausing turn for confirmation.')
            if self._search_goal_handle is not None:
                self.cancel()

    def tick(self):
        if self.done or self.stop_reason:
            return
        now = self.seconds()
        if self.started is None:
            if now <= 0:
                return
            self.started = now
            self.phase_started = now
        if self.state == 'waiting':
            if now - self.started > 30:
                self.finish('ERROR: not ready within 30 simulation seconds. '
                            + self.readiness(now))
            elif (self.received >= 3 and self.last_received is not None
                  and now - self.last_received <= 5
                  and self.nav.server_is_ready() and self.spin_client.server_is_ready()):
                goal = NavigateToPose.Goal()
                goal.pose.header.frame_id = 'map'
                goal.pose.header.stamp = self.get_clock().now().to_msg()
                goal.pose.pose.position.x = float(self.get_parameter('x').value)
                goal.pose.pose.position.y = float(self.get_parameter('y').value)
                goal.pose.pose.orientation.w = 1.0
                self.send(self.nav, goal, 'navigating')
            elif time.monotonic() - self.last_notice > 10:
                self.get_logger().info('Waiting: ' + self.readiness(now))
                self.last_notice = time.monotonic()
        elif self.last_received is None or now - self.last_received > 8:
            self.request_stop('ERROR: camera inference stream stopped; ending search.')
        elif now - self.phase_started > 180:
            self.request_stop('ERROR: action timed out after 180 simulation seconds.')

    def readiness(self, now):
        age = 'none' if self.last_received is None else f'{now - self.last_received:.1f}s'
        return (f'AI frames={self.received}, last frame age={age}, '
                f'navigate_to_pose ready={self.nav.server_is_ready()}, '
                f'spin ready={self.spin_client.server_is_ready()}')

    def send(self, client, goal, state):
        self.state = state
        self.phase_started = self.seconds()
        self.pending = True
        self._cancel_requested = False
        self.get_logger().info('Moving to observation point.' if state == 'navigating'
                               else 'At observation point; turning to look for chair.')
        client.send_goal_async(goal, feedback_callback=self.feedback).add_done_callback(self.accepted)

    def feedback(self, message):
        if self.state == 'scanning':
            self._scan_progress = abs(float(message.feedback.angular_distance_traveled))

    def start_scan(self):
        if self._scan_remaining <= 0.05:
            self.finish('NOT FOUND: completed one turn; chair not confirmed.')
            return
        self._verify_requested = False
        self._scan_progress = 0.0
        self.votes = 0
        goal = Spin.Goal()
        goal.target_yaw = self._scan_remaining
        goal.time_allowance.sec = 120
        self.send(self.spin_client, goal, 'scanning')

    def accepted(self, future):
        self.pending = False
        try:
            handle = future.result()
        except Exception as error:
            self.finish(f'ERROR: could not send goal: {error}')
            return
        if not handle.accepted:
            self.finish(self.stop_reason or 'ERROR: Nav2 rejected the goal.')
            return
        self._search_goal_handle = handle
        handle.get_result_async().add_done_callback(self.completed)
        if self.stop_reason or self._verify_requested:
            self.cancel()

    def completed(self, future):
        self._search_goal_handle = None
        try:
            response = future.result()
        except Exception as error:
            self.finish(f'ERROR: action result unavailable: {error}')
            return
        if self.stop_reason:
            if response.status in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_SUCCEEDED):
                self.finish(self.stop_reason + ' Nav2 action ended.')
            else:
                self.finish(self.stop_reason + f' Action ended with status {response.status}.')
        elif self.state == 'scanning' and self._verify_requested:
            if response.status not in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_SUCCEEDED):
                self.finish(f'ERROR: could not pause turn (status={response.status}).')
                return
            self._scan_remaining = (0.0 if response.status == GoalStatus.STATUS_SUCCEEDED
                                    else max(0.0, self._scan_remaining - self._scan_progress))
            self.state = 'verifying'
            self.phase_started = self.seconds()
            self._verification_samples = 0
            self._verification_hits = 0
            self.votes = 0
            self.get_logger().info('Turn ended; checking up to 5 new camera frames while stationary.')
        elif response.status != GoalStatus.STATUS_SUCCEEDED:
            self.finish(f'ERROR: {self.state} failed: '
                        f'{getattr(response.result, "error_msg", "")} '
                        f'(status={response.status}).')
        elif self.state == 'navigating':
            self.start_scan()
        else:
            self.finish('NOT FOUND: completed one turn at the observation point.')

    def request_stop(self, reason):
        if self.stop_reason:
            return
        self.stop_reason = reason
        if self._search_goal_handle is not None:
            self.get_logger().info('Requesting cancellation of current Nav2 action.')
            self.cancel()
        elif not self.pending:
            self.finish(reason + ' No search action is running.')

    def cancel(self):
        if self._cancel_requested:
            return
        self._cancel_requested = True
        self._search_goal_handle.cancel_goal_async().add_done_callback(self.cancellation_response)

    def cancellation_response(self, future):
        try:
            if not future.result().goals_canceling and self._search_goal_handle is not None:
                self.get_logger().warning('Cancellation not accepted; waiting for action result.')
        except Exception as error:
            self.get_logger().error(f'Cancellation failed: {error}')

    def finish(self, message):
        self.get_logger().info(message)
        self.done = True


def main():
    rclpy.init()
    node = ChairSearch()
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
            node.get_logger().warning('Exit before action end confirmed; cancel goal in RViz.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
