#!/usr/bin/env python3
"""ROS 2 camera -> local YOLO inference -> annotated image and JSON detections.

This node observes images only; it does not command robot motion.
"""
import json
import os
from pathlib import Path
import time

import cv2
import rclpy
import torch
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO


class CameraDetector(Node):
    def __init__(self):
        super().__init__('camera_detector')
        self.declare_parameter('image_topic', '/oakd/rgb/preview/image_raw')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('model', 'yolo11s.pt')
        self.declare_parameter('confidence', 0.4)
        self.declare_parameter('image_size', 640)
        self.declare_parameter('snapshot_dir', str(Path(__file__).resolve().parent / 'runtime' / 'snapshots'))
        self._snapshot_dir = Path(str(self.get_parameter('snapshot_dir').value))
        self._snapshot_dir.mkdir(parents=True, exist_ok=True)
        self._last_snapshot = 0.0
        self.device = str(self.get_parameter('device').value)
        self.confidence = float(self.get_parameter('confidence').value)
        torch.set_num_threads(2)
        self.get_logger().info('Loading model; first run may download model weights.')
        self.model = YOLO(str(self.get_parameter('model').value))
        self.bridge = CvBridge()
        self.latest = None
        self.last_report = 0.0
        self._task_state = 'READY'
        self._status_subscription = self.create_subscription(
            String, '/demo/task_status', self.receive_status,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.started = time.monotonic()
        self.image_pub = self.create_publisher(Image, '/ai/detections/image', 1)
        self.json_pub = self.create_publisher(String, '/ai/detections', 1)
        topic = str(self.get_parameter('image_topic').value)
        self.subscription = self.create_subscription(
            Image, topic, self.receive, qos_profile_sensor_data)
        self.timer = self.create_timer(0.5, self.detect)
        self.get_logger().info(
            f'Listening: {topic}; device={self.device}; output=/ai/detections/image')

    def receive(self, message):
        # Keep just the newest image, so inference cannot build a frame backlog.
        self.latest = message

    def receive_status(self, message):
        self._task_state = message.data

    def detect(self):
        message = self.latest
        now = time.monotonic()
        if message is None:
            if now - max(self.started, self.last_report) >= 5.0:
                self.get_logger().warning('Waiting for camera frames. Check Gazebo is playing.')
                self.last_report = now
            return
        self.latest = None
        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        result = self.model.predict(
            source=frame, imgsz=int(self.get_parameter('image_size').value), conf=self.confidence,
            device=self.device, verbose=False)[0]
        detections = []
        if result.boxes is not None:
            for box in result.boxes:
                class_id = int(box.cls.item())
                detections.append({
                    'label': result.names[class_id],
                    'confidence': round(float(box.conf.item()), 4),
                    'xyxy_pixels': box.xyxy[0].cpu().tolist(),
                })
        canvas = result.plot()
        color = (40, 200, 60) if self._task_state.startswith('FOUND') else (230, 210, 60)
        cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 34), (25, 25, 25), -1)
        cv2.putText(canvas, 'CHAIR SEARCH | ' + self._task_state,
                    (8, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)
        annotated = self.bridge.cv2_to_imgmsg(canvas, encoding='bgr8')
        annotated.header = message.header
        self.image_pub.publish(annotated)
        payload = {
            'stamp': {'sec': message.header.stamp.sec,
                      'nanosec': message.header.stamp.nanosec},
            'frame_id': message.header.frame_id,
            'width': message.width,
            'height': message.height,
            'detections': detections,
        }
        self.json_pub.publish(String(data=json.dumps(payload)))
        if now - self._last_snapshot >= 5.0:
            self.save_snapshots(frame, canvas, payload)
            self._last_snapshot = now
        if now - self.last_report >= 5.0:
            labels = ', '.join(sorted({d['label'] for d in detections}))
            self.get_logger().info('Detected: ' + (labels or '(none)'))
            self.last_report = now

    def save_snapshots(self, frame, canvas, payload):
        try:
            for name, content in [('latest_camera.png', frame), ('latest_detection.jpg', canvas)]:
                destination = self._snapshot_dir / name
                temporary = destination.with_name('.writing_' + name)
                if not cv2.imwrite(str(temporary), content):
                    raise OSError(f'Could not encode {name}')
                os.replace(temporary, destination)
            destination = self._snapshot_dir / 'latest_detection.json'
            temporary = destination.with_name('.writing_' + destination.name)
            temporary.write_text(json.dumps(payload, indent=2), encoding='utf-8')
            os.replace(temporary, destination)
        except OSError as error:
            self.get_logger().warning(f'Could not save diagnostic image: {error}')


def main():
    rclpy.init()
    node = None
    try:
        node = CameraDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
