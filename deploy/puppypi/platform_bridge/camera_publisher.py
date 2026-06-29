#!/usr/bin/env python3
"""PuppyPi 容器侧摄像头发布器：OpenCV /dev/videoX -> ROS2 Image。"""

from __future__ import annotations

import logging
import os
import sys
from glob import glob
from typing import Optional

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


LOGGER = logging.getLogger("puppypi_camera_publisher")

CAMERA_DEVICE = os.environ.get("PUPPYPI_CAMERA_DEVICE", "/dev/video0").strip()
CAMERA_DEVICES = [item.strip() for item in os.environ.get("PUPPYPI_CAMERA_DEVICES", "").split(",") if item.strip()]
IMAGE_TOPIC = os.environ.get("PUPPYPI_IMAGE_TOPIC", "/image_raw").strip()
FPS = float(os.environ.get("PUPPYPI_CAMERA_FPS", "10"))
WIDTH = int(os.environ.get("PUPPYPI_CAMERA_WIDTH", "640"))
HEIGHT = int(os.environ.get("PUPPYPI_CAMERA_HEIGHT", "480"))


class CameraPublisher(Node):
    def __init__(self) -> None:
        super().__init__("puppypi_camera_publisher")
        self._publisher = self.create_publisher(Image, IMAGE_TOPIC, 10)
        self._camera_device = ""
        self._capture = self._open_capture()
        if WIDTH > 0:
            self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        if HEIGHT > 0:
            self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)

        if not self._capture.isOpened():
            raise RuntimeError(f"无法打开摄像头设备: {self._candidate_devices()}")

        interval = 1.0 / max(1.0, FPS)
        self._timer = self.create_timer(interval, self._publish_frame)
        LOGGER.info("Publishing camera %s to %s @ %.1f fps", self._camera_device, IMAGE_TOPIC, FPS)

    def _candidate_devices(self) -> list[str]:
        if CAMERA_DEVICES:
            return CAMERA_DEVICES
        candidates = [CAMERA_DEVICE] if CAMERA_DEVICE else []
        candidates.extend(sorted(glob("/dev/video*")))
        deduped: list[str] = []
        for item in candidates:
            if item not in deduped:
                deduped.append(item)
        return deduped

    def _open_capture(self):
        for device in self._candidate_devices():
            LOGGER.info("Trying camera device %s with V4L2 backend", device)
            capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
            if capture.isOpened():
                ok, frame = capture.read()
                if ok and frame is not None:
                    self._camera_device = device
                    return capture
            capture.release()
        return cv2.VideoCapture()

    def _publish_frame(self) -> None:
        ok, frame = self._capture.read()
        if not ok or frame is None:
            LOGGER.warning("读取摄像头帧失败: %s", CAMERA_DEVICE)
            return
        message = self._frame_to_image(frame)
        self._publisher.publish(message)

    def _frame_to_image(self, frame) -> Image:
        height, width = frame.shape[:2]
        message = Image()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = os.environ.get("PUPPYPI_CAMERA_FRAME_ID", "puppypi_camera")
        message.height = int(height)
        message.width = int(width)
        message.encoding = "bgr8"
        message.is_bigendian = 0
        message.step = int(width * 3)
        message.data = frame.tobytes()
        return message

    def close(self) -> None:
        self._capture.release()


def main() -> None:
    logging.basicConfig(level=os.environ.get("PUPPYPI_PROXY_LOG_LEVEL", "INFO"))
    rclpy.init()
    node: Optional[CameraPublisher] = None
    try:
        node = CameraPublisher()
        rclpy.spin(node)
    except Exception:
        LOGGER.exception("camera_publisher 启动失败")
        sys.exit(1)
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
