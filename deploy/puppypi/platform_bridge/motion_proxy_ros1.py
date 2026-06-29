#!/usr/bin/env python3
"""PuppyPi ROS1 motion proxy: HTTP /twist -> /puppy_control/velocity."""

from __future__ import annotations

import json
import logging
import os
import queue
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

import rospy
from puppy_control.msg import Velocity


LOGGER = logging.getLogger("puppypi_motion_proxy_ros1")

MAX_X_CM_S = float(os.environ.get("PUPPYPI_MAX_X_CM_S", "8.0"))
MAX_YAW_RATE = float(os.environ.get("PUPPYPI_MAX_YAW_RATE", "0.4"))
VELOCITY_TOPIC = os.environ.get("PUPPYPI_CMD_VEL_TOPIC", "/puppy_control/velocity")
LISTEN_HOST = os.environ.get("PUPPYPI_PROXY_HOST", "0.0.0.0")
LISTEN_PORT = int(os.environ.get("PUPPYPI_PROXY_PORT", "8090"))
COMMAND_REPEAT = max(1, int(os.environ.get("PUPPYPI_COMMAND_REPEAT", "3")))
COMMAND_REPEAT_INTERVAL_SEC = max(0.0, float(os.environ.get("PUPPYPI_COMMAND_REPEAT_INTERVAL_SEC", "0.05")))

_PUBLISH_QUEUE: queue.Queue[Velocity] = queue.Queue()
_ROS_READY = threading.Event()
_ROS_ERROR: Optional[str] = None


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _build_velocity_message(x_cm_s: float, yaw_rate: float) -> Velocity:
    message = Velocity()
    message.x = x_cm_s
    message.y = 0.0
    message.yaw_rate = yaw_rate
    return message


def _ros_worker() -> None:
    global _ROS_ERROR
    try:
        rospy.init_node("nuwax_motion_proxy", anonymous=False, disable_signals=True)
        publisher = rospy.Publisher(VELOCITY_TOPIC, Velocity, queue_size=10)
        deadline = time.monotonic() + 5.0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if publisher.get_num_connections() > 0:
                break
            rospy.sleep(0.1)
        rospy.sleep(0.5)
        LOGGER.info(
            "ROS publisher ready topic=%s connections=%s",
            VELOCITY_TOPIC,
            publisher.get_num_connections(),
        )
        _ROS_READY.set()

        while not rospy.is_shutdown():
            try:
                message = _PUBLISH_QUEUE.get(timeout=0.2)
            except queue.Empty:
                continue
            for _ in range(COMMAND_REPEAT):
                publisher.publish(message)
                if COMMAND_REPEAT_INTERVAL_SEC > 0:
                    rospy.sleep(COMMAND_REPEAT_INTERVAL_SEC)
    except Exception as exc:
        _ROS_ERROR = str(exc)
        LOGGER.exception("ROS worker failed")
        _ROS_READY.set()


def _ensure_ros_ready() -> None:
    if _ROS_ERROR:
        raise RuntimeError(_ROS_ERROR)
    if not _ROS_READY.wait(timeout=5.0):
        raise RuntimeError("ROS publisher not ready")


def _enqueue_twist(x_cm_s: float, yaw_rate: float) -> None:
    _ensure_ros_ready()
    _PUBLISH_QUEUE.put(_build_velocity_message(x_cm_s, yaw_rate))


class MotionProxyHandler(BaseHTTPRequestHandler):
    server_version = "PuppyPiMotionProxyROS1/0.2"

    def log_message(self, format: str, *args) -> None:  # noqa: A003
        LOGGER.info("%s - %s", self.address_string(), format % args)

    def _send_json(self, status_code: int, payload: dict) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status_code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:  # noqa: N802
        if self.path != "/health":
            self._send_json(404, {"ok": False, "error": "not_found"})
            return
        try:
            _ensure_ros_ready()
            self._send_json(200, {"ok": True, "topic": VELOCITY_TOPIC, "ros": "noetic"})
        except Exception as exc:
            LOGGER.exception("health check failed")
            self._send_json(503, {"ok": False, "error": str(exc)})

    def do_POST(self) -> None:  # noqa: N802
        if self.path != "/twist":
            self._send_json(404, {"ok": False, "error": "not_found"})
            return

        length = int(self.headers.get("Content-Length", "0") or "0")
        raw = self.rfile.read(length) if length > 0 else b"{}"
        try:
            payload = json.loads(raw.decode("utf-8") or "{}")
            x_mps = float(payload.get("x_mps", 0.0))
            yaw_rate = float(payload.get("yaw_rate", 0.0))
            x_cm_s = _clamp(x_mps * 100.0, -MAX_X_CM_S, MAX_X_CM_S)
            yaw_rate = _clamp(yaw_rate, -MAX_YAW_RATE, MAX_YAW_RATE)
            _enqueue_twist(x_cm_s, yaw_rate)
            self._send_json(200, {"ok": True, "x_cm_s": x_cm_s, "yaw_rate": yaw_rate})
        except Exception as exc:
            LOGGER.exception("failed to handle /twist")
            self._send_json(500, {"ok": False, "error": str(exc)})


def main() -> None:
    logging.basicConfig(level=os.environ.get("PUPPYPI_PROXY_LOG_LEVEL", "INFO"))
    worker = threading.Thread(target=_ros_worker, name="nuwax-motion-proxy-ros", daemon=True)
    worker.start()
    _ensure_ros_ready()
    server = ThreadingHTTPServer((LISTEN_HOST, LISTEN_PORT), MotionProxyHandler)
    LOGGER.info("motion_proxy listening on http://%s:%s -> %s", LISTEN_HOST, LISTEN_PORT, VELOCITY_TOPIC)
    try:
        server.serve_forever()
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
