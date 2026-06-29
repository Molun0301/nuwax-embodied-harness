#!/usr/bin/env python3
"""PuppyPi 容器侧运动代理：HTTP /twist -> puppy_control_msgs/Velocity。"""

from __future__ import annotations

import json
import logging
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

import rclpy
from puppy_control_msgs.msg import Velocity
from rclpy.node import Node


LOGGER = logging.getLogger("puppypi_motion_proxy")

MAX_X_CM_S = float(os.environ.get("PUPPYPI_MAX_X_CM_S", "8.0"))
MAX_YAW_RATE = float(os.environ.get("PUPPYPI_MAX_YAW_RATE", "0.4"))
VELOCITY_TOPIC = os.environ.get("PUPPYPI_CMD_VEL_TOPIC", "/puppy_control/velocity")
LISTEN_HOST = os.environ.get("PUPPYPI_PROXY_HOST", "0.0.0.0")
LISTEN_PORT = int(os.environ.get("PUPPYPI_PROXY_PORT", "8090"))


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class MotionPublisher(Node):
    def __init__(self) -> None:
        super().__init__("puppypi_motion_proxy")
        self._publisher = self.create_publisher(Velocity, VELOCITY_TOPIC, 10)
        LOGGER.info("Publishing Velocity on %s", VELOCITY_TOPIC)

    def publish_twist(self, x_mps: float, yaw_rate: float) -> None:
        message = Velocity()
        message.x = _clamp(float(x_mps) * 100.0, -MAX_X_CM_S, MAX_X_CM_S)
        message.y = 0.0
        message.yaw_rate = _clamp(float(yaw_rate), -MAX_YAW_RATE, MAX_YAW_RATE)
        self._publisher.publish(message)


_PUBLISHER: Optional[MotionPublisher] = None
_PUBLISHER_LOCK = threading.Lock()


def _get_publisher() -> MotionPublisher:
    global _PUBLISHER
    with _PUBLISHER_LOCK:
        if _PUBLISHER is None:
            rclpy.init()
            _PUBLISHER = MotionPublisher()
        return _PUBLISHER


class MotionProxyHandler(BaseHTTPRequestHandler):
    server_version = "PuppyPiMotionProxy/0.1"

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
            _get_publisher()
            self._send_json(200, {"ok": True, "topic": VELOCITY_TOPIC})
        except Exception as exc:
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
            publisher = _get_publisher()
            publisher.publish_twist(x_mps, yaw_rate)
            rclpy.spin_once(publisher, timeout_sec=0)
            self._send_json(
                200,
                {
                    "ok": True,
                    "x_cm_s": _clamp(x_mps * 100.0, -MAX_X_CM_S, MAX_X_CM_S),
                    "yaw_rate": _clamp(yaw_rate, -MAX_YAW_RATE, MAX_YAW_RATE),
                },
            )
        except Exception as exc:
            LOGGER.exception("处理 /twist 失败")
            self._send_json(500, {"ok": False, "error": str(exc)})


def main() -> None:
    logging.basicConfig(level=os.environ.get("PUPPYPI_PROXY_LOG_LEVEL", "INFO"))
    server = ThreadingHTTPServer((LISTEN_HOST, LISTEN_PORT), MotionProxyHandler)
    LOGGER.info("motion_proxy listening on http://%s:%s", LISTEN_HOST, LISTEN_PORT)
    try:
        server.serve_forever()
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
