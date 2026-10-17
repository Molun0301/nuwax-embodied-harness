"""The delivery recorder must ship the transform protocol it imports."""

from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]


def test_lio_delivery_help_imports_shipped_protocols():
    result = subprocess.run(
        [sys.executable, str(ROOT / "scripts/diagnostics/capture_lio_delivery.py"), "--help"],
        capture_output=True, text=True, timeout=10,
    )
    assert result.returncode == 0, result.stderr
    assert "--private-prefix" in result.stdout


def test_transform_message_roundtrip(monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT / "services/nuwax-workspace/lcmtypes/generated/python"))
    from geometry_msgs.TransformStamped import TransformStamped
    from tf2_msgs.TFMessage import TFMessage

    edge = TransformStamped()
    edge.header.frame_id = "map"
    edge.child_frame_id = "base_link"
    edge.transform.translation.x = 1.25
    message = TFMessage()
    message.transforms_length = 1
    message.transforms = [edge]
    decoded = TFMessage.decode(message.encode())
    assert decoded.transforms_length == 1
    assert decoded.transforms[0].header.frame_id == "map"
    assert decoded.transforms[0].child_frame_id == "base_link"
    assert decoded.transforms[0].transform.translation.x == 1.25
