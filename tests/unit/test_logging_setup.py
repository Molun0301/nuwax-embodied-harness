from __future__ import annotations

import logging
from pathlib import Path

from fastapi.testclient import TestClient

from gateways.relay.server import create_relay_app
from logging_utils import ContextAwareFormatter, LogRateLimiter
from settings import LoggingConfig, RelayServerConfig, configure_logging, load_config


def test_load_config_prefers_nuwax_log_variables(monkeypatch, tmp_path: Path) -> None:
    """新日志变量名应优先覆盖旧变量名。"""

    new_log_dir = tmp_path / "new_logs"
    old_log_dir = tmp_path / "old_logs"
    new_log_file = new_log_dir / "nuwax_robot_bridge.log"
    old_log_file = old_log_dir / "go2_proxy.log"

    monkeypatch.setenv("NUWAX_LOG_DIR", str(new_log_dir))
    monkeypatch.setenv("NUWAX_LOG_FILE", str(new_log_file))
    monkeypatch.setenv("NUWAX_LOG_LEVEL", "WARNING")
    monkeypatch.setenv("NUWAX_LOG_MAX_LINES", "123")
    monkeypatch.setenv("NUWAX_LOG_TRIM_CHECK_INTERVAL", "9")
    monkeypatch.setenv("GO2_PROXY_LOG_DIR", str(old_log_dir))
    monkeypatch.setenv("GO2_PROXY_LOG_FILE", str(old_log_file))
    monkeypatch.setenv("GO2_PROXY_LOG_LEVEL", "INFO")
    monkeypatch.setenv("GO2_PROXY_LOG_MAX_LINES", "100000")
    monkeypatch.setenv("GO2_PROXY_LOG_TRIM_CHECK_INTERVAL", "200")

    config = load_config()

    assert config.logging.log_dir == str(new_log_dir)
    assert config.logging.log_file == str(new_log_file)
    assert config.logging.level == "WARNING"
    assert config.logging.max_lines == 123
    assert config.logging.trim_check_interval == 9


def test_configure_logging_writes_file_and_request_log(tmp_path: Path) -> None:
    """日志初始化后应能写入文件，并记录 HTTP 请求摘要。"""

    log_dir = tmp_path / "logs"
    log_file = log_dir / "runtime.log"
    configure_logging(
        LoggingConfig(
            log_dir=str(log_dir),
            log_file=str(log_file),
            level="INFO",
            max_lines=1000,
            trim_check_interval=10,
        )
    )

    app = create_relay_app(
        RelayServerConfig(
            enabled=True,
            host="127.0.0.1",
            port=9766,
            upstream_base_url="http://example.invalid",
            incoming_tokens=("relay-token",),
            upstream_token="agent-token",
        )
    )

    with TestClient(app) as client:
        response = client.get("/relay/health")
        assert response.status_code == 200

    for handler in logging.getLogger().handlers:
        flush = getattr(handler, "flush", None)
        if callable(flush):
            flush()

    content = log_file.read_text(encoding="utf-8")
    assert "Relay 请求完成" in content
    assert "[nuwax_robot_bridge.relay]" in content
    assert "event=relay.request.completed" in content
    assert "path=/relay/health" in content


def test_configure_logging_repairs_nul_bytes_and_keeps_latest_lines(tmp_path: Path) -> None:
    """初始化日志时应修复空洞字节，并保留尾部日志。"""

    log_dir = tmp_path / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    log_file = log_dir / "runtime.log"
    log_file.write_bytes(
        b"line-1\n"
        b"\x00\x00\x00\x00line-2\n"
        b"line-3\n"
    )

    configure_logging(
        LoggingConfig(
            log_dir=str(log_dir),
            log_file=str(log_file),
            level="INFO",
            max_lines=2,
            trim_check_interval=100,
        )
    )

    logger = logging.getLogger("nuwax_robot_bridge.test.repair")
    logger.info("line-4")

    for handler in logging.getLogger().handlers:
        flush = getattr(handler, "flush", None)
        if callable(flush):
            flush()

    data = log_file.read_bytes()
    content = data.decode("utf-8")
    assert b"\x00" not in data
    assert "line-1" not in content
    assert "line-2" in content
    assert "line-3" in content
    assert "line-4" in content


def test_context_aware_formatter_appends_extra_fields() -> None:
    """统一 formatter 应把扩展字段稳定追加到日志末尾。"""

    formatter = ContextAwareFormatter("%(levelname)s %(message)s")
    record = logging.makeLogRecord(
        {
            "name": "nuwax_robot_bridge.test",
            "levelno": logging.INFO,
            "levelname": "INFO",
            "msg": "日志测试",
            "args": (),
            "event": "test.logging",
            "path": "/api/health",
            "duration_ms": 12.345,
        }
    )

    rendered = formatter.format(record)

    assert rendered.startswith("INFO 日志测试")
    assert "event=test.logging" in rendered
    assert "path=/api/health" in rendered
    assert "duration_ms=12.345" in rendered


def test_log_rate_limiter_suppresses_repeated_logs() -> None:
    """重复日志应在时间窗内被压制，并在下一次放行时回填 suppressed 统计。"""

    class _ListHandler(logging.Handler):
        def __init__(self) -> None:
            super().__init__()
            self.records = []

        def emit(self, record: logging.LogRecord) -> None:
            self.records.append(record)

    now = {"value": 0.0}
    limiter = LogRateLimiter(clock=lambda: now["value"])
    logger = logging.getLogger("nuwax_robot_bridge.test.rate_limit")
    handler = _ListHandler()
    previous_handlers = list(logger.handlers)
    previous_level = logger.level
    previous_propagate = logger.propagate
    logger.handlers = [handler]
    logger.setLevel(logging.INFO)
    logger.propagate = False

    try:
        limiter.log(logger, logging.INFO, "重复日志测试", key="same", interval_sec=5.0, extra={"event": "test.rate"})
        limiter.log(logger, logging.INFO, "重复日志测试", key="same", interval_sec=5.0, extra={"event": "test.rate"})
        now["value"] = 6.0
        limiter.log(logger, logging.INFO, "重复日志测试", key="same", interval_sec=5.0, extra={"event": "test.rate"})
    finally:
        logger.handlers = previous_handlers
        logger.setLevel(previous_level)
        logger.propagate = previous_propagate

    assert len(handler.records) == 2
    assert getattr(handler.records[0], "event") == "test.rate"
    assert getattr(handler.records[1], "suppressed_count") == 1
    assert getattr(handler.records[1], "rate_limit_window_sec") == 5.0


def test_relay_health_log_is_rate_limited(tmp_path: Path) -> None:
    """高频健康检查日志应被限流，避免重复刷屏。"""

    log_dir = tmp_path / "logs"
    log_file = log_dir / "runtime.log"
    configure_logging(
        LoggingConfig(
            log_dir=str(log_dir),
            log_file=str(log_file),
            level="INFO",
            max_lines=1000,
            trim_check_interval=10,
        )
    )

    app = create_relay_app(
        RelayServerConfig(
            enabled=True,
            host="127.0.0.1",
            port=9766,
            upstream_base_url="http://example.invalid",
            incoming_tokens=("relay-token",),
            upstream_token="agent-token",
        )
    )

    with TestClient(app) as client:
        assert client.get("/relay/health").status_code == 200
        assert client.get("/relay/health").status_code == 200

    for handler in logging.getLogger().handlers:
        flush = getattr(handler, "flush", None)
        if callable(flush):
            flush()

    content = log_file.read_text(encoding="utf-8")
    assert content.count("Relay 请求完成") == 1
