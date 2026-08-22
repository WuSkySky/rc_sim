"""Tests for serial reconnection and one-shot abort behavior."""

from types import SimpleNamespace

from robot_r2_common import ABORT_TOPIC
from std_msgs.msg import Empty

import serial_pkg.serial_bridge as serial_bridge_module
from serial_pkg.serial_bridge import SerialBridge


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class RecordingLogger:
    def __init__(self):
        self.warnings = []
        self.errors = []
        self.infos = []

    def warn(self, message):
        self.warnings.append(message)

    def error(self, message):
        self.errors.append(message)

    def info(self, message):
        self.infos.append(message)


class RecordingParser:
    def __init__(self):
        self.clear_count = 0

    def clear(self):
        self.clear_count += 1


class FakeSerialPort:
    def __init__(self):
        self.is_open = True
        self.closed = False

    def close(self):
        self.closed = True
        self.is_open = False


def make_bridge():
    bridge = object.__new__(SerialBridge)
    bridge.serial_port = None
    bridge.next_reconnect_time = 0.0
    bridge.reconnect_interval_sec = 1.0
    bridge.serial_port_name = '/dev/fake'
    bridge.baud_rate = 115200
    bridge.write_timeout_sec = 0.1
    bridge.open_failure_abort_sent = False
    bridge.abort_publisher = RecordingPublisher()
    bridge.parser = RecordingParser()
    logger = RecordingLogger()
    bridge.get_logger = lambda: logger
    return bridge, logger


def test_abort_interface_matches_gui_abort_interface():
    assert serial_bridge_module.ABORT_TOPIC is ABORT_TOPIC
    assert serial_bridge_module.ABORT_TOPIC == '/r2/system/abort'
    assert serial_bridge_module.Empty is Empty


def test_open_failures_publish_abort_once_until_success(monkeypatch):
    bridge, logger = make_bridge()
    clock = SimpleNamespace(now=0.0)
    monkeypatch.setattr(
        serial_bridge_module.time, 'monotonic', lambda: clock.now)

    outcomes = [
        OSError('first failure'),
        OSError('second failure'),
        FakeSerialPort(),
        OSError('failure after reconnect'),
    ]
    attempts = []

    def open_serial(**kwargs):
        attempts.append(kwargs)
        outcome = outcomes.pop(0)
        if isinstance(outcome, Exception):
            raise outcome
        return outcome

    monkeypatch.setattr(serial_bridge_module.serial, 'Serial', open_serial)

    assert not bridge.ensure_serial_connection()
    assert len(bridge.abort_publisher.messages) == 1
    assert isinstance(bridge.abort_publisher.messages[0], Empty)
    assert len(logger.warnings) == 1

    clock.now = 0.5
    assert not bridge.ensure_serial_connection()
    assert len(attempts) == 1
    assert len(bridge.abort_publisher.messages) == 1
    assert len(logger.warnings) == 1

    clock.now = 1.0
    assert not bridge.ensure_serial_connection()
    assert len(attempts) == 2
    assert len(bridge.abort_publisher.messages) == 1
    assert len(logger.warnings) == 2

    clock.now = 2.0
    assert bridge.ensure_serial_connection()
    assert not bridge.open_failure_abort_sent
    assert bridge.parser.clear_count == 1
    assert len(logger.infos) == 1

    bridge.mark_serial_disconnected('Serial read failed', OSError('lost'))
    assert len(bridge.abort_publisher.messages) == 1

    clock.now = 3.0
    assert not bridge.ensure_serial_connection()
    assert len(attempts) == 4
    assert len(bridge.abort_publisher.messages) == 2
    assert isinstance(bridge.abort_publisher.messages[1], Empty)
