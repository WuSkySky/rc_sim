import threading

import pytest

from robot_r2_common.abort import (
    ABORT_MESSAGE,
    AbortMonitor,
    AbortRequested,
)


class FakeLogger:
    def warn(self, _message):
        pass

    def error(self, _message):
        pass


class FakeNode:
    def __init__(self):
        self.callback = None

    def create_subscription(
        self, _message_type, _topic, callback, _qos, callback_group=None
    ):
        del callback_group
        self.callback = callback
        return object()

    def get_logger(self):
        return FakeLogger()


def test_abort_only_affects_scope_created_before_event():
    node = FakeNode()
    monitor = AbortMonitor(node)

    old_scope = monitor.scope()
    node.callback(object())

    with old_scope:
        assert monitor.requested()
        with pytest.raises(AbortRequested, match=ABORT_MESSAGE):
            monitor.raise_if_requested()

    with monitor.scope():
        assert not monitor.requested()


def test_abort_interrupts_event_wait():
    node = FakeNode()
    monitor = AbortMonitor(node)
    waiting = threading.Event()
    abort_thread = threading.Thread(target=lambda: node.callback(object()))

    with monitor.scope():
        abort_thread.start()
        with pytest.raises(AbortRequested, match=ABORT_MESSAGE):
            monitor.wait_for_event(waiting, 1.0, poll_interval_sec=0.01)

    abort_thread.join()
