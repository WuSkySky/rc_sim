from contextlib import nullcontext
import math
import threading
import time

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, Float64


ABORT_TOPIC = '/r2/system/abort'
ABORT_MESSAGE = f'aborted by {ABORT_TOPIC}'


class AbortRequested(RuntimeError):
    def __init__(self):
        super().__init__(ABORT_MESSAGE)


class _AbortScope:
    def __init__(self, monitor, generation):
        self._monitor = monitor
        self._generation = generation
        self._previous = None

    def __enter__(self):
        self._previous = getattr(
            self._monitor._local, 'generation', None)
        self._monitor._local.generation = self._generation
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback):
        if self._previous is None:
            try:
                del self._monitor._local.generation
            except AttributeError:
                pass
        else:
            self._monitor._local.generation = self._previous


class AbortMonitor:
    """Tracks one-shot abort events without poisoning later requests."""

    def __init__(self, node, callback_group=None, on_abort=None):
        self._node = node
        self._on_abort = on_abort
        self._condition = threading.Condition()
        self._generation = 0
        self._local = threading.local()
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscription = node.create_subscription(
            Empty,
            ABORT_TOPIC,
            self._handle_abort,
            qos,
            callback_group=callback_group,
        )

    def _handle_abort(self, _message):
        with self._condition:
            self._generation += 1
            self._condition.notify_all()
        if self._on_abort is not None:
            try:
                self._on_abort()
            except Exception as exc:
                self._node.get_logger().error(
                    f'Failed to stop after {ABORT_TOPIC}: {exc}')
        self._node.get_logger().warn(
            f'Received {ABORT_TOPIC}; aborting active request')

    def scope(self):
        with self._condition:
            generation = self._generation
        return _AbortScope(self, generation)

    def requested(self):
        generation = getattr(self._local, 'generation', None)
        if generation is None:
            return False
        with self._condition:
            return generation != self._generation

    def raise_if_requested(self):
        if self.requested():
            raise AbortRequested()

    def wait_for_event(self, event, timeout_sec, poll_interval_sec=0.05):
        deadline = time.monotonic() + timeout_sec
        while True:
            self.raise_if_requested()
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            if event.wait(min(remaining, poll_interval_sec)):
                self.raise_if_requested()
                return True


class AbortableMixin:
    """No-op under unit-test construction until an AbortMonitor is installed."""

    def abort_scope(self):
        monitor = getattr(self, 'abort_monitor', None)
        return monitor.scope() if monitor is not None else nullcontext()

    def abort_requested(self):
        monitor = getattr(self, 'abort_monitor', None)
        return monitor.requested() if monitor is not None else False

    def raise_if_abort_requested(self):
        monitor = getattr(self, 'abort_monitor', None)
        if monitor is not None:
            monitor.raise_if_requested()

    def wait_for_event_or_abort(self, event, timeout_sec):
        monitor = getattr(self, 'abort_monitor', None)
        if monitor is None:
            return event.wait(timeout_sec)
        return monitor.wait_for_event(event, timeout_sec)

    def wait_for_service_or_abort(self, client, timeout_sec):
        if getattr(self, 'abort_monitor', None) is None:
            return client.wait_for_service(timeout_sec=timeout_sec)
        deadline = time.monotonic() + timeout_sec
        while True:
            self.raise_if_abort_requested()
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            if client.wait_for_service(timeout_sec=min(remaining, 0.05)):
                self.raise_if_abort_requested()
                return True

    def call_async_or_abort(self, client, request):
        self.raise_if_abort_requested()
        future = client.call_async(request)
        self.raise_if_abort_requested()
        return future


class PositionAbortMixin(AbortableMixin):
    """Abort behavior shared by single-axis position service nodes."""

    def hold_current_position_on_abort(self):
        with self.state_condition:
            position = self.current_position
            self.state_condition.notify_all()
        if position is not None and math.isfinite(float(position)):
            command = Float64()
            command.data = float(position)
            self.command_publisher.publish(command)

    def fill_aborted_position_response(self, request, response):
        with self.state_condition:
            final_position = (
                self.current_position
                if self.current_position is not None else 0.0
            )
        response.success = False
        response.message = ABORT_MESSAGE
        response.final_position = final_position
        response.position_error = float(request.position) - final_position
        return response
