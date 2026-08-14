import threading
import time
from types import SimpleNamespace

import pytest
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

from robot_r2_control.alignment import (
    ALIGN_SERVICE,
    AlignmentConfig,
    AlignmentController,
    CMD_VEL_TOPIC,
    DETECTION_TOPIC,
)
from robot_r2_interfaces.msg import AlignmentDetection
from robot_r2_interfaces.srv import Align


def wait_until(predicate, timeout_sec=2.0):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


@pytest.fixture
def alignment_harness():
    initialized_here = not rclpy.ok()
    if initialized_here:
        rclpy.init()
    controller = AlignmentController()
    results = controller.set_parameters([
        Parameter('stable_cycles', value=2),
        Parameter('default_timeout_sec', value=0.2),
    ])
    assert all(result.successful for result in results)

    driver = Node('alignment_test_driver')
    detections = driver.create_publisher(
        AlignmentDetection,
        DETECTION_TOPIC,
        qos_profile_sensor_data,
    )
    commands = []
    driver.create_subscription(
        Twist,
        CMD_VEL_TOPIC,
        commands.append,
        10,
    )
    client = driver.create_client(Align, ALIGN_SERVICE)

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(controller)
    executor.add_node(driver)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    assert client.wait_for_service(timeout_sec=2.0)

    yield SimpleNamespace(
        controller=controller,
        detections=detections,
        commands=commands,
        client=client,
    )

    executor.shutdown()
    spin_thread.join(timeout=2.0)
    executor.remove_node(driver)
    executor.remove_node(controller)
    driver.destroy_node()
    controller.destroy_node()
    if initialized_here and rclpy.ok():
        rclpy.shutdown()


def send_request(harness, tolerance=0.0, timeout=0.0):
    request = Align.Request()
    request.pixel_tolerance = tolerance
    request.timeout_sec = timeout
    future = harness.client.call_async(request)
    assert wait_until(lambda: harness.controller._alignment_active)
    return future


def publish_detection(harness, valid, offset=0):
    message = AlignmentDetection()
    message.valid = valid
    message.center_offset_x = offset
    harness.detections.publish(message)


def wait_for_nonzero_command(harness, start_index, offset=30):
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        publish_detection(harness, valid=True, offset=offset)
        for command in harness.commands[start_index:]:
            if command.linear.y != 0.0:
                return command.linear.y
        time.sleep(0.02)
    pytest.fail('alignment did not publish a non-zero command')


def test_alignment_succeeds_after_stable_detections(alignment_harness):
    future = send_request(alignment_harness, tolerance=2.0, timeout=1.0)

    deadline = time.monotonic() + 1.0
    while not future.done() and time.monotonic() < deadline:
        publish_detection(alignment_harness, valid=True, offset=1)
        time.sleep(0.02)

    assert future.done()
    response = future.result()
    assert response.success
    assert response.final_offset_x == 1
    assert 'stable=2/2' in response.message
    assert wait_until(lambda: bool(alignment_harness.commands))
    assert alignment_harness.commands[-1].linear.y == 0.0


def test_alignment_reports_no_detection_timeout(alignment_harness):
    future = send_request(alignment_harness, timeout=0.1)

    assert wait_until(future.done)
    response = future.result()
    assert not response.success
    assert response.message == 'Alignment timeout: no target detected'
    assert response.final_offset_x == 0
    assert wait_until(lambda: bool(alignment_harness.commands))
    assert alignment_harness.commands[-1].linear.y == 0.0


def test_alignment_reports_target_lost(alignment_harness):
    future = send_request(alignment_harness, timeout=0.2)
    initial_sequence = alignment_harness.controller._frame_sequence
    deadline = time.monotonic() + 1.0
    while (
        alignment_harness.controller._frame_sequence == initial_sequence
        and time.monotonic() < deadline
    ):
        publish_detection(alignment_harness, valid=True, offset=30)
        time.sleep(0.02)
    assert alignment_harness.controller._frame_sequence > initial_sequence
    deadline = time.monotonic() + 1.0
    while not future.done() and time.monotonic() < deadline:
        publish_detection(alignment_harness, valid=False)
        time.sleep(0.02)

    assert future.done()
    response = future.result()
    assert not response.success
    assert response.message == 'Alignment timeout: target lost'
    assert response.final_offset_x == 0
    assert alignment_harness.commands[-1].linear.y == 0.0


def test_pid_output_and_integral_are_limited():
    controller = object.__new__(AlignmentController)
    controller._pid_reset()
    config = AlignmentConfig(
        pixel_tolerance=1,
        stable_cycles=2,
        default_timeout_sec=1.0,
        reverse_direction=False,
        kp=1.0,
        ki=1.0,
        kd=0.0,
        integral_limit=0.25,
        output_limit=0.1,
    )

    assert controller._pid_update(10.0, 1.0, config) == 0.1
    assert controller._integral == 0.25


@pytest.mark.parametrize(
    'field, value',
    [
        ('pixel_tolerance', -1),
        ('stable_cycles', 0),
        ('default_timeout_sec', 0.0),
        ('reverse_direction', 1),
        ('integral_limit', -0.1),
        ('output_limit', 0.0),
    ],
)
def test_invalid_alignment_config_is_rejected(field, value):
    values = {
        'pixel_tolerance': 1,
        'stable_cycles': 2,
        'default_timeout_sec': 1.0,
        'reverse_direction': False,
        'kp': 0.1,
        'ki': 0.0,
        'kd': 0.0,
        'integral_limit': 0.5,
        'output_limit': 0.1,
    }
    values[field] = value

    with pytest.raises(ValueError):
        AlignmentController._config_from_values(values)


def test_dynamic_parameter_update_is_validated(alignment_harness):
    rejected = alignment_harness.controller.set_parameters([
        Parameter('stable_cycles', value=0),
    ])[0]
    rejected_direction = alignment_harness.controller._on_parameters_changed([
        Parameter('reverse_direction', value=1),
    ])
    accepted = alignment_harness.controller.set_parameters([
        Parameter('output_limit', value=0.05),
    ])[0]

    assert not rejected.successful
    assert not rejected_direction.successful
    assert accepted.successful
    assert alignment_harness.controller._config.stable_cycles == 2
    assert not alignment_harness.controller._config.reverse_direction
    assert alignment_harness.controller._config.output_limit == 0.05


def test_reverse_direction_changes_the_next_control_output(
    alignment_harness,
):
    results = alignment_harness.controller.set_parameters([
        Parameter('kp', value=0.001),
        Parameter('ki', value=0.0),
        Parameter('kd', value=0.0),
        Parameter('output_limit', value=0.1),
    ])
    assert all(result.successful for result in results)
    future = send_request(alignment_harness, timeout=1.0)

    original_output = wait_for_nonzero_command(
        alignment_harness,
        start_index=0,
    )
    result = alignment_harness.controller.set_parameters([
        Parameter('reverse_direction', value=True),
    ])[0]
    assert result.successful
    start_index = len(alignment_harness.commands)
    reversed_output = wait_for_nonzero_command(
        alignment_harness,
        start_index=start_index,
    )

    assert original_output == pytest.approx(-0.03)
    assert reversed_output == pytest.approx(0.03)
    deadline = time.monotonic() + 1.0
    while not future.done() and time.monotonic() < deadline:
        publish_detection(alignment_harness, valid=True, offset=0)
        time.sleep(0.02)
    assert future.done()
