from types import SimpleNamespace

from serial_pkg.frame_ids import (
    BASE_FRAME,
    MAP_FRAME,
    ODOM_FRAME,
    set_odometry_frame_ids,
)


def test_serial_odometry_frame_chain_is_source_specific():
    assert (MAP_FRAME, ODOM_FRAME, BASE_FRAME) == (
        'map',
        'odom_serial',
        'base_link_serial',
    )


def test_serial_odometry_message_uses_serial_frames():
    message = SimpleNamespace(
        header=SimpleNamespace(frame_id=''),
        child_frame_id='',
    )

    set_odometry_frame_ids(message)

    assert message.header.frame_id == 'odom_serial'
    assert message.child_frame_id == 'base_link_serial'

