from types import SimpleNamespace

from builtin_interfaces.msg import Time

from serial_pkg.odometry_tf import OdometryTf


class FakeBroadcaster:
    def __init__(self):
        self.transforms = []

    def sendTransform(self, transform):
        self.transforms.append(transform)


def test_serial_dynamic_transform_uses_serial_frames():
    broadcaster = FakeBroadcaster()
    node = SimpleNamespace(
        ODOM_FRAME=OdometryTf.ODOM_FRAME,
        BASE_FRAME=OdometryTf.BASE_FRAME,
        _make_transform=OdometryTf._make_transform,
        _transform_broadcaster=broadcaster,
    )

    OdometryTf._broadcast_odom_to_base(
        node,
        ((1.0, 2.0, 0.0), (0.0, 0.0, 0.0, 1.0), Time()),
    )

    transform = broadcaster.transforms[0]
    assert transform.header.frame_id == 'odom_serial'
    assert transform.child_frame_id == 'base_link_serial'

