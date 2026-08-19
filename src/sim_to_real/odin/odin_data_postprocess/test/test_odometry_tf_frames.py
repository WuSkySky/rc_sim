from types import SimpleNamespace

from builtin_interfaces.msg import Time

from odin_data_postprocess.odometry_postprocess import OdometryPostprocess


class FakeBroadcaster:
    def __init__(self):
        self.transforms = []

    def sendTransform(self, transform):
        self.transforms.append(transform)


class FakeClock:
    def now(self):
        return SimpleNamespace(to_msg=Time)


def test_odin_dynamic_transform_uses_odin_frames():
    broadcaster = FakeBroadcaster()
    node = SimpleNamespace(
        ODOM_FRAME=OdometryPostprocess.ODOM_FRAME,
        BASE_FRAME=OdometryPostprocess.BASE_FRAME,
        _make_transform=OdometryPostprocess._make_transform,
        _transform_broadcaster=broadcaster,
    )

    OdometryPostprocess._broadcast_odom_to_base(
        node,
        ((1.0, 2.0, 0.0), (0.0, 0.0, 0.0, 1.0), Time()),
    )

    transform = broadcaster.transforms[0]
    assert transform.header.frame_id == 'odom_odin'
    assert transform.child_frame_id == 'base_link_odin'


def test_odin_static_transform_keeps_unsuffixed_sensor_frame():
    broadcaster = FakeBroadcaster()
    node = SimpleNamespace(
        BASE_FRAME=OdometryPostprocess.BASE_FRAME,
        ODIN_FRAME=OdometryPostprocess.ODIN_FRAME,
        _make_transform=OdometryPostprocess._make_transform,
        _static_transform_broadcaster=broadcaster,
        get_clock=FakeClock,
    )

    OdometryPostprocess._publish_static_transform(
        node,
        (0.1, 0.2, 0.3),
        (0.0, 0.0, 0.0, 1.0),
    )

    transform = broadcaster.transforms[0]
    assert transform.header.frame_id == 'base_link_odin'
    assert transform.child_frame_id == 'odin_link'
