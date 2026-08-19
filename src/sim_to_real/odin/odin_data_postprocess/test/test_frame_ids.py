from odin_data_postprocess.frame_ids import (
    BASE_FRAME,
    MAP_FRAME,
    ODIN_FRAME,
    ODOM_FRAME,
)


def test_odin_odometry_frame_chain_is_source_specific():
    assert (MAP_FRAME, ODOM_FRAME, BASE_FRAME, ODIN_FRAME) == (
        'map',
        'odom_odin',
        'base_link_odin',
        'odin_link',
    )
