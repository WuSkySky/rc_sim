"""Fixed frame IDs for lower-machine serial odometry."""


MAP_FRAME = 'map'
ODOM_FRAME = 'odom_serial'
BASE_FRAME = 'base_link_serial'


def set_odometry_frame_ids(message):
    """Apply the lower-machine odometry frames to an Odometry-like message."""
    message.header.frame_id = ODOM_FRAME
    message.child_frame_id = BASE_FRAME

