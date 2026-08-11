from array import array

import numpy as np
from std_msgs.msg import Header

from robot_r2_target_alignment.camera_frame import bgr_to_image


def test_bgr_to_image_uses_native_uint8_array_without_changing_pixels():
    image = np.array(
        [
            [[0, 1, 2], [3, 4, 5]],
            [[250, 251, 252], [253, 254, 255]],
        ],
        dtype=np.uint8,
    )
    header = Header()
    header.frame_id = "front_camera"

    message = bgr_to_image(image, header)

    assert message.header.frame_id == "front_camera"
    assert message.height == 2
    assert message.width == 2
    assert message.encoding == "bgr8"
    assert message.is_bigendian == 0
    assert message.step == 6
    assert isinstance(message.data, array)
    assert message.data.typecode == "B"
    assert bytes(message.data) == image.tobytes()
