# KFS region of interest and alignment metadata from one camera frame.
std_msgs/Header header
bool valid
sensor_msgs/Image roi

int32 image_width
int32 image_height

# Inclusive ROI bounds in the source image.
int32 x1
int32 y1
int32 x2
int32 y2

int32 center_u
int32 center_v
int32 center_offset_x
int32 center_offset_y
