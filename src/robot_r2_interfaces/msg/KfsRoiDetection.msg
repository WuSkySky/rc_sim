# KFS region of interest and alignment metadata from one camera frame.
# The nested frame is the sole source of sequence, timestamp, and frame_id.
# When valid is false, roi keeps that identity but has zero dimensions/data.
bool valid
robot_r2_interfaces/CameraFrame roi

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
