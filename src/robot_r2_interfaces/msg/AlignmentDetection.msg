# Lightweight alignment-target detection from one camera frame.
# Pixels remain on the source camera/debug topics and are not transported here.
std_msgs/Header header
uint64 sequence

bool valid

int32 image_width
int32 image_height

# Inclusive qualified column bounds in the source image.
int32 x1
int32 x2

# Lowest non-zero mask pixels on x1 and x2 respectively.
int32 left_bottom_y
int32 right_bottom_y

int32 center_u
int32 center_offset_x
