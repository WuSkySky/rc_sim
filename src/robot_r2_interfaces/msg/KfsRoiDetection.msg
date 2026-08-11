# Lightweight KFS region-of-interest result from one camera frame.
# Pixels remain on the source camera/debug topics and are not transported here.
std_msgs/Header header
uint64 sequence

bool valid

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
