# Best detection after mapping + filtering + center calculation.
std_msgs/Header header

string  class_name        # original YOLO label, e.g. "blue" / "red"
string  kfs_type          # mapped type: "true_kfs" / "fake_kfs"
int32   image_width       # input image width  (pixels)
int32   image_height      # input image height (pixels)
float32 confidence        # confidence score [0, 1]
int32   x1                # top-left x
int32   y1                # top-left y
int32   x2                # bottom-right x
int32   y2                # bottom-right y
int32   center_u          # box center u = (x1+x2)/2
int32   center_v          # box center v = (y1+y2)/2
int32   center_offset_x   # center_u - image_width/2
int32   center_offset_y   # center_v - image_height/2
