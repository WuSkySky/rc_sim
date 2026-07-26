# Single raw KFS detection box.
string  class_name    # KFS class: "r2", "fake", or "r1"
int32   class_id      # stable KFS class id
float32 confidence    # confidence score [0, 1]
int32   x1            # top-left x (pixel)
int32   y1            # top-left y (pixel)
int32   x2            # bottom-right x (pixel)
int32   y2            # bottom-right y (pixel)
