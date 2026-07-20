# Single raw YOLO detection box.
string  class_name    # original YOLO class label, e.g. "red"
int32   class_id      # YOLO class id
float32 confidence    # confidence score [0, 1]
int32   x1            # top-left x (pixel)
int32   y1            # top-left y (pixel)
int32   x2            # bottom-right x (pixel)
int32   y2            # bottom-right y (pixel)
