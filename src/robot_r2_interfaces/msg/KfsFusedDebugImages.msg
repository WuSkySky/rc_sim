# Optional annotated debug images for the front, left, and right cameras in one
# inference batch. A camera that produced no new frame in this batch keeps its
# default-constructed (empty) image.
sensor_msgs/Image front
sensor_msgs/Image left
sensor_msgs/Image right
