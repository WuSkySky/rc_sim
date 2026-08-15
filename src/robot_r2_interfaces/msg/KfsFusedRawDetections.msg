# Raw top-1 classification for the front, left, and right cameras in one
# inference batch. A camera that produced no new frame in this batch keeps its
# default-constructed (empty) detections.
KfsRawDetections front
KfsRawDetections left
KfsRawDetections right
