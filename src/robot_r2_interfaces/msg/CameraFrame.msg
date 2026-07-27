# Bounded camera frame for Fast DDS Data Sharing.
# The maximum data size is 3280 * 2464 * 3 bytes (the largest current BGR8
# mode). The large image payload is a bounded sequence; frame_id uses a small
# fixed array because Humble's Fast RTPS typesupport does not classify bounded
# strings as fully bounded.

uint8 ENCODING_BGR8=0
uint8 ENCODING_RGB8=1
uint8 ENCODING_MONO8=2
uint8 LAYOUT_VERSION=1
uint32 FRAME_ID_CAPACITY=128
uint32 DATA_CAPACITY=24245760

uint64 sequence
int32 stamp_sec
uint32 stamp_nanosec

uint32 width
uint32 height
uint32 step
uint32 data_size

uint8 encoding
uint8 is_bigendian
uint8 layout_version
uint8 frame_id_size

uint8[128] frame_id
uint8[<=24245760] data
