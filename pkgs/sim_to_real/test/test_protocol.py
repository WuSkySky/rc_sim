import math
import struct

from sim_to_real.protocol import FRAME_SIZE, FrameParser, encode_frame


def test_encode_frame_is_46_bytes_and_little_endian():
    values = tuple(float(index) for index in range(11))
    frame = encode_frame(values)

    assert len(frame) == FRAME_SIZE
    assert frame[0] == 0xAA
    assert frame[-1] == 0x55
    assert struct.unpack('<11f', frame[1:-1]) == values


def test_parser_handles_noise_fragmentation_and_multiple_frames():
    first = encode_frame([0.0] * 11)
    second = encode_frame([math.pi] * 11)
    parser = FrameParser()

    assert parser.feed(b'noise' + first[:20]) == []
    assert parser.feed(first[20:] + second) == [first, second]


def test_parser_recovers_after_invalid_tail():
    invalid = bytearray(encode_frame([0.0] * 11))
    invalid[-1] = 0x00
    valid = encode_frame([1.0] * 11)
    parser = FrameParser()

    assert parser.feed(invalid + valid) == [valid]
