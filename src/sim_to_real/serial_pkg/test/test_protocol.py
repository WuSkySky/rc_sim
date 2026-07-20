"""Tests for the serial frame protocol."""

import math
import struct

import pytest

from serial_pkg.protocol import (
    FRAME_SIZE,
    FrameParser,
    decode_frame,
    encode_frame,
)


def test_encode_frame_is_46_bytes_and_little_endian():
    values = tuple(float(index) for index in range(11))
    frame = encode_frame(values)

    assert len(frame) == FRAME_SIZE
    assert frame[0] == 0xAA
    assert frame[-1] == 0x55
    assert struct.unpack('<11f', frame[1:-1]) == values


def test_decode_frame_returns_all_little_endian_float_values():
    values = tuple(float(index) / 4.0 for index in range(11))

    assert decode_frame(encode_frame(values)) == values


def test_decode_frame_supports_big_endian():
    values = tuple(float(index) for index in range(11))
    frame = encode_frame(values, endianness='big')

    assert decode_frame(frame, endianness='big') == values


@pytest.mark.parametrize(
    'frame',
    [
        b'',
        bytes((0x00,)) + encode_frame([0.0] * 11)[1:],
        encode_frame([0.0] * 11)[:-1] + bytes((0x00,)),
    ],
)
def test_decode_frame_rejects_invalid_structure(frame):
    with pytest.raises(ValueError):
        decode_frame(frame)


def test_decode_frame_rejects_non_finite_values():
    frame = bytearray(encode_frame([0.0] * 11))
    frame[1:5] = struct.pack('<f', math.nan)

    with pytest.raises(ValueError):
        decode_frame(frame)


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
