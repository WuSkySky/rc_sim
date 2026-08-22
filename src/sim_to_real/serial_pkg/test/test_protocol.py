"""Tests for the serial frame protocol."""

import math
import struct

import pytest

from serial_pkg.protocol import (
    COMMAND_FRAME_SIZE,
    FEEDBACK_FRAME_SIZE,
    FRAME_SIZE,
    FrameParser,
    decode_feedback_frame,
    decode_frame,
    encode_frame,
)


def feedback_frame(values, button, endianness='little'):
    prefix = '<' if endianness == 'little' else '>'
    return struct.pack(
        f'{prefix}B14fBB', 0xAA, *values, button, 0x55)


def test_encode_command_frame_is_46_bytes_and_little_endian():
    values = tuple(float(index) for index in range(11))
    frame = encode_frame(values)

    assert len(frame) == FRAME_SIZE
    assert len(frame) == COMMAND_FRAME_SIZE
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


@pytest.mark.parametrize('button', [0, 1])
def test_decode_feedback_frame_is_59_bytes_with_button_before_tail(button):
    values = tuple(float(index) / 4.0 for index in range(14))
    frame = feedback_frame(values, button)

    decoded_values, pressed = decode_feedback_frame(frame)

    assert len(frame) == FEEDBACK_FRAME_SIZE
    assert frame[-2] == button
    assert frame[-1] == 0x55
    assert decoded_values == values
    assert pressed is bool(button)


def test_decode_feedback_frame_supports_big_endian():
    values = tuple(float(index) for index in range(14))

    assert decode_feedback_frame(
        feedback_frame(values, 1, endianness='big'),
        endianness='big',
    ) == (values, True)


@pytest.mark.parametrize('button', [2, 255])
def test_decode_feedback_frame_rejects_invalid_button(button):
    with pytest.raises(ValueError, match='button must be 0 or 1'):
        decode_feedback_frame(feedback_frame([0.0] * 14, button))


@pytest.mark.parametrize(
    'frame',
    [
        b'',
        feedback_frame([0.0] * 14, 0)[:-1],
        feedback_frame([0.0] * 14, 0)[:-1] + bytes((0x00,)),
    ],
)
def test_decode_feedback_frame_rejects_invalid_length_or_tail(frame):
    with pytest.raises(ValueError):
        decode_feedback_frame(frame)


def test_decode_feedback_frame_rejects_non_finite_values():
    frame = bytearray(feedback_frame([0.0] * 14, 0))
    frame[1:5] = struct.pack('<f', math.nan)

    with pytest.raises(ValueError, match='finite'):
        decode_feedback_frame(frame)


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


def test_feedback_parser_handles_noise_fragmentation_and_multiple_frames():
    first = feedback_frame([0.0] * 14, 0)
    second = feedback_frame([math.pi] * 14, 1)
    parser = FrameParser(frame_size=FEEDBACK_FRAME_SIZE)

    assert parser.feed(b'noise' + first[:20]) == []
    assert parser.feed(first[20:] + second) == [first, second]


def test_feedback_parser_recovers_after_invalid_tail():
    invalid = bytearray(feedback_frame([0.0] * 14, 0))
    invalid[-1] = 0x00
    valid = feedback_frame([1.0] * 14, 1)
    parser = FrameParser(frame_size=FEEDBACK_FRAME_SIZE)

    assert parser.feed(invalid + valid) == [valid]
