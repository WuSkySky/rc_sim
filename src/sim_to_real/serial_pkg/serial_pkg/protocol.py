"""Encoding and parsing for the Robot R2 serial frame protocol."""

import math
import struct


COMMAND_FLOAT_FIELD_COUNT = 11
FEEDBACK_FLOAT_FIELD_COUNT = 14
COMMAND_FRAME_SIZE = 46
FEEDBACK_FRAME_SIZE = 59
# Backward-compatible name for the command frame used by existing callers.
FRAME_SIZE = COMMAND_FRAME_SIZE


def _byte_order_prefix(endianness):
    if endianness == 'little':
        return '<'
    if endianness == 'big':
        return '>'
    raise ValueError("endianness must be 'little' or 'big'")


def _validate_marker(value, name):
    marker = int(value)
    if not 0 <= marker <= 0xFF:
        raise ValueError(f'{name} must be between 0 and 255')
    return marker


def frame_struct(endianness='little'):
    result = struct.Struct(
        f'{_byte_order_prefix(endianness)}B{COMMAND_FLOAT_FIELD_COUNT}fB')
    if result.size != COMMAND_FRAME_SIZE:
        raise RuntimeError(
            f'command frame size is {result.size}, '
            f'expected {COMMAND_FRAME_SIZE}')
    return result


def feedback_frame_struct(endianness='little'):
    result = struct.Struct(
        f'{_byte_order_prefix(endianness)}B{FEEDBACK_FLOAT_FIELD_COUNT}fBB')
    if result.size != FEEDBACK_FRAME_SIZE:
        raise RuntimeError(
            f'feedback frame size is {result.size}, '
            f'expected {FEEDBACK_FRAME_SIZE}')
    return result


def encode_frame(values, header=0xAA, tail=0x55, endianness='little'):
    float_values = tuple(float(value) for value in values)
    if len(float_values) != COMMAND_FLOAT_FIELD_COUNT:
        raise ValueError(
            f'command frame requires {COMMAND_FLOAT_FIELD_COUNT} floats, got '
            f'{len(float_values)}')
    if not all(math.isfinite(value) for value in float_values):
        raise ValueError('frame values must all be finite')

    return frame_struct(endianness).pack(
        _validate_marker(header, 'header'),
        *float_values,
        _validate_marker(tail, 'tail'),
    )


def decode_frame(frame, header=0xAA, tail=0x55, endianness='little'):
    frame_bytes = bytes(frame)
    if len(frame_bytes) != COMMAND_FRAME_SIZE:
        raise ValueError(
            f'command frame must be {COMMAND_FRAME_SIZE} bytes, '
            f'got {len(frame_bytes)}')

    expected_header = _validate_marker(header, 'header')
    expected_tail = _validate_marker(tail, 'tail')
    if frame_bytes[0] != expected_header:
        raise ValueError(
            f'invalid frame header 0x{frame_bytes[0]:02X}')
    if frame_bytes[-1] != expected_tail:
        raise ValueError(
            f'invalid frame tail 0x{frame_bytes[-1]:02X}')

    values = frame_struct(endianness).unpack(frame_bytes)[1:-1]
    if not all(math.isfinite(value) for value in values):
        raise ValueError('frame values must all be finite')
    return values


def decode_feedback_frame(
    frame,
    header=0xAA,
    tail=0x55,
    endianness='little',
):
    frame_bytes = bytes(frame)
    if len(frame_bytes) != FEEDBACK_FRAME_SIZE:
        raise ValueError(
            f'feedback frame must be {FEEDBACK_FRAME_SIZE} bytes, '
            f'got {len(frame_bytes)}')

    expected_header = _validate_marker(header, 'header')
    expected_tail = _validate_marker(tail, 'tail')
    if frame_bytes[0] != expected_header:
        raise ValueError(
            f'invalid frame header 0x{frame_bytes[0]:02X}')
    if frame_bytes[-1] != expected_tail:
        raise ValueError(
            f'invalid frame tail 0x{frame_bytes[-1]:02X}')

    unpacked = feedback_frame_struct(endianness).unpack(frame_bytes)
    values = unpacked[1:1 + FEEDBACK_FLOAT_FIELD_COUNT]
    button = unpacked[-2]
    if not all(math.isfinite(value) for value in values):
        raise ValueError('frame values must all be finite')
    if button not in (0, 1):
        raise ValueError(f'button must be 0 or 1, got {button}')
    return values, bool(button)


class FrameParser:
    def __init__(
        self,
        header=0xAA,
        tail=0x55,
        frame_size=COMMAND_FRAME_SIZE,
    ):
        self.header = _validate_marker(header, 'header')
        self.tail = _validate_marker(tail, 'tail')
        self.frame_size = int(frame_size)
        if self.frame_size < 2:
            raise ValueError('frame_size must be at least 2')
        self.buffer = bytearray()

    def clear(self):
        self.buffer.clear()

    def feed(self, data):
        self.buffer.extend(data)
        frames = []

        while self.buffer:
            header_index = self.buffer.find(bytes((self.header,)))
            if header_index < 0:
                self.buffer.clear()
                break
            if header_index > 0:
                del self.buffer[:header_index]
            if len(self.buffer) < self.frame_size:
                break

            if self.buffer[self.frame_size - 1] != self.tail:
                del self.buffer[0]
                continue

            frames.append(bytes(self.buffer[:self.frame_size]))
            del self.buffer[:self.frame_size]

        return frames
