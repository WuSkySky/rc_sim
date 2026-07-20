"""Encoding and parsing for the Robot R2 serial frame protocol."""

import math
import struct


FLOAT_FIELD_COUNT = 11
FRAME_SIZE = 46


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
        f'{_byte_order_prefix(endianness)}B{FLOAT_FIELD_COUNT}fB')
    if result.size != FRAME_SIZE:
        raise RuntimeError(
            f'protocol frame size is {result.size}, expected {FRAME_SIZE}')
    return result


def encode_frame(values, header=0xAA, tail=0x55, endianness='little'):
    float_values = tuple(float(value) for value in values)
    if len(float_values) != FLOAT_FIELD_COUNT:
        raise ValueError(
            f'frame requires {FLOAT_FIELD_COUNT} floats, got '
            f'{len(float_values)}')
    if not all(math.isfinite(value) for value in float_values):
        raise ValueError('frame values must all be finite')

    return frame_struct(endianness).pack(
        _validate_marker(header, 'header'),
        *float_values,
        _validate_marker(tail, 'tail'),
    )


class FrameParser:
    def __init__(self, header=0xAA, tail=0x55):
        self.header = _validate_marker(header, 'header')
        self.tail = _validate_marker(tail, 'tail')
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
            if len(self.buffer) < FRAME_SIZE:
                break

            if self.buffer[FRAME_SIZE - 1] != self.tail:
                del self.buffer[0]
                continue

            frames.append(bytes(self.buffer[:FRAME_SIZE]))
            del self.buffer[:FRAME_SIZE]

        return frames
