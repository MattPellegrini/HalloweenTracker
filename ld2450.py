"""Minimal streaming parser for LD2450 target-data frames."""

FRAME_HEADER = b"\xaa\xff\x03\x00"
FRAME_FOOTER = b"\x55\xcc"
FRAME_LENGTH = 30
TARGET_COUNT = 3


def _signed_magnitude(raw):
    magnitude = raw & 0x7fff
    return magnitude if raw & 0x8000 else -magnitude


def decode_frame(frame):
    """Return up to three (x_mm, y_mm, speed_cm_s, resolution_mm) tuples."""
    if len(frame) != FRAME_LENGTH:
        raise ValueError("LD2450 frame must be 30 bytes")
    if frame[:4] != FRAME_HEADER or frame[-2:] != FRAME_FOOTER:
        raise ValueError("invalid LD2450 frame markers")

    targets = []
    for index in range(TARGET_COUNT):
        offset = 4 + index * 8
        x_raw = frame[offset] | (frame[offset + 1] << 8)
        y_raw = frame[offset + 2] | (frame[offset + 3] << 8)
        speed_raw = frame[offset + 4] | (frame[offset + 5] << 8)
        resolution = frame[offset + 6] | (frame[offset + 7] << 8)

        x_mm = _signed_magnitude(x_raw)
        y_mm = _signed_magnitude(y_raw)
        if x_mm == 0 and y_mm == 0 and speed_raw == 0 and resolution == 0:
            continue
        targets.append((x_mm, y_mm, _signed_magnitude(speed_raw), resolution))
    return targets


class LD2450Reader:
    def __init__(self, uart):
        self.uart = uart
        self.buffer = bytearray()

    def poll(self):
        """Return one decoded target list when a complete frame is available."""
        available = self.uart.any()
        if available:
            self.buffer.extend(self.uart.read(available))

        header_index = self.buffer.find(FRAME_HEADER)
        if header_index < 0:
            keep = len(FRAME_HEADER) - 1
            if len(self.buffer) > keep:
                self.buffer = bytearray(self.buffer[-keep:])
            return None
        if header_index:
            self.buffer = bytearray(self.buffer[header_index:])
        if len(self.buffer) < FRAME_LENGTH:
            return None
        if self.buffer[FRAME_LENGTH - 2:FRAME_LENGTH] != FRAME_FOOTER:
            self.buffer = bytearray(self.buffer[1:])
            return None

        frame = bytes(self.buffer[:FRAME_LENGTH])
        self.buffer = bytearray(self.buffer[FRAME_LENGTH:])
        return decode_frame(frame)