"""Streaming UART parser for Hi-Link LD2450 24 GHz mmWave radar target-data frames.

Protocol Specification:
  - Baud rate: 256000 bps (8 data bits, no parity, 1 stop bit).
  - Update rate: ~10 Hz (new frame roughly every 100 ms).
  - Frame structure (30 bytes total):
      Bytes  0..3  : Frame header: 0xAA 0xFF 0x03 0x00
      Bytes  4..11 : Target Slot 1 (8 bytes)
      Bytes 12..19 : Target Slot 2 (8 bytes)
      Bytes 20..27 : Target Slot 3 (8 bytes)
      Bytes 28..29 : Frame footer: 0x55 0xCC

  - Each 8-byte target slot layout (Little-Endian):
      Byte 0..1: X coordinate (mm) - 16-bit sign-magnitude (bit 15: 1=pos, 0=neg)
      Byte 2..3: Y coordinate (mm) - 16-bit sign-magnitude (bit 15: 1=pos, 0=neg)
      Byte 4..5: Speed (cm/s)      - 16-bit sign-magnitude (bit 15: 1=pos, 0=neg)
      Byte 6..7: Distance resolution gate (mm) - 16-bit unsigned integer
"""

FRAME_HEADER = b"\xaa\xff\x03\x00"
FRAME_FOOTER = b"\x55\xcc"
FRAME_LENGTH = 30
TARGET_COUNT = 3


def _signed_magnitude(raw):
    """Decode a 16-bit integer using the LD2450 sign-magnitude convention.

    Bit 15 is the sign flag:
      - Bit 15 == 1 -> positive magnitude
      - Bit 15 == 0 -> negative magnitude
    Bits 0..14 store the absolute magnitude.
    """
    magnitude = raw & 0x7fff
    return magnitude if raw & 0x8000 else -magnitude


def decode_frame_slots(frame):
    """Decode a 30-byte LD2450 frame into three slot entries.

    Returns:
      A 3-element list [slot1, slot2, slot3], where each entry is either:
        (x_mm, y_mm, speed_cm_s, resolution_mm)
      or None if that target slot is unoccupied.

    Preserving slot positions is essential for 'fixed_slot' target assignment
    so that Skull 1 stays locked to radar slot 1 and Skull 2 to slot 2.
    """
    if len(frame) != FRAME_LENGTH:
        raise ValueError("LD2450 frame must be 30 bytes")
    if frame[:4] != FRAME_HEADER or frame[-2:] != FRAME_FOOTER:
        raise ValueError("invalid LD2450 frame markers")

    slots = []
    for index in range(TARGET_COUNT):
        offset = 4 + index * 8
        # Little-endian 16-bit values
        x_raw = frame[offset] | (frame[offset + 1] << 8)
        y_raw = frame[offset + 2] | (frame[offset + 3] << 8)
        speed_raw = frame[offset + 4] | (frame[offset + 5] << 8)
        resolution = frame[offset + 6] | (frame[offset + 7] << 8)

        x_mm = _signed_magnitude(x_raw)
        y_mm = _signed_magnitude(y_raw)
        # An empty radar slot reports all zeros across all fields
        if x_mm == 0 and y_mm == 0 and speed_raw == 0 and resolution == 0:
            slots.append(None)
        else:
            slots.append((x_mm, y_mm, _signed_magnitude(speed_raw),
                          resolution))
    return slots


def decode_frame(frame):
    """Decode frame and return only active (non-empty) targets as a list."""
    return [target for target in decode_frame_slots(frame)
            if target is not None]


class LD2450Reader:
    """Streaming reader that parses LD2450 frames from a MicroPython UART."""

    def __init__(self, uart):
        self.uart = uart
        self.buffer = bytearray()

    def poll(self):
        """Poll the UART and return decoded target slots if a valid frame is ready.

        Returns:
          A 3-element list [slot1, slot2, slot3] if a complete, valid frame was
          received, or None if more UART data is needed. Handles stream synchronization,
          skips framing noise, and recovers cleanly from dropped bytes.
        """
        available = self.uart.any()
        if available:
            self.buffer.extend(self.uart.read(available))

        # Look for the 4-byte frame header
        header_index = self.buffer.find(FRAME_HEADER)
        if header_index < 0:
            # Retain only the trailing bytes that could form part of a header
            keep = len(FRAME_HEADER) - 1
            if len(self.buffer) > keep:
                self.buffer = bytearray(self.buffer[-keep:])
            return None

        # Discard any noise prior to the frame header
        if header_index:
            self.buffer = bytearray(self.buffer[header_index:])

        # Wait until full 30-byte frame length is accumulated
        if len(self.buffer) < FRAME_LENGTH:
            return None

        # Validate 2-byte frame footer
        if self.buffer[FRAME_LENGTH - 2:FRAME_LENGTH] != FRAME_FOOTER:
            # Corrupted frame: shift past the invalid byte and search again
            self.buffer = bytearray(self.buffer[1:])
            return None

        # Extract and unpack the validated frame
        frame = bytes(self.buffer[:FRAME_LENGTH])
        self.buffer = bytearray(self.buffer[FRAME_LENGTH:])
        return decode_frame_slots(frame)