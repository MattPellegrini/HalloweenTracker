import unittest

from aiming import solve_angles
from ld2450 import FRAME_FOOTER, FRAME_HEADER, LD2450Reader, decode_frame
from tmc2209 import (build_read_packet, build_write_packet,
                     current_cs_for_rms, gconf_for_uart, uart_crc)
from tracking import TargetSelector


def word(value):
    return bytes((value & 0xff, (value >> 8) & 0xff))


class FakeUART:
    def __init__(self):
        self.data = b""

    def any(self):
        return len(self.data)

    def read(self, count):
        chunk = self.data[:count]
        self.data = self.data[count:]
        return chunk


class LD2450Tests(unittest.TestCase):
    def make_frame(self):
        frame = bytearray(FRAME_HEADER)
        frame.extend(bytes(24))
        frame.extend(FRAME_FOOTER)
        return bytes(frame)

    def test_decode_signed_coordinates_and_empty_slots(self):
        frame = bytearray(FRAME_HEADER)
        frame.extend(word(0x8000 | 125))
        frame.extend(word(2400))
        frame.extend(word(35))
        frame.extend(word(70))
        frame.extend(word(125))
        frame.extend(word(0x8000 | 1800))
        frame.extend(word(0x8000 | 12))
        frame.extend(word(80))
        frame.extend(bytes(8))
        frame.extend(FRAME_FOOTER)

        self.assertEqual(decode_frame(frame), [(125, -2400, -35, 70),
                                               (-125, 1800, 12, 80)])

    def test_decode_manual_protocol_example(self):
        frame = bytearray(FRAME_HEADER)
        frame.extend(word(0x030e))
        frame.extend(word(0x86b1))
        frame.extend(word(0x0010))
        frame.extend(word(0x0140))
        frame.extend(bytes(16))
        frame.extend(FRAME_FOOTER)

        self.assertEqual(decode_frame(frame), [(-782, 1713, -16, 320)])

    def test_reader_discards_noise_and_accepts_fragmented_frame(self):
        uart = FakeUART()
        reader = LD2450Reader(uart)
        frame = self.make_frame()
        uart.data = b"noise before header" + frame[:11]

        self.assertIsNone(reader.poll())
        uart.data = frame[11:]
        self.assertEqual(reader.poll(), [])


class AimingTests(unittest.TestCase):
    def test_forward_level_target(self):
        yaw, pitch = solve_angles(0, 2000, 0, (0, 0, 0), (0, 0, 0))
        self.assertAlmostEqual(yaw, 0)
        self.assertAlmostEqual(pitch, 0)

    def test_right_and_above_target(self):
        yaw, pitch = solve_angles(1000, 1000, 1000,
                                  (0, 0, 0), (0, 0, 0))
        self.assertAlmostEqual(yaw, 45)
        self.assertAlmostEqual(pitch, 35.26438968)


class TargetSelectorTests(unittest.TestCase):
    def test_empty_frame_does_not_discard_target_continuity(self):
        selector = TargetSelector(300, 6000)
        first = (100, 2000, 0, 50)
        nearby = (120, 2050, 0, 50)
        farther = (-1200, 3000, 0, 50)

        self.assertEqual(selector.choose([first]), first)
        self.assertIsNone(selector.choose([]))
        self.assertEqual(selector.choose([farther, nearby]), nearby)


class TMC2209Tests(unittest.TestCase):
    def test_tmc_uart_crc_is_stable(self):
        self.assertEqual(uart_crc(bytes((0x05, 0x00, 0x06))), 0x6f)

    def test_read_and_write_use_the_selected_slave_address(self):
        self.assertEqual(build_read_packet(0, 0x06),
                         bytes((0x05, 0x00, 0x06, 0x6f)))
        packet = build_write_packet(0, 0x10, 0x00080d06)
        self.assertEqual(packet[:7], bytes((0x05, 0x00, 0x90, 0x00,
                                           0x08, 0x0d, 0x06)))
        self.assertEqual(packet[7], uart_crc(packet[:7]))

    def test_initial_current_codes_are_conservative(self):
        run_cs = current_cs_for_rms(770, 0.11, 0.325)
        hold_cs = current_cs_for_rms(390, 0.11, 0.325)
        self.assertEqual(run_cs, 13)
        self.assertEqual(hold_cs, 6)

    def test_gconf_selects_uart_scaling_and_register_microsteps(self):
        self.assertEqual(gconf_for_uart(0x201), 0x2c0)

    def test_current_code_is_clamped_to_register_range(self):
        self.assertEqual(current_cs_for_rms(0, 0.11, 0.325), 0)
        self.assertEqual(current_cs_for_rms(5000, 0.11, 0.325), 31)


if __name__ == "__main__":
    unittest.main()