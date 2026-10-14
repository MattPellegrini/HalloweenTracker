import unittest
from unittest.mock import patch

from aiming import solve_angles
from ld2450 import (FRAME_FOOTER, FRAME_HEADER, LD2450Reader, decode_frame,
                    decode_frame_slots)
from tmc2209 import (build_read_packet, build_write_packet,
                     current_cs_for_rms, gconf_for_uart, uart_crc)
from stepper import StepperAxis
from tracking import (TargetSelector, choose_two_targets, slew_step_rate,
                      smooth_toward_target, tracking_step_rate,
                      update_deadband)


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


class FakePin:
    def __init__(self):
        self.level = 0

    def value(self, level=None):
        if level is None:
            return self.level
        self.level = level


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
        self.assertEqual(decode_frame_slots(frame),
                         [(125, -2400, -35, 70), (-125, 1800, 12, 80), None])

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
        self.assertEqual(reader.poll(), [None, None, None])


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

    def test_tracking_rate_eases_out_near_target(self):
        steps_per_degree = 3200 / 360
        near = tracking_step_rate(1.5, 0, 40, 1600, 25, 1.5, 0.8,
                                  steps_per_degree, 0.65)
        middle = tracking_step_rate(13.25, 0, 40, 1600, 25, 1.5, 0.8,
                                    steps_per_degree, 0.65)
        far = tracking_step_rate(25, 0, 40, 1600, 25, 1.5, 0.8,
                                 steps_per_degree, 0.65)
        self.assertEqual(near, 40)
        self.assertGreater(middle, near)
        self.assertEqual(far, 1600)

    def test_feedforward_is_gated_for_small_errors(self):
        steps_per_degree = 3200 / 360
        small = tracking_step_rate(1.5, 45, 40, 1600, 25, 1.5, 0.8,
                                   steps_per_degree, 0.65)
        # Below the 3-degree gate, feed-forward is ignored: rate stays at the
        # minimum despite the high bearing rate.
        self.assertEqual(small, 40)
        # Just above the gate, feed-forward contributes 45 * 8.89 * 0.65 = 260
        # pps while the error term for a 3.0-degree error is still small.
        above_gate = tracking_step_rate(3.0, 45, 40, 1600, 25, 1.5, 0.8,
                                        steps_per_degree, 0.65)
        self.assertEqual(above_gate, 260)

    def test_deadband_has_separate_start_and_stop_thresholds(self):
        self.assertFalse(update_deadband(2.4, False, 2.5, 1.5))
        self.assertTrue(update_deadband(2.5, False, 2.5, 1.5))
        self.assertTrue(update_deadband(1.6, True, 2.5, 1.5))
        self.assertFalse(update_deadband(1.5, True, 2.5, 1.5))

    def test_step_rate_slew_limits_acceleration_and_deceleration(self):
        self.assertEqual(slew_step_rate(100, 800, 100, 2400), 340)
        self.assertEqual(slew_step_rate(800, 100, 250, 2400), 200)
        self.assertEqual(slew_step_rate(100, 180, 50, 2400), 180)
        self.assertEqual(slew_step_rate(100, 800, 0, 2400), 100)

    def test_target_smoothing_glides_between_measurements(self):
        self.assertEqual(smooth_toward_target(None, 10.0, 40, 120), 10.0)
        self.assertAlmostEqual(smooth_toward_target(0.0, 12.0, 60, 120), 6.0)
        self.assertEqual(smooth_toward_target(0.0, 12.0, 0, 120), 0.0)
        self.assertEqual(smooth_toward_target(0.0, 12.0, 40, 0), 12.0)


class TwoTargetSelectionTests(unittest.TestCase):
    def test_nearest_two_are_assigned_in_order(self):
        far = (-2000, 4000, 0, 50)
        nearest = (100, 1000, 0, 50)
        middle = (-500, 2000, 0, 50)
        first, second = choose_two_targets([far, nearest, middle],
                                           300, 6000, "nearest_two")
        self.assertEqual(first, nearest)
        self.assertEqual(second, middle)

    def test_missing_second_target_yields_none(self):
        nearest = (100, 1000, 0, 50)
        first, second = choose_two_targets([nearest], 300, 6000,
                                           "nearest_two")
        self.assertEqual(first, nearest)
        self.assertIsNone(second)
        first, second = choose_two_targets([], 300, 6000, "nearest_two")
        self.assertIsNone(first)
        self.assertIsNone(second)

    def test_out_of_range_and_behind_targets_are_ignored(self):
        behind = (100, -1000, 0, 50)
        too_close = (10, 100, 0, 50)
        too_far = (0, 9000, 0, 50)
        valid = (300, 1500, 0, 50)
        first, second = choose_two_targets([behind, too_close, too_far,
                                            valid], 300, 6000,
                                           "nearest_two")
        self.assertEqual(first, valid)
        self.assertIsNone(second)

    def test_fixed_slot_mode_uses_slot_positions(self):
        nearest = (100, 1000, 0, 50)
        farther = (-500, 3000, 0, 50)
        # Slot order differs from distance order; fixed mode must keep it.
        first, second = choose_two_targets([farther, nearest, None],
                                           300, 6000, "fixed_slot")
        self.assertEqual(first, farther)
        self.assertEqual(second, nearest)
        # Empty slot 1 with a target in slot 2: skull 1 idles, skull 2 works.
        first, second = choose_two_targets([None, nearest, None],
                                           300, 6000, "fixed_slot")
        self.assertIsNone(first)
        self.assertEqual(second, nearest)


class StepperTests(unittest.TestCase):
    def test_direction_can_be_inverted_for_pan_installation(self):
        step_pin = FakePin()
        direction_pin = FakePin()
        enable_pin = FakePin()
        with patch("stepper.time.ticks_us", side_effect=(0, 10000),
                   create=True), patch(
                       "stepper.time.ticks_diff",
                       side_effect=lambda now, then: now - then,
                       create=True):
            axis = StepperAxis(step_pin, direction_pin, enable_pin, 3200,
                               -60, 60, 100, initial_degrees=0,
                               direction_inverted=True)
            axis.set_target_degrees(1)
            axis.run()

        self.assertEqual(direction_pin.value(), 0)
        self.assertEqual(axis.position_steps, 1)


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