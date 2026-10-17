import math
import unittest
from unittest.mock import patch

import config
from aiming import solve_angles
from ld2450 import (FRAME_FOOTER, FRAME_HEADER, LD2450Reader, decode_frame,
                    decode_frame_slots)
from tmc2209 import (REG_IOIN, TMC2209, build_read_packet, build_write_packet,
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


class FakeRMT:
    def __init__(self):
        self.dispatched_pulses = []
        self._wait_done_val = True
        self._active_val = False
        self.deinited = False

    def write_pulses(self, pulses, data=1):
        self.dispatched_pulses.append((pulses, data))
        self._wait_done_val = False
        self._active_val = True

    def wait_done(self, timeout=0):
        return self._wait_done_val

    def active(self, val=None):
        if val is None:
            return self._active_val
        if not val:
            self._active_val = False
            self._wait_done_val = True

    def deinit(self):
        self.deinited = True


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

    def test_offset_pan_pivot(self):
        # Target at (1000, 2000), motor pivot offset at (200, 100)
        # dx = 1000 - 200 = 800, dy = 2000 - 100 = 1900
        yaw, _ = solve_angles(1000, 2000, 0, (200, 100, 0), (0, 0, 0))
        expected_yaw = math.degrees(math.atan2(800, 1900))
        self.assertAlmostEqual(yaw, expected_yaw)

    def test_z_offset_does_not_affect_yaw_for_planar_pan(self):
        # Because yaw is planar atan2(dx, dy), any Z height difference
        # between sensor and pivot does not change the pan yaw angle.
        yaw_no_z, _ = solve_angles(1000, 2000, 0, (200, 100, 0), (0, 0, 0))
        yaw_with_z, _ = solve_angles(1000, 2000, 0, (200, 100, 450), (0, 0, 0))
        self.assertAlmostEqual(yaw_no_z, yaw_with_z)


class TrackerConfigTests(unittest.TestCase):
    def test_trackers_have_independent_pivot_offsets(self):
        self.assertGreater(len(config.TRACKERS), 0)
        for tracker in config.TRACKERS:
            self.assertIn("pan_pivot_x_mm", tracker)
            self.assertIn("pan_pivot_y_mm", tracker)
            self.assertIsInstance(tracker["pan_pivot_x_mm"], (int, float))
            self.assertIsInstance(tracker["pan_pivot_y_mm"], (int, float))


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

    def test_micro_hysteresis_deadband_prevents_hunting_and_allows_smooth_tracking(self):
        # 2 microsteps at 1/8 microstepping is 0.45 deg; 1 microstep is 0.225 deg.
        start_deg = 0.45
        stop_deg = 0.0
        # 1 microstep error at rest: does not activate (prevents hunting over rounding noise)
        self.assertFalse(update_deadband(0.225, False, start_deg, stop_deg))
        # 2 microsteps error at rest: activates motion
        self.assertTrue(update_deadband(0.45, False, start_deg, stop_deg))
        # Once active, tracking continues smoothly even when error drops to 1 microstep
        self.assertTrue(update_deadband(0.225, True, start_deg, stop_deg))
        # Deactivates only when error drops to stop threshold (0.0 deg)
        self.assertFalse(update_deadband(0.0, True, start_deg, stop_deg))

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
        # Empty slot 1 with a target in slot 2: tracker 1 idles, tracker 2 works.
        first, second = choose_two_targets([None, nearest, None],
                                           300, 6000, "fixed_slot")
        self.assertIsNone(first)
        self.assertEqual(second, nearest)


class StepperTests(unittest.TestCase):
    def test_enable_pin_controlled_explicitly_not_toggled_during_motion(self):
        step_pin = FakePin()
        direction_pin = FakePin()
        enable_pin = FakePin()
        with patch("stepper.time.ticks_us", side_effect=(0, 10000, 20000),
                   create=True), patch(
                       "stepper.time.ticks_diff",
                       side_effect=lambda now, then: now - then,
                       create=True):
            # Active-low enable: disabled = 1, enabled = 0
            axis = StepperAxis(step_pin, direction_pin, enable_pin, 3200,
                               -60, 60, 100, enable_active_low=True,
                               initial_degrees=0)
            self.assertEqual(enable_pin.value(), 1)
            self.assertFalse(axis.enabled)

            # Enabling driver explicitly
            axis.set_enabled(True)
            self.assertEqual(enable_pin.value(), 0)
            self.assertTrue(axis.enabled)

            # set_target_degrees must NOT toggle EN
            axis.set_target_degrees(1)
            self.assertEqual(enable_pin.value(), 0)
            self.assertTrue(axis.enabled)

            # run() stepping must NOT toggle EN
            axis.run()
            self.assertEqual(enable_pin.value(), 0)
            self.assertTrue(axis.enabled)

            # When reaching target, run() must NOT toggle EN off
            axis.set_target_degrees(0)
            axis.run()
            self.assertEqual(enable_pin.value(), 0)
            self.assertTrue(axis.enabled)

            # Disabling driver explicitly
            axis.set_enabled(False)
            self.assertEqual(enable_pin.value(), 1)
            self.assertFalse(axis.enabled)

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

    def test_rmt_burst_generation_and_slice_dispatch(self):
        step_pin = FakePin()
        direction_pin = FakePin()
        enable_pin = FakePin()
        fake_rmt = FakeRMT()

        with patch("stepper.time.ticks_us", return_value=1000, create=True), patch(
            "stepper.time.ticks_diff", side_effect=lambda now, then: now - then, create=True
        ):
            # 3200 steps/rev (1/16 microstepping), max 6400 pps, slice 15 ms
            axis = StepperAxis(
                step_pin, direction_pin, enable_pin, 3200,
                -60, 60, 6400, enable_active_low=True,
                initial_degrees=0.0, rmt=fake_rmt, burst_slice_ms=15
            )
            # Target 10 degrees -> 10 * (3200/360) = 89 steps
            axis.set_target_degrees(10.0)
            self.assertEqual(axis.target_steps, 89)

            # run() dispatches first slice: at 6400 pps, 15ms slice = (6400 * 15)//1000 = 96 steps
            # Since 89 < 96, entire 89 steps fit into one slice
            axis.run()
            self.assertEqual(len(fake_rmt.dispatched_pulses), 1)
            pulses, data = fake_rmt.dispatched_pulses[0]
            self.assertEqual(data, 1)
            self.assertEqual(len(pulses), 89 * 2)
            # High pulse is 10us
            self.assertEqual(pulses[0], 10)
            self.assertTrue(axis.is_busy())

            # Mark pulse train done in RMT
            fake_rmt._wait_done_val = True
            self.assertFalse(axis.is_busy())
            self.assertEqual(axis.position_steps, 89)

    def test_rmt_burst_handles_float_step_rate_and_targets(self):
        step_pin = FakePin()
        direction_pin = FakePin()
        enable_pin = FakePin()
        fake_rmt = FakeRMT()

        with patch("stepper.time.ticks_us", return_value=1000, create=True), patch(
            "stepper.time.ticks_diff", side_effect=lambda now, then: now - then, create=True
        ):
            axis = StepperAxis(
                step_pin, direction_pin, enable_pin, 3200,
                -60, 60, 6400, enable_active_low=True,
                initial_degrees=0.0, rmt=fake_rmt, burst_slice_ms=15
            )
            # Pass float step rate (e.g. from slew calculation)
            axis.set_step_rate(35.5)
            axis.set_target_degrees(5.3)
            # Must not raise TypeError: unsupported types for __mul__: 'tuple', 'float'
            axis.run()
            self.assertTrue(axis.is_busy())
            pulses, data = fake_rmt.dispatched_pulses[0]
            self.assertIsInstance(len(pulses), int)

    def test_rmt_position_tracking_during_flight(self):
        step_pin = FakePin()
        direction_pin = FakePin()
        enable_pin = FakePin()
        fake_rmt = FakeRMT()

        # Step rate = 1000 pps -> step period = 1000 us
        with patch("stepper.time.ticks_us", side_effect=[1000, 1000, 3500, 15000], create=True), patch(
            "stepper.time.ticks_diff", side_effect=lambda now, then: now - then, create=True
        ):
            axis = StepperAxis(
                step_pin, direction_pin, enable_pin, 3200,
                -60, 60, 1000, enable_active_low=True,
                initial_degrees=0.0, rmt=fake_rmt, burst_slice_ms=15
            )
            axis.set_step_rate(1000)
            axis.set_target_degrees(1.0)  # ~9 steps

            # Dispatch burst at t=1000
            axis.run()
            self.assertTrue(axis.is_busy())

            # At t=3500 (elapsed = 2500 us, period = 1000 us -> 2 steps completed in flight)
            self.assertEqual(axis.position_steps, 2)

            # RMT finishes
            fake_rmt._wait_done_val = True
            axis.run()
            self.assertFalse(axis.is_busy())
            self.assertEqual(axis.position_steps, axis.target_steps)

    def test_rmt_stop_and_deinit(self):
        step_pin = FakePin()
        direction_pin = FakePin()
        enable_pin = FakePin()
        fake_rmt = FakeRMT()

        with patch("stepper.time.ticks_us", return_value=1000, create=True), patch(
            "stepper.time.ticks_diff", side_effect=lambda now, then: now - then, create=True
        ):
            axis = StepperAxis(
                step_pin, direction_pin, enable_pin, 3200,
                -60, 60, 6400, enable_active_low=True,
                initial_degrees=0.0, rmt=fake_rmt
            )
            axis.set_enabled(True)
            axis.set_target_degrees(5.0)
            axis.run()
            self.assertTrue(axis.is_busy())

            axis.stop()
            self.assertFalse(axis.is_busy())

            axis.deinit()
            self.assertTrue(fake_rmt.deinited)
            self.assertFalse(axis.enabled)


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

    def test_read_register_success_on_retry(self):
        class MockTMCUART:
            def __init__(self, responses):
                self.responses = list(responses)
                self.written = []
                self.buf = bytearray()

            def write(self, packet):
                self.written.append(bytes(packet))
                if self.responses:
                    self.buf.extend(self.responses.pop(0))

            def any(self):
                return len(self.buf)

            def read(self, count=None):
                if count is None:
                    count = len(self.buf)
                chunk = bytes(self.buf[:count])
                self.buf = self.buf[count:]
                return chunk

        read_echo = bytes((0x05, 0x00, 0x06, 0x6f))
        reply = bytearray((0x05, 0xff, 0x06, 0x21, 0x00, 0x00, 0x41))
        reply.append(uart_crc(reply))

        mock_uart = MockTMCUART([read_echo, read_echo + reply])
        driver = TMC2209(mock_uart, slave_address=0, timeout_ms=10)
        value = driver.read_register(REG_IOIN, retries=2)
        self.assertEqual(value, 0x21000041)
        self.assertEqual(len(mock_uart.written), 2)

    def test_read_register_fails_after_retries_exhausted(self):
        class FailingUART:
            def __init__(self):
                self.buf = bytearray()
            def write(self, packet):
                self.buf.extend(bytes((0x05, 0x00, 0x06, 0x6f)))
            def any(self):
                return len(self.buf)
            def read(self, count=None):
                if count is None:
                    count = len(self.buf)
                chunk = bytes(self.buf[:count])
                self.buf = self.buf[count:]
                return chunk

        driver = TMC2209(FailingUART(), slave_address=0, timeout_ms=5)
        with self.assertRaises(OSError) as ctx:
            driver.read_register(REG_IOIN, retries=1)
        self.assertIn("no valid TMC2209 UART response", str(ctx.exception))
        self.assertIn("0500066f", str(ctx.exception))


if __name__ == "__main__":
    unittest.main()