"""Minimal TMC2209 single-wire UART access for MicroPython and host tests."""

import time


REG_GCONF = 0x00
REG_GSTAT = 0x01
REG_IFCNT = 0x02
REG_IOIN = 0x06
REG_GLOBALSCALER = 0x0b
REG_IHOLD_IRUN = 0x10
REG_CHOPCONF = 0x6c
REG_DRV_STATUS = 0x6f
CHOPCONF_VSENSE_BIT = 1 << 17
CHOPCONF_MRES_SHIFT = 24
CHOPCONF_MRES_MASK = 0x0f << CHOPCONF_MRES_SHIFT
GCONF_I_SCALE_ANALOG = 1 << 0
GCONF_PDN_DISABLE = 1 << 6
GCONF_MSTEP_REG_SELECT = 1 << 7


def gconf_for_uart(value):
    """Use UART current scaling and register-selected microstep resolution."""
    value &= ~GCONF_I_SCALE_ANALOG
    return value | GCONF_PDN_DISABLE | GCONF_MSTEP_REG_SELECT


def uart_crc(data):
    """Return TMC UART CRC-8 (poly 0x07, processing each byte LSB-first)."""
    crc = 0
    for value in data:
        current = value
        for _ in range(8):
            if ((crc >> 7) ^ (current & 1)):
                crc = ((crc << 1) ^ 0x07) & 0xff
            else:
                crc = (crc << 1) & 0xff
            current >>= 1
    return crc


def build_read_packet(slave_address, address):
    packet = bytearray((0x05, slave_address, address & 0x7f))
    packet.append(uart_crc(packet))
    return packet


def build_write_packet(slave_address, address, value):
    packet = bytearray((0x05, slave_address, address | 0x80,
                        (value >> 24) & 0xff,
                        (value >> 16) & 0xff,
                        (value >> 8) & 0xff,
                        value & 0xff))
    packet.append(uart_crc(packet))
    return packet


def current_cs_for_rms(current_ma, sense_resistor_ohms=0.11,
                       vsense_volts=0.325, global_scaler=0):
    """Convert desired RMS milliamps to the TMC2209 5-bit CS setting.

    A GLOBALSCALER value of zero represents 256. Assumes the driver's
    internal sense-voltage selection is 0.325 V (VSENSE=0).
    """
    scaler = 256 if global_scaler == 0 else global_scaler
    amps = current_ma / 1000.0
    raw_cs = (amps * 32.0 * 1.41421356237 *
              (sense_resistor_ohms + 0.02) * 256.0 /
              (vsense_volts * scaler))
    return max(0, min(31, int(raw_cs + 0.5) - 1))


class TMC2209:
    def __init__(self, uart, slave_address=0, timeout_ms=80):
        self.uart = uart
        self.slave_address = slave_address
        self.timeout_ms = timeout_ms

    def _send(self, packet):
        self.uart.write(packet)

    def write_register(self, address, value):
        packet = build_write_packet(self.slave_address, address, value)
        self._send(packet)
        time.sleep_ms(2)

    def write_register_confirmed(self, address, value):
        before = self.read_register(REG_IFCNT)
        self.write_register(address, value)
        after = self.read_register(REG_IFCNT)
        expected = (before + 1) & 0xff
        if after != expected:
            raise OSError("TMC2209 did not accept write to register "
                          "0x{:02x}: IFCNT {} -> {}, expected {}".format(
                              address, before, after, expected))
        return after

    def read_register(self, address):
        while self.uart.any():
            self.uart.read()

        request = build_read_packet(self.slave_address, address)
        self._send(request)

        response = bytearray()
        received = bytearray()
        start = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), start) < self.timeout_ms:
            available = self.uart.any()
            if available:
                chunk = self.uart.read(available)
                received.extend(chunk)
                response.extend(chunk)
                while len(response) >= 8:
                    marker = response.find(b"\x05")
                    if marker < 0:
                        response = bytearray()
                        break
                    if marker:
                        response = bytearray(response[marker:])
                    if len(response) < 8:
                        break
                    candidate = response[:8]
                    if (candidate[2] == (address & 0x7f) and
                            uart_crc(candidate[:7]) == candidate[7]):
                        return ((candidate[3] << 24) |
                                (candidate[4] << 16) |
                                (candidate[5] << 8) | candidate[6])
                    response = bytearray(response[1:])
            time.sleep_ms(1)
        raise OSError("no valid TMC2209 UART response for register 0x{:02x} "
                  "(raw RX={})".format(address, received.hex()))

    def configure_current(self, run_current_ma, hold_current_ma,
                          sense_resistor_ohms=0.11, vsense_volts=0.325):
        run_cs = current_cs_for_rms(run_current_ma, sense_resistor_ohms,
                                    vsense_volts)
        hold_cs = current_cs_for_rms(hold_current_ma, sense_resistor_ohms,
                                     vsense_volts)
        hold_delay = 8
        value = (hold_delay << 16) | (run_cs << 8) | hold_cs
        ifcnt = self.write_register_confirmed(REG_IHOLD_IRUN, value)
        return run_cs, hold_cs, value, ifcnt

    def configure_uart_mode(self):
        current = self.read_register(REG_GCONF)
        updated = gconf_for_uart(current)
        self.write_register(REG_GCONF, updated)
        actual = self.read_register(REG_GCONF)
        required = GCONF_PDN_DISABLE | GCONF_MSTEP_REG_SELECT
        if (actual & (GCONF_I_SCALE_ANALOG | required)) != required:
            raise OSError("GCONF UART/current mode readback mismatch: "
                          "0x{:08x}".format(actual))
        return actual

    def configure_microsteps(self, microsteps):
        resolutions = {256: 0, 128: 1, 64: 2, 32: 3, 16: 4, 8: 5,
                   4: 6, 2: 7, 1: 8}
        if microsteps not in resolutions:
            raise ValueError("unsupported microstep resolution")
        chopconf = self.read_register(REG_CHOPCONF)
        mres = resolutions[microsteps]
        updated = ((chopconf & ~(CHOPCONF_MRES_MASK | CHOPCONF_VSENSE_BIT)) |
                   (mres << CHOPCONF_MRES_SHIFT))
        self.write_register(REG_CHOPCONF, updated)
        actual = self.read_register(REG_CHOPCONF)
        if (actual & CHOPCONF_MRES_MASK) != (mres << CHOPCONF_MRES_SHIFT):
            raise OSError("CHOPCONF MRES readback mismatch")
        return actual