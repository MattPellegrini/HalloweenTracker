"""Minimal TMC2209 single-wire UART driver for MicroPython and host test suites.

Physical & Electrical Architecture:
  - Bus Topology: Half-duplex single-wire UART. Multiple drivers share the bus.
  - Wiring: ESP32 RX connects directly to the shared PDN_UART bus line. ESP32 TX
    connects to the same line through a 1 kOhm series resistor.
  - Slave Addressing: Up to 4 drivers on one bus, selected by strapping pins MS1 & MS2:
      Address 0: MS1=GND, MS2=GND  (Skull 1)
      Address 1: MS1=VIO, MS2=GND  (Skull 2)
      Address 2: MS1=GND, MS2=VIO
      Address 3: MS1=VIO, MS2=VIO
  - Echo Handling: Because TX and RX are wired to the same bus line, every byte
    transmitted by the ESP32 is echoed back into its own RX FIFO. The parser
    strips echoed bytes and synchronizes on the driver's reply packet.

CRC & Framing:
  - CRC-8 with polynomial 0x07 (x^8 + x^2 + x + 1), evaluated LSB-first.
  - Read request  (4 bytes): [0x05 (Sync), SlaveAddr, RegAddr & 0x7F, CRC]
  - Write request (8 bytes): [0x05 (Sync), SlaveAddr, RegAddr | 0x80, Data3, Data2, Data1, Data0, CRC]
  - Read reply    (8 bytes): [0x05 (Sync), MasterAddr(0xFF), RegAddr & 0x7F, Data3, Data2, Data1, Data0, CRC]
"""

import time

# Host compatibility fallback for testing under standard CPython
if not hasattr(time, "ticks_ms"):
    time.ticks_ms = lambda: int(time.time() * 1000)
if not hasattr(time, "ticks_diff"):
    time.ticks_diff = lambda a, b: a - b
if not hasattr(time, "sleep_ms"):
    time.sleep_ms = lambda ms: time.sleep(ms / 1000.0)


# TMC2209 Register Addresses
REG_GCONF = 0x00         # Global configuration flags
REG_GSTAT = 0x01         # Global status flags (reset, drv_err, uv_cp)
REG_IFCNT = 0x02         # Interface transmission counter (increments on valid writes)
REG_IOIN = 0x06          # Direct pin inputs & driver silicon version
REG_GLOBALSCALER = 0x0b  # Global current scaling (0 = 256/256 full scale)
REG_IHOLD_IRUN = 0x10    # Standstill hold current & active run current
REG_CHOPCONF = 0x6c      # Chopper and microstep configuration (MRES)
REG_DRV_STATUS = 0x6f    # Driver error flags, actual current scale, standstill flag

# Register Bitmasks and Flags
CHOPCONF_VSENSE_BIT = 1 << 17     # 0 = standard 0.325 V sense voltage, 1 = 0.180 V
CHOPCONF_MRES_SHIFT = 24          # Microstep resolution field (bits 27..24)
CHOPCONF_MRES_MASK = 0x0f << CHOPCONF_MRES_SHIFT
GCONF_I_SCALE_ANALOG = 1 << 0    # 1 = use VREF pot, 0 = use digital UART current
GCONF_PDN_DISABLE = 1 << 6       # 1 = disable PDN pin pull-down, use UART
GCONF_MSTEP_REG_SELECT = 1 << 7  # 1 = select microsteps from CHOPCONF.MRES, 0 = from MS1/MS2 pins


def gconf_for_uart(value):
    """Modify GCONF register value to enable UART digital control.

    Clears I_SCALE_ANALOG (bypasses the VREF potentiometer in favor of digital
    register scaling), sets PDN_DISABLE, and sets MSTEP_REG_SELECT (so microstepping
    is commanded via CHOPCONF rather than external pin strapping).
    """
    value &= ~GCONF_I_SCALE_ANALOG
    return value | GCONF_PDN_DISABLE | GCONF_MSTEP_REG_SELECT


def uart_crc(data):
    """Calculate TMC UART CRC-8 (polynomial 0x07, processing each byte LSB-first)."""
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
    """Build a 4-byte read request packet with CRC."""
    packet = bytearray((0x05, slave_address, address & 0x7f))
    packet.append(uart_crc(packet))
    return packet


def build_write_packet(slave_address, address, value):
    """Build an 8-byte write request packet with big-endian data and CRC."""
    packet = bytearray((0x05, slave_address, address | 0x80,
                        (value >> 24) & 0xff,
                        (value >> 16) & 0xff,
                        (value >> 8) & 0xff,
                        value & 0xff))
    packet.append(uart_crc(packet))
    return packet


def current_cs_for_rms(current_ma, sense_resistor_ohms=0.11,
                       vsense_volts=0.325, global_scaler=0):
    """Convert desired RMS current (mA) to the TMC2209 5-bit Current Scale (CS) code.

    Math:
      I_RMS = (CS + 1) / 32 * (V_FS / (R_SENSE + 0.02)) * (1 / sqrt(2)) * (GLOBALSCALER / 256)
    Solving for CS:
      CS = round(I_RMS * 32 * sqrt(2) * (R_SENSE + 0.02) * 256 / (V_FS * GLOBALSCALER)) - 1
    Returns an integer clamped to [0..31].
    """
    scaler = 256 if global_scaler == 0 else global_scaler
    amps = current_ma / 1000.0
    raw_cs = (amps * 32.0 * 1.41421356237 *
              (sense_resistor_ohms + 0.02) * 256.0 /
              (vsense_volts * scaler))
    return max(0, min(31, int(raw_cs + 0.5) - 1))


class TMC2209:
    """Controls a single TMC2209 stepper driver on a shared single-wire UART bus."""

    def __init__(self, uart, slave_address=0, timeout_ms=80):
        self.uart = uart
        self.slave_address = slave_address
        self.timeout_ms = timeout_ms

    def _send(self, packet):
        self.uart.write(packet)

    def write_register(self, address, value):
        """Send a write packet to the driver register."""
        packet = build_write_packet(self.slave_address, address, value)
        self._send(packet)
        time.sleep_ms(2)

    def write_register_confirmed(self, address, value):
        """Write a register and verify acceptance via the IFCNT register.

        Because several important registers (such as IHOLD_IRUN and GLOBALSCALER)
        are write-only or not directly readable on TMC2209, write acceptance is
        verified by reading the Interface Transmission Counter (IFCNT). The TMC2209
        increments IFCNT on every successfully decoded, error-free write command.
        """
        before = self.read_register(REG_IFCNT)
        self.write_register(address, value)
        after = self.read_register(REG_IFCNT)
        expected = (before + 1) & 0xff
        if after != expected:
            raise OSError("TMC2209 did not accept write to register "
                          "0x{:02x}: IFCNT {} -> {}, expected {}".format(
                              address, before, after, expected))
        return after

    def read_register(self, address, retries=2):
        """Read a 32-bit register from the driver with retry and echo-filtering."""
        last_error = None
        for attempt in range(retries + 1):
            if attempt > 0:
                time.sleep_ms(20)
            # Flush RX buffer of any leftover bytes
            while self.uart.any():
                self.uart.read()

            request = build_read_packet(self.slave_address, address)
            self._send(request)

            response = bytearray()
            received = bytearray()
            start = time.ticks_ms()
            # Accumulate UART bytes until valid 8-byte response packet arrives or timeout
            while time.ticks_diff(time.ticks_ms(), start) < self.timeout_ms:
                available = self.uart.any()
                if available:
                    chunk = self.uart.read(available)
                    received.extend(chunk)
                    response.extend(chunk)
                    # Scan for 8-byte reply: [0x05, 0xFF, reg_addr, d3, d2, d1, d0, crc]
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
                        # Verify register address match and valid CRC-8
                        if (candidate[2] == (address & 0x7f) and
                                uart_crc(candidate[:7]) == candidate[7]):
                            return ((candidate[3] << 24) |
                                    (candidate[4] << 16) |
                                    (candidate[5] << 8) | candidate[6])
                        # Candidate failed check; slide forward one byte
                        response = bytearray(response[1:])
                time.sleep_ms(1)
            last_error = OSError(
                "no valid TMC2209 UART response for register 0x{:02x} "
                "(raw RX={})".format(address, received.hex()))
        raise last_error

    def configure_current(self, run_current_ma, hold_current_ma,
                          sense_resistor_ohms=0.11, vsense_volts=0.325):
        """Set run and hold current scale registers and verify via IFCNT."""
        run_cs = current_cs_for_rms(run_current_ma, sense_resistor_ohms,
                                    vsense_volts)
        hold_cs = current_cs_for_rms(hold_current_ma, sense_resistor_ohms,
                                     vsense_volts)
        hold_delay = 8  # Delay cycles before dropping from IRUN to IHOLD on standstill
        value = (hold_delay << 16) | (run_cs << 8) | hold_cs
        ifcnt = self.write_register_confirmed(REG_IHOLD_IRUN, value)
        return run_cs, hold_cs, value, ifcnt

    def configure_uart_mode(self):
        """Configure GCONF for UART control and verify with readback."""
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
        """Configure microstep resolution (CHOPCONF.MRES) and verify with readback."""
        resolutions = {256: 0, 128: 1, 64: 2, 32: 3, 16: 4, 8: 5,
                   4: 6, 2: 7, 1: 8}
        if microsteps not in resolutions:
            raise ValueError("unsupported microstep resolution")
        chopconf = self.read_register(REG_CHOPCONF)
        mres = resolutions[microsteps]
        # Preserve chopper flags, set MRES bits, ensure VSENSE bit is 0
        updated = ((chopconf & ~(CHOPCONF_MRES_MASK | CHOPCONF_VSENSE_BIT)) |
                   (mres << CHOPCONF_MRES_SHIFT))
        self.write_register(REG_CHOPCONF, updated)
        actual = self.read_register(REG_CHOPCONF)
        if (actual & CHOPCONF_MRES_MASK) != (mres << CHOPCONF_MRES_SHIFT):
            raise OSError("CHOPCONF MRES readback mismatch")
        return actual