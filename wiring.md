# Proposed Wiring

This is a starting point for a uPesy ESP32 Low Power DevKit with an
ESP32-WROOM-32E. Confirm the pin labels against the exact board revision before
connecting anything. GPIO numbers below refer to ESP32 GPIOs, not header
positions.

## GPIO map

| Function            | ESP32 GPIO | Connect to                                |
| ------------------- | ---------: | ----------------------------------------- |
| LD2450 UART2 RX     |         16 | LD2450 TX                                 |
| LD2450 UART2 TX     |         17 | LD2450 RX (optional for receive-only use) |
| Pan STEP            |         25 | Pan TMC2209 STEP                          |
| Pan DIR             |         26 | Pan TMC2209 DIR                           |
| Pan enable          |         27 | Pan TMC2209 EN/ENN                        |
| Driver logic supply |        3V3 | Both BTT TMC2209 V1.3 VIO pins            |
| Tilt STEP           |         18 | Tilt TMC2209 STEP                         |
| Tilt DIR            |         19 | Tilt TMC2209 DIR                          |
| Tilt enable         |         21 | Tilt TMC2209 EN/ENN                       |
| NeoPixel data       |         13 | First eye LED DIN                         |
| Future pan DIAG     |         32 | Pan BTT TMC2209 V1.3 DIAG (not used yet)  |
| Future tilt DIAG    |         33 | Tilt BTT TMC2209 V1.3 DIAG (not used yet) |
| TMC driver UART RX  |         22 | Shared BTT RX/UART bus (direct)           |
| TMC driver UART TX  |         23 | Shared BTT RX/UART bus through 1 kOhm     |

## Connection sketch

```text
                         USB (programming; takes priority over VIN)
                                   |
                         +---------v----------+
                         | uPesy ESP32-WROOM  |
                         |                    |
LD2450 TX -------------->| GPIO16 (UART2 RX)  |
LD2450 RX <--------------| GPIO17 (UART2 TX)  |  optional
LD2450 GND --------------| GND                |
LD2450 VCC <-------------| external buck +5V  |
                         |                    |
Pan STEP/DIR/EN <--------| GPIO25/26/27       |
Tilt STEP/DIR/EN <-------| GPIO18/19/21       |
Eye LED DIN <------------| GPIO13             |
GPIO22 UART RX <----------+---- shared bus ----> BTT pan driver RX/UART
GPIO23 UART TX ---------[1 kOhm]---+
                         | 3V3                |------> both TMC2209 VIO
                         +----+----------+----+
                              |          |
                              |          +------ common ground
                              |                    |
                    +---------v------+    +--------v-------+
                    | Pan BTT 1.3    |    | Tilt BTT 1.3   |
                    | STEP DIR EN    |    | STEP DIR EN    |
                    | VM VIO GND     |    | VM VIO GND     |
                    +--+----+--------+    +--+----+--------+
                       |    |                  |    |
                  motor PSU +12V ------> TMC VM
                    |
                    +--------------> buck IN+
                  motor PSU GND --------> TMC GND, buck IN-, common GND
                  buck OUT+ 5V ----------> ESP32 VIN and LD2450 VCC
                  buck OUT- GND ----------> ESP32 GND and LD2450 GND
                       |                         |
                 Pan NEMA17                 Tilt NEMA17

ESP32 GPIO13 --[level shifter recommended]--> DIN [Eye 1] DOUT --> DIN [Eye 2]
LED 5V supply --------------------------------------> both LED +5V pins
LED GND --------------------------------------------> both LED GND pins
```

The uPesy ESP32 Wroom Low Power DevKit supports an external 3.5-7 V supply on
`VIN` while USB is connected. Its power path gives USB priority and disables
external VIN when USB is present; the board's `5V` pin is only powered when USB
is connected. Therefore feed the regulated buck output to `VIN`, not the
board's `5V` pin. The buck can also power the LD2450 at 5 V. All grounds must
be common: ESP32, radar, driver logic, buck, and 12 V supply. Do not power the
NEMA17 motors from the ESP32 or USB. Size the buck for the ESP32 and radar (and
any later LEDs); verify its output is 5 V before connecting it.

## Electrical notes

- ESP32 GPIOs are not 5 V tolerant. Check the LD2450 TX logic voltage; add a
  level shifter or divider if TX can exceed 3.3 V.
- Use a suitable regulated 5 V supply for the NeoPixels. A 74AHCT125 or
  74HCT14 data buffer is recommended for reliable 3.3 V-to-5 V signaling. Put
  a 330-470 ohm resistor in series with DIN and a bulk capacitor across LED
  5 V/GND near the LEDs.
- On the BTT TMC2209 V1.3, connect VM to a motor supply within the manual's
  12-28 V range, VIO to ESP32 3V3, and GND to common ground. Do not connect VM
  to ESP32 5 V. The driver exposes `STEP`, `DIR`, `EN`, `DIAG`, `MS1`, `MS2`,
  `RX`, `TX`, and `CLK`, plus motor outputs `A1/A2/B1/B2`.
- uPesy power paths are documented in the [board documentation](https://www.upesy.com/blogs/tutorials/upesy-esp32-wroom-low-power-devkit-board-documentation-version-latest): external buck 5 V goes to `VIN` (3.5-7 V allowed), not `5V`. USB may remain connected for serial/programming; it takes priority for the ESP32 board while the external buck still powers the LD2450.
- Join each motor's two coil pairs to A1/A2 and B1/B2. Identify pairs with a
  meter; never plug or unplug a motor while its driver is powered.
- `EN/ENN` is normally active low; confirm this on the carrier. Verify the
  selected GPIOs against the exact uPesy board revision before wiring.
- Do not insert this BTT carrier into a solderless breadboard in the usual
  way: its extra `VREF` and `DIAG` pins can land in the same connected strip as
  `EN`, shorting all three. Leave `VREF` and `DIAG` unconnected for this
  project. Use individual jumper wires or a socket/adapter that keeps those
  pins electrically separate; do not trim pins while powered.
- In the BTT V1.3 manual's factory-default mode, the UART bus is on each
  module's `RX` pad. Connect that pad to ESP32 GPIO22 (UART receive) directly
  and to GPIO23 (UART transmit) through a 1 kOhm series resistor. Leave each
  module's separate `TX` pad and `CLK` unconnected. Do not bridge R10; the
  manual shows that as an alternate pin assignment. DIAG is reserved for future
  homing and is not used yet.
- Both drivers use the shared UART bus. Leave pan MS1/MS2 low/open (address
  0), and strap tilt MS1 to VIO (3V3) and MS2 to GND (address 1). With
  `MSTEP_REG_SELECT` enabled, these pins select the UART address. The tilt
  diagnostics address only driver 1 and do not step the pan motor. Verify both
  addresses before enabling VM or running either motor test.
- Keep motor VM off until UART readback confirms the current and microstep
  settings. Never unplug the motor while the driver is powered.

## TMC2209 microsteps

The code currently configures **1/16 microstepping** (`MICROSTEPS = 16` in
`config.py`) through TMC2209 UART at tracker startup. The V1.3 manual lists
256-microstep capability and UART as the default mode; it does not document a
standalone MS1/MS2 truth table or state the active resolution at power-up. Do
not assume that leaving MS1/MS2 open gives the software-configured resolution.

Run `motor_driver_setup.py` with the motors unplugged at their motor-end connectors
and the drivers powered from a suitable 12 V VM supply. It checks UART
response for each tracker in `TRACKERS`, then sets each tracker's configured run current,
hold current, and microstep mode. GCONF and CHOPCONF are read back. Writes to
GLOBALSCALER and IHOLD_IRUN are verified through IFCNT because IHOLD_IRUN is
not read back by this setup. It never commands motor steps. Stop if the initial
UART read fails. These register values are volatile and reset when driver
power is removed. The main controller configures both drivers at startup.

For a 1.8 degree motor at 1/16, the driver receives 3,200 STEP pulses per
revolution. The TMC2209's interpolation setting does not change that input
pulse count.

See the supplied [BTT TMC2209 V1.3 manual](manuals/BIGTREETECH%20TMC2209%20V1.3%20User%20Manual.pdf),
especially pages 5-7, for the carrier pinout and UART pad diagrams.

For a 1.8 degree motor at 1/16 microstepping, the driver receives 3,200 STEP
pulses per revolution. Change `MICROSTEPS` only together with the TMC driver's
configured MRES. TMC2209 interpolation can smooth motion internally but does not
change the STEP pulse count expected by this software.
