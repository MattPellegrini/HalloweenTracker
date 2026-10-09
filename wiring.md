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
| Pan driver UART RX  |         22 | Shared BTT RX/UART bus (direct)           |
| Pan driver UART TX  |         23 | Shared BTT RX/UART bus through 1 kOhm     |

## Connection sketch

```text
                         USB (logic power/programming)
                                   |
                         +---------v----------+
                         | uPesy ESP32-WROOM  |
                         |                    |
LD2450 TX -------------->| GPIO16 (UART2 RX)  |
LD2450 RX <--------------| GPIO17 (UART2 TX)  |  optional
LD2450 GND --------------| GND                |
LD2450 VCC <-------------| 5V/VIN*            |
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
             motor PSU +12V  +------------------+    |
             motor PSU GND -------------------------+
                       |                         |
                 Pan NEMA17                 Tilt NEMA17

ESP32 GPIO13 --[level shifter recommended]--> DIN [Eye 1] DOUT --> DIN [Eye 2]
LED 5V supply --------------------------------------> both LED +5V pins
LED GND --------------------------------------------> both LED GND pins
```

`*` Power the LD2450 from board 5 V/VIN only if that pin can supply the
module's required current. Check the radar board documentation. All grounds
must be common: ESP32, radar, TMC2209 logic grounds, motor supply, and LED
supply. Do not power the NEMA17 motors from the ESP32 or USB supply. Use a
separate motor supply matched to the motor/driver setup, and connect its ground
to ESP32 ground.

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
- Join each motor's two coil pairs to A1/A2 and B1/B2. Identify pairs with a
  meter; never plug or unplug a motor while its driver is powered.
- `EN/ENN` is normally active low; confirm this on the carrier. Verify the
  selected GPIOs against the exact uPesy board revision before wiring.
- Do not insert this BTT carrier into a solderless breadboard in the usual
  way: its extra `VREF` and `DIAG` pins can land in the same connected strip as
  `EN`, shorting all three. Leave `VREF` and `DIAG` unconnected for this
  project. Use individual jumper wires or a socket/adapter that keeps those
  pins electrically separate; do not trim pins while powered.
- In the BTT V1.3 manual's factory-default mode, the UART bus is on the module's
  `RX` pad. Connect that pad to ESP32 GPIO22 (UART receive) directly and to
  GPIO23 (UART transmit) through a 1 kOhm series resistor. Leave the module's
  separate `TX` pad, `CLK`, `MS1`, and `MS2` unconnected. Do not bridge R10; the
  manual shows that as an alternate pin assignment. DIAG is reserved for future
  homing and is not used yet.
- Keep motor VM off until UART readback confirms the current and microstep
  settings. Never unplug the motor while the driver is powered.

## TMC2209 microsteps

The code currently assumes **1/16 microstepping** (`MICROSTEPS = 16` in
`config.py`), but that constant does not set the BTT driver's resolution. The
V1.3 manual lists 256-microstep capability and UART as the default mode; it
does not document a standalone MS1/MS2 truth table or state the active
resolution at power-up. Do not assume that leaving MS1/MS2 open gives 1/16.

Run `pan_driver_setup.py` with the motor unplugged at its motor-end connector
and the driver powered from a suitable 12 V VM supply. It first checks UART
response, then sets about 0.77 A RMS run current, about 0.39 A RMS hold current,
and 1/16 microsteps. It scans UART baud rates and addresses; this module replied
at 57600 baud, address 0. GCONF and CHOPCONF are read back. Writes to
GLOBALSCALER and IHOLD_IRUN are verified through IFCNT because IHOLD_IRUN is
not read back by this setup. It never commands motor steps. Stop if the initial
UART read fails. These register values are volatile and reset when driver
power is removed. The main skull controller does not yet configure the driver
at startup, so do not run the full tracker until startup configuration is
integrated.

For a 1.8 degree motor at 1/16, the driver receives 3,200 STEP pulses per
revolution. The TMC2209's interpolation setting does not change that input
pulse count.

See the supplied [BTT TMC2209 V1.3 manual](manuals/BIGTREETECH%20TMC2209%20V1.3%20User%20Manual.pdf),
especially pages 5-7, for the carrier pinout and UART pad diagrams.

For a 1.8 degree motor at 1/16 microstepping, the driver receives 3,200 STEP
pulses per revolution. Change `MICROSTEPS` if the driver is configured
differently. TMC2209 interpolation can smooth motion internally but does not
change the STEP pulse count expected by this software.
