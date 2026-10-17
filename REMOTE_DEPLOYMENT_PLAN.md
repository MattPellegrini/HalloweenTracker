# Remote Deployment & Wireless Serial Monitoring Plan

Updated: 2026-10-17.

This document outlines options and architectural trade-offs for deploying code and reading live serial telemetry remotely on the dual-skull Halloween tracker, allowing testing in large spaces without tethering over USB.

---

## 1. Trade-off & Performance Comparison

Due to the hard real-time requirements of the tracker (50 Hz control loop, 20 ms cycle budget, 256,000 baud radar packet streaming, and hardware RMT pulse generation), introducing networking has direct implications on timing and reliability.

| Dimension                   | Option A: Dedicated Wireless Bridge (Hardware-Isolated)                                          | Option B: On-Chip MicroPython WebREPL (Zero Extra Hardware)                                                                     |
| :-------------------------- | :----------------------------------------------------------------------------------------------- | :------------------------------------------------------------------------------------------------------------------------------ |
| **Tracker Codebase Impact** | **Zero changes**: No networking code in `boot.py`, `config.py`, or `main.py`.                    | **Moderate**: Requires `boot.py` network manager, `webrepl`, config secrets, and defensive guards.                              |
| **Tracker CPU Overhead**    | **0%**: Tracker MCU runs exactly as if plugged into USB.                                         | **5–15%**: WiFi ISRs, LwIP packet processing, and WebSocket framing.                                                            |
| **Tracker RAM & Heap**      | **0 bytes lost**: Full ~111 KB MicroPython heap retained.                                        | **-35 to -45 KB**: LwIP buffers reduce heap to ~65–75 KB, increasing garbage collection (GC) frequency.                         |
| **Loop Jitter & Latency**   | **None**: No network interrupts or GC spikes on the tracker.                                     | **2–6 ms GC pauses**: High risk of jitter against the 20 ms tracking budget; socket flushes can block loop.                     |
| **Radar Stream Integrity**  | **Safe**: No CPU preemption of UART2 ISR at 256,000 baud.                                        | **Risk of FIFO overrun**: GC pauses or socket stalls can overflow UART2's 128-byte FIFO without custom `rxbuf`.                 |
| **Electrical / RF Noise**   | **Separable**: Bridge antenna can be positioned away from radar.                                 | **High RF proximity**: 350 mA WiFi TX bursts on the shared 5V buck converter can induce ripple into LD2450 24 GHz RF front-end. |
| **Tooling & Workflow**      | **Identical to USB**: `mpremote connect rfc2217://...` works with all existing scripts and REPL. | **Different**: Requires `webrepl_cli.py` or browser WebSockets; no native `mpremote mount`.                                     |
| **Setup Cost**              | Requires soldering power and 2 signal wires to a spare ESP32.                                    | Pure software; no wiring required.                                                                                              |

---

## 2. Recommended Approach: Hardware-Isolated Wireless Serial Bridge

Given your goal to keep the tracker codebase clean and eliminate any potential source of loop starvation or radar FIFO overflow, using a spare ESP32 as a dedicated wireless bridge over **UART0** is the cleanest solution.

### 2.1 Hardware Architecture & Wiring

The primary uPesy ESP32 DevKit exposes UART0 on standard programming pins:

- **GPIO1 (TX0)**: Transmits tracker output / REPL to bridge RX.
- **GPIO3 (RX0)**: Receives commands / code uploads from bridge TX.

```text
                  +---------------------------+
                  |  Primary Tracker ESP32    |
                  |  (uPesy WROOM-32E)        |
                  |                           |
                  |   GPIO1 (TX0) ------------+--------> RX (Bridge)
                  |   GPIO3 (RX0) <-----------+--------- TX (Bridge)
                  |   GND --------------------+--------- GND (Common)
                  |   VIN <-------------------+----+
                  +---------------------------+    |
                                                   |
                  +---------------------------+    |
                  |  Secondary Bridge ESP32   |    |
                  |  (RFC2217 / Serial Bridge)|    |
                  |                           |    |
                  |   5V / VIN <--------------+----+<--- Buck Converter +5V OUT
                  |   GND --------------------+--------< Buck Converter GND OUT
                  +---------------------------+
```

#### Wiring Checklist:

1. **Power Tap**: Solder bridge ESP32 `VIN` (or `5V`) and `GND` to the output terminals of the existing 5V buck converter (verify buck output is 5.0–5.1 V before connection).
2. **Serial Signals**:
   - Primary ESP32 **GPIO1 (TX0)** $\rightarrow$ Secondary ESP32 **RX** (e.g., GPIO3 or GPIO16 depending on bridge pinout).
   - Primary ESP32 **GPIO3 (RX0)** $\leftarrow$ Secondary ESP32 **TX** (e.g., GPIO1 or GPIO17).
   - Common **GND** connected directly between both boards.
3. _Optional Reset Line_: If remote hard-reboot is desired, wire a GPIO from the bridge to the primary ESP32's `EN` pin (active low).

### 2.2 Bridge Firmware Options

1. **esp-link / esp-rfc2217 (Recommended)**:
   - Implements RFC2217 (Telnet Serial Port Control) at 115200 baud.
   - Fully compatible with `mpremote`:
     ```sh
     ./venv/bin/mpremote connect rfc2217://<bridge-ip>:2217 run main.py
     ./venv/bin/mpremote connect rfc2217://<bridge-ip>:2217 repl
     ```
   - All existing `./venv/bin/mpremote` commands in `AI_HANDOFF.md` continue to work without changing a single character other than the device path.

2. **Simple TCP-to-UART Bridge**:
   - A 30-line Arduino or MicroPython script on the spare ESP32 that bridges TCP port 8888 to `UART(baudrate=115200)`.
   - On macOS, pipe via `socat` or `nc`:
     ```sh
     # Terminal serial monitoring:
     nc <bridge-ip> 8888
     ```

---

## 3. Alternative: On-Chip MicroPython WebREPL (No-Solder Option)

If you need to test before soldering the secondary board, you can enable on-chip WiFi with defensive tuning to minimize real-time loop impacts.

### 3.1 Network Bootstrap (`boot.py`)

Create `boot.py` on the ESP32 to establish WiFi station mode and start `webrepl` before `main.py` executes:

```python
import network
import time
import webrepl
import config

def connect_wifi():
    sta = network.WLAN(network.STA_IF)
    sta.active(True)
    if not sta.isconnected():
        print("Connecting to WiFi: {}...".format(config.WIFI_SSID))
        sta.connect(config.WIFI_SSID, config.WIFI_PASSWORD)
        start = time.ticks_ms()
        while not sta.isconnected():
            if time.ticks_diff(time.ticks_ms(), start) > config.WIFI_CONNECT_TIMEOUT_MS:
                print("WiFi connection timed out; continuing offline.")
                return False
            time.sleep_ms(100)
    print("WiFi connected:", sta.ifconfig())
    return True

if connect_wifi():
    webrepl.start(password=config.WEBREPL_PASSWORD)
```

### 3.2 Defensive Real-Time Mitigations

If running WebREPL on-chip, the following codebase adjustments are required to protect motor pulses and radar tracking:

1. **Enlarge LD2450 UART Buffer** (`main.py`):
   ```python
   # Default 128-byte RX buffer overflows in ~5 ms at 256k baud during a GC pause.
   radar_uart = UART(config.RADAR_UART_ID, baudrate=config.RADAR_BAUDRATE,
                     bits=8, parity=None, stop=1,
                     tx=config.RADAR_TX_PIN, rx=config.RADAR_RX_PIN,
                     rxbuf=1024)
   ```
2. **Increase Hardware RMT Slice Duration** (`config.py`):
   - Increase `RMT_BURST_SLICE_MS` from `15` to `20` or `25`.
   - Provides safety margin so a 4–6 ms network interrupt or GC sweep does not deplete the hardware pulse queue before the next slice is dispatched.
3. **Deterministic Garbage Collection**:
   - Trigger `gc.collect()` explicitly inside `Tracker.update()` only when targets expire and motors are parked, keeping active tracking free of GC pauses.

### 3.3 Host Deployment via WebREPL

Using `webrepl_cli.py` (available in MicroPython tools repository):

```sh
# Copy updated files over WiFi
python webrepl_cli.py -p <password> config.py <esp32-ip>:/config.py
python webrepl_cli.py -p <password> main.py <esp32-ip>:/main.py

# Soft reboot tracker
python -c 'import websocket; ws = websocket.create_connection("ws://<esp32-ip>:8266/"); ws.send("\x03\x04")'
```

---

## 4. Verification & Validation Checklist

Before testing in the open space, verify the remote link:

1. **Station Association**: Confirm the ESP32 associates cleanly with the venue router within 3–5 seconds from cold boot.
2. **Throughput & Echo Check**: Push a full file set (`main.py`, `config.py`, `stepper.py`) over the link and verify byte counts match.
3. **Radar Frame Rate Consistency**:
   - Stream telemetry logs while moving targets in front of the LD2450.
   - Verify radar delta times (`+dt_ms`) remain tightly centered around ~100 ms (~10 Hz) with zero frame drops or header corruptions.
4. **Motion Continuity**:
   - Command continuous slewed sweeps (35 $\rightarrow$ 6400 $\rightarrow$ 35 pps) while network packets are being exchanged.
   - Listen and observe for any audible ticking, hitching, or motor stall that would indicate RMT slice starvation.
5. **Offline Fallback**:
   - Power up the assembly with the WiFi router disconnected or out of range.
   - Verify the prop starts tracking normally after the connection timeout without hanging.
