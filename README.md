# VESL EMF Scanner

An automated, keyboard-controlled **1D magnetic-field scanner**. A stepper motor moves a handheld gaussmeter probe along a belt-driven rail past a magnet, and a Python app plots **magnetic flux density vs. position** live, fills a spreadsheet-style table, and saves each session as a CSV, a graph and a summary.

Built for the Villanova Engineering Service Learning (VESL) trip with [WindAid](https://www.windaid.org/) in Trujillo, Peru, to help an electrical engineering professor at Universidad Nacional de Trujillo replace manual magnet testing with a repeatable, automated scan.

<!-- Add a screenshot of the live window here:
![Live scan window](docs/screenshot.png)
-->

## Features

- **Keyboard control** of the carriage: move, speed up/down, nudge 1 mm, go to zero.
- **Automatic calibration** with two limit switches: finds the rail start (zero) and far end, and re-syncs position whenever a switch is hit.
- **Scan mode:** set a step (for example 10 mm), press one key, and the probe stops at 0, 10, 20 ... mm, waiting for each reading.
- **Live graph** of field vs. position, with a smooth curve (monotone cubic / PCHIP, so it never overshoots) or straight lines.
- **Spreadsheet-style table** that fills in as each point is recorded; magnet edges are marked.
- **Zero point (Z):** set the graph zero anywhere, for example at the center of the magnet.
- **Timestamped sessions:** each scan saves to its own folder: `scan_data.csv`, `scan_graph.png`, `scan_info.txt`.
- **Safety:** limit switches, a host watchdog, maximum travel, and the motor driver switched off when idle.
- **Automatic COM-port detection**, so it works on any laptop without knowing the port number.

## How it works

```
 keyboard --> Python app (emf_scan.py) --USB serial, 115200 baud--> Arduino Nano (src/main.cpp)
                  ^   live graph + table + CSV                          |  STEP / DIR / EN
                  |                                                     v
                  +---- position, field, state, switches (10 Hz) --- A4988 driver --> NEMA 17 --> GT2 belt --> probe
                                                                           limit switches (D12 start, D11 end)
```

- **Firmware (C++, PlatformIO):** non-blocking control loop. Counts steps for position, ramps the speed, checks the limit switches before every step, parses `g<steps>` go-to commands, and streams one line of data every 100 ms.
- **Python app:** a reader thread parses the stream, a heartbeat thread keeps the firmware's watchdog fed, and a matplotlib window shows the graph and table. Moves use absolute positions, so errors do not build up during a scan.

## Hardware

| Part | Notes |
|---|---|
| Arduino Nano (ATmega328) | Runs the firmware |
| CNC Shield V4 + A4988 driver | Stepper driver, 1/16 microstepping |
| NEMA 17 stepper motor | Moves the carriage |
| GT2 belt, 30-tooth pulley, aluminum rail | 53.333 steps per mm |
| 2 limit switches | START on D12, END on D11, wired to GND (pressed = LOW) |
| Handheld gaussmeter | Analog output 0 to ±2 V (readout is typed in for now, see below) |

### Pins (from `src/main.cpp`)

| Function | Pin |
|---|---|
| STEP | D5 |
| DIR | D2 |
| EN (active LOW) | D8 |
| START limit switch | D12 (`INPUT_PULLUP`) |
| END limit switch | D11 (`INPUT_PULLUP`) |
| Gaussmeter signal (planned) | A0 |
| 2.5 V bias node (planned) | A1 |

## Getting started

### 1. Upload the firmware

1. Install [VS Code](https://code.visualstudio.com/) and the **PlatformIO IDE** extension.
2. Open this folder in VS Code and plug in the Arduino Nano over USB.
3. Click **Upload** in the PlatformIO toolbar. Afterwards **close the PlatformIO Serial Monitor**; only one program can use the port.

### 2. Run the Python app

Python 3.9 or newer.

**Windows (easiest):** double-click `run_scan.bat`. It creates a virtual environment, installs the requirements and starts the app.

**Manually:**

```bash
python -m venv .venv
.venv\Scripts\activate          # macOS/Linux: source .venv/bin/activate
pip install -r requirements.txt
python emf_scan.py              # finds the Arduino automatically
```

Options:

```bash
python emf_scan.py COM14        # force a port
python emf_scan.py --list       # list serial ports
python emf_scan.py --step 10    # start with a 10 mm step
```

**Click the graph window** before pressing keys; keys only work while it is selected.

## Using it

Before you start: turn the gaussmeter on, set **DC-B** for a permanent magnet, pick the smallest range that does not overload, and zero it with its offset knob while the probe is far from the magnet.

1. Press **C**: the probe drives back to the START switch (zero), then forward to the END switch (rail length).
2. Press **I**, type a step in mm (for example `10`), press **ENTER**.
3. Optional: move to the magnet's center or edge and press **Z** to make it position 0.
4. Press **T**: the probe stops at each position and asks for a reading. Read the gaussmeter, type the value (use `-` for negative), press **ENTER**. It moves on by itself.
5. Watch the table fill and the curve build. Press **ENTER** to save. **SPACE** stops a scan at any time.

### Keys

| Key | Action |
|---|---|
| D / A | Move forward / backward until stopped |
| SPACE | Stop (also cancels a scan or a typing prompt) |
| W / S | Faster / slower |
| `.` / `,` | Nudge 1 mm forward / back |
| V | Go to zero |
| C | Calibrate with the limit switches |
| I | Set the distance between points |
| Z | Set the graph zero at the current position |
| T | Run an automatic scan using the step from I |
| R | Record one point here |
| N | Move one step, then record |
| M | Record a magnet **edge** point |
| U | Undo the last point |
| L | Smooth curve / straight lines |
| ENTER | Save the data and graph |
| X | Discard and close |
| F | Full screen on / off |
| H | Open the instructions (`EMF_scanner_instructions.txt`) |

## Output

Each scan creates a folder under `scans/` named with the date and time:

| File | Contents |
|---|---|
| `scan_data.csv` | Two columns: `position (mm)` and `Magnetic Field (mT)`. Positions are relative to the Z zero if one was set. Opens directly in Excel. |
| `scan_graph.png` | The graph |
| `scan_info.txt` | Edge positions, graph zero, rail range and how it was found, units, and whether readings were typed |

## Serial protocol

The firmware prints one line every 100 ms at 115200 baud:

```
time_ms,position_mm,sensor_V,field,mode,dir,step_rate,start_switch,end_switch
```

- `mode`: 0 idle, 1 moving, 2 calibrating back, 3 calibrating forward, 4 going to zero
- `dir`: +1 forward, -1 backward, 0 stopped
- switches: 1 pressed, 0 open, -1 switches disabled
- Lines starting with `#` are status messages (for example `#RANGE,0,<edge_mm>,switch`).

Commands: single characters (`d a w s c v . ,` and space), `g<steps>` + newline for an absolute go-to, and `p` as the host heartbeat.

## Safety

- The motor stops at either limit switch, and the firmware refuses to move into a pressed switch.
- **Watchdog:** the app sends a heartbeat every 0.25 s; if the motor is moving and nothing arrives for 1.5 s, it stops.
- Maximum uncalibrated travel of 1,500 mm; the rail must be at least 20 mm long to accept an edge.
- The A4988 driver is switched off 150 ms after the last step, so the motor draws no current when still (this also means no holding torque; set `RELEASE_WHEN_IDLE` to `false` in `src/main.cpp` to hold position).
- A switch hit re-syncs the position, so lost steps do not accumulate.

## Configuration

Firmware (`src/main.cpp`): `STEPS_PER_MM`, `MIN_STEP_RATE` / `MAX_STEP_RATE`, `START_PERIOD_US` / `RAMP_US` (acceleration ramp), `WATCHDOG_MS`, `RELEASE_WHEN_IDLE`, `USE_SWITCHES_DEFAULT`, `FIELD_UNITS_PER_VOLT`.

Python (top of `emf_scan.py`): `PORT`, `BAUD`, `DEFAULT_STEP_MM`, `SMOOTH_CURVE`, `MANUAL_FIELD`, `SETTLE_S`, `OUT_DIR`.

## Current status and limitations

- **Readings are typed in.** The gaussmeter's analog output (0 to ±2 V) is not wired to the Arduino yet, so the operator reads the meter and types each value (`MANUAL_FIELD = True`). The firmware already contains the analog readout: a differential reading of A0 (signal) minus A1 (a 2.5 V bias node), scaled by `FIELD_UNITS_PER_VOLT` (10 mT per volt on the ±20 mT range).
- **Open-loop positioning.** A stepper has no position feedback; the limit switches correct drift at the ends of the rail. An encoder would detect missed steps mid-rail.
- **Validation.** The control logic was tested against a software model of the rail (the real firmware compiled on a PC with an Arduino shim, driven by Python test scripts). Those scripts are not part of this repository yet. Real-hardware characterization (positional accuracy and repeatability) is still to do.

## Roadmap

- Wire and calibrate the analog gaussmeter readout, with averaging and error bars.
- Add an encoder or more reference switches to detect missed steps.
- Move stepping to a timer interrupt so analog reads never delay a step.
- Second axis for 2D field maps.

## Troubleshooting

| Problem | Fix |
|---|---|
| "No scanner found" | Is the firmware uploaded? Is the cable a data cable? Is the PlatformIO monitor closed? Try `python emf_scan.py COM14`. |
| "Access is denied" on the port | Another program has it open (PlatformIO monitor, Arduino IDE). Close it. |
| Keys do nothing | Click the graph window first. |
| Probe will not go back | The START switch reads pressed. Check the status line and the switch wiring (pressed must pull the pin to GND). |
| Probe drifts when idle | The motor is off at rest; set `RELEASE_WHEN_IDLE` to `false` and re-upload. |
| Readings look wrong | Check the gaussmeter range, DC-B mode and zero offset. |

## Project structure

```
src/main.cpp          Arduino firmware
emf_scan.py           Python control, plotting and logging app
platformio.ini        PlatformIO build settings (Arduino Nano, ATmega328)
requirements.txt      pyserial, matplotlib, numpy
run_scan.bat          One-click launcher for Windows
scans/                Saved sessions (created at run time)
```

## Credits

Developed by Nick Prevaj (Villanova University, Computer Engineering) for the VESL / WindAid project in Trujillo, Peru. 
