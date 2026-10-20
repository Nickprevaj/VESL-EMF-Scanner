"""
EMF scanner: keyboard-driven live scan tool. Works with the matching main.cpp.

Run:     python emf_scan.py              (finds the scanner by itself, any COM port / any computer)
         python emf_scan.py COM14        (force a specific port, e.g. COM14 or /dev/ttyUSB0)
         python emf_scan.py --list       (show the serial ports this computer has)

A live graph window opens. CLICK THE GRAPH WINDOW, then use these keys:

  Motion                              Recording
    D   move forward until stopped      R      record a reading at this position
    A   move backward until stopped     N      move one step, then record
    SPACE  stop (also stops a test)     T      test run: go to zero, then record every step
    W / S  faster / slower                     (step set with I) until 3 cm before the edge
    C   calibrate (see below)           I      set the step in mm (type 10 -> 10, 20, 30 ...)
                                        M      record a magnet EDGE point (red diamond)
    V   go to zero                      U      undo the last recorded point
    Z   set position 0 here (e.g. at the magnet)
    . / ,  nudge +1 mm / -1 mm          L      toggle smooth curve / straight lines
                                        ENTER  save the data + graph
                                        X      discard everything and close

Calibration (C): the carriage moves BACK until it presses the START limit switch
(D12), which becomes ZERO, then FORWARD until it presses the END switch (D11), which
is the far EDGE. Without switches, or to override them, press SPACE to set each
end by hand. After that it will not move past zero or the edge. Press C again to
cancel. The limit switches also stop the carriage at any time, even before
calibrating, and a switch hit re-syncs the position if the motor ever lost steps.

Readings: while MANUAL_FIELD is True (below) recording asks you to TYPE the
number shown on the gaussmeter, right in the graph window (digits, '.', '-',
BACKSPACE, then ENTER; ESC cancels). Set MANUAL_FIELD = False once the meter's
analog output is wired to the Arduino and the readings will be automatic.

Saved sessions go into  scans/<date_time>/  next to this script. Nothing is
written until you press ENTER (or close the window with points recorded).
The motor stops by itself if this program stops talking to the Arduino.
Close PlatformIO's serial monitor first (only one program can use the port).
"""
import sys, time, csv, threading, collections, statistics
from pathlib import Path
import numpy as np
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.figure import Figure

import serial.tools.list_ports as list_ports

PORT = "auto"             # "auto" = search for the scanner; main() reads the command line
BAUD = 115200
DEFAULT_STEP_MM = 1.0     # distance between points for N and T (press I to change it)
# USB chips Nano boards use: CH340 (1A86), FTDI (0403), Arduino (2341/2A03), CP210x (10C4)
KNOWN_VIDS = {0x1A86, 0x0403, 0x2341, 0x2A03, 0x10C4}
HANDSHAKE_S = 3.5            # how long to listen on each port (the Nano reboots when a port opens)

SMOOTH_CURVE = True    # draw a smooth curve through the recorded points (press L to toggle)
MANUAL_FIELD = True    # True: type each reading from the meter's display. False: read the analog output
FIELD_LABEL = "Magnetic flux density (mT)"
STEPS_PER_MM = 53.333
TEST_EDGE_MARGIN_MM = 30.0   # the T test stops this far before the far edge
NUDGE_MM = 53 / STEPS_PER_MM  # one . or , nudge in the firmware (53 steps)
SETTLE_S = 0.5         # wait after stopping so vibration dies down
WINDOW_S = 0.6         # automatic mode: average readings over this long at each point
HEARTBEAT_S = 0.25     # keep-alive for the firmware's watchdog
DEBOUNCE_S = 0.35      # ignore a held-down SPACE / C / V / ENTER repeating
OUT_DIR = Path(__file__).resolve().parent / "scans"

MOTION_KEYS = {"d", "a", "w", "s", " ", "c", "v", ".", ","}

# zsw / fsw: START / END limit switch (1 pressed, 0 open, -1 = switches not in use)
Sample = collections.namedtuple("Sample", "t pos volts field mode dir rate zsw fsw", defaults=(-1, -1))


def smooth_curve(points, n=400):
    """Smooth curve through (x, y) points: monotone cubic (PCHIP), so it never
    overshoots between points the way an ordinary spline can. Returns (xs, ys)."""
    by = {}
    for x, y in points:
        by.setdefault(round(x, 2), []).append(y)       # average repeats at the same position
    xs = np.array(sorted(by), dtype=float)
    ys = np.array([sum(by[x]) / len(by[x]) for x in sorted(by)], dtype=float)
    if len(xs) < 3:
        return xs, ys
    h = np.diff(xs)
    d = np.diff(ys) / h
    m = np.zeros_like(ys)
    for i in range(1, len(xs) - 1):
        if d[i - 1] * d[i] > 0:
            w1, w2 = 2 * h[i] + h[i - 1], h[i] + 2 * h[i - 1]
            m[i] = (w1 + w2) / (w1 / d[i - 1] + w2 / d[i])

    def end_slope(h0, h1, d0, d1):
        e = ((2 * h0 + h1) * d0 - h0 * d1) / (h0 + h1)
        if np.sign(e) != np.sign(d0):
            return 0.0
        if np.sign(d0) != np.sign(d1) and abs(e) > 3 * abs(d0):
            return 3 * d0
        return e

    m[0] = end_slope(h[0], h[1], d[0], d[1])
    m[-1] = end_slope(h[-1], h[-2], d[-1], d[-2])
    xx = np.linspace(xs[0], xs[-1], n)
    i = np.clip(np.searchsorted(xs, xx) - 1, 0, len(xs) - 2)
    t = (xx - xs[i]) / h[i]
    yy = ((2 * t**3 - 3 * t**2 + 1) * ys[i] + (t**3 - 2 * t**2 + t) * h[i] * m[i]
          + (-2 * t**3 + 3 * t**2) * ys[i + 1] + (t**3 - t**2) * h[i] * m[i + 1])
    return xx, yy


def candidate_ports():
    """Serial ports that could be the scanner, most likely first. Bluetooth ports are skipped."""
    ports = [p for p in list_ports.comports() if "bluetooth" not in (p.description or "").lower()]
    def rank(p):
        text = f"{p.description} {p.manufacturer}".lower()
        likely = p.vid in KNOWN_VIDS or any(w in text for w in ("ch340", "arduino", "usb-serial", "usb serial", "ftdi"))
        return (0 if likely else 1, p.device)
    return sorted(ports, key=rank)


def looks_like_scanner(ser):
    """Listen for our firmware: its '#READY' line, or a 7-field data line. Anything else is not ours.
    Never waits longer than HANDSHAKE_S, whatever the port does."""
    deadline = time.time() + HANDSHAKE_S
    buf = b""
    while time.time() < deadline:
        try:
            chunk = ser.read(ser.in_waiting or 1)
        except Exception:
            return False
        if not chunk:
            continue
        buf += chunk[-2000:]
        for raw in buf.split(b"\n")[:-1]:
            line = raw.decode(errors="ignore").strip()
            if line.startswith("#READY") or (line.count(",") in (6, 8) and line[:1].isdigit()):
                return True
        buf = buf[-300:]
    return False


def open_scanner(name):
    """Open the scanner. name='auto' tries every candidate port and keeps the one that answers."""
    if name != "auto":
        return serial.Serial(name, BAUD, timeout=0.1)
    busy = []
    cands = candidate_ports()
    for p in cands:
        print(f"  looking for the scanner on {p.device} ({p.description})...", flush=True)
        try:
            ser = serial.Serial(p.device, BAUD, timeout=0.1, write_timeout=1)
        except serial.SerialException:
            busy.append(p.device)
            continue
        if looks_like_scanner(ser):
            print(f"  found it: {p.device}", flush=True)
            return ser
        print(f"  {p.device}: no answer from the scanner firmware", flush=True)
        try:
            ser.close()
        except Exception:
            pass
    msg = "No scanner found. " + ("No serial ports at all - is the USB cable plugged in (a data cable, not charge-only)?"
                                  if not cands else f"Checked: {', '.join(p.device for p in cands)}.")
    if busy:
        msg += f"  Could not open {', '.join(busy)} - another program (PlatformIO monitor?) may be using it."
    if cands and len(busy) < len(cands):
        msg += "  Is the new firmware uploaded? (Or run  python emf_scan.py COM14  to skip the search.)"
    raise serial.SerialException(msg)


def curve_xy(records, smooth, origin=0.0):
    """Curve through the recorded points, in graph coordinates (rail position minus the Z zero point)."""
    pts = sorted((r[0] - origin, r[1]) for r in records)
    if smooth and len(pts) >= 3:
        return smooth_curve(pts)
    return [p[0] for p in pts], [p[1] for p in pts]


def draw_edge(ax, x, y, first=False):
    """A magnet edge is a normal data point on the curve, drawn as a big red diamond with its position."""
    (pt,) = ax.plot([x], [y], "D", ms=9, color="tab:red", mec="black", zorder=5,
                    label="magnet edge" if first else None)
    tx = ax.annotate(f"{x:g} mm", (x, y), xytext=(0, 10), textcoords="offset points",
                     ha="center", fontsize=8, color="tab:red")
    return pt, tx

class Link:
    """Background reader + heartbeat for the Arduino."""

    def __init__(self, ser):
        self.ser = ser
        self.lock = threading.Lock()
        self.wlock = threading.Lock()
        self.samples = collections.deque(maxlen=6000)
        self.last_msg = ("", 0.0)
        self.range = None            # (zero_mm, edge_mm) after calibration
        self.range_source = None     # "switch" (found by the limit switches) or "manual" (SPACE)
        self.bytes_received = 0
        self.lines = collections.deque(maxlen=5)   # last few raw lines, for diagnostics
        self._stop = False
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._beat = threading.Thread(target=self._beat_loop, daemon=True)
        self._reader.start()
        self._beat.start()

    def _handle(self, raw):
        if not raw:
            return
        self.lines.append(raw)
        if raw.startswith("#"):
            text = raw[1:].strip()
            if text.startswith("RANGE,"):
                try:
                    f = text.split(",")
                    self.range = (float(f[1]), float(f[2]))
                    self.range_source = f[3].strip() if len(f) > 3 else "manual"
                    how = "found by the limit switches" if self.range_source == "switch" else "set by hand"
                    text = f"Calibrated: zero to {float(f[2]):.1f} mm ({how})"
                except ValueError:
                    return
            if text.startswith("CALIBRATING"):
                self.range = None            # a new calibration invalidates the old range
                self.range_source = None
            print(f"  [scanner] {text}")
            self.last_msg = (text, time.time())
            return
        parts = raw.split(",")
        if len(parts) not in (4, 7, 9):
            return
        try:
            v = [float(x) for x in parts]
        except ValueError:
            return
        if len(v) == 4:
            v += [0, 0, 1000]
        if len(v) == 7:
            v += [-1, -1]
        with self.lock:
            self.samples.append(Sample(time.time(), v[1], v[2], v[3], int(v[4]), int(v[5]), int(v[6]),
                                       int(v[7]), int(v[8])))

    def _read_loop(self):
        buf = b""
        while not self._stop:
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except Exception:
                break
            if not chunk:
                continue
            self.bytes_received += len(chunk)
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self._handle(line.decode(errors="ignore").strip())

    def _beat_loop(self):
        while not self._stop:
            try:
                self.send("p")
            except Exception:
                break
            time.sleep(HEARTBEAT_S)

    def send(self, ch):
        with self.wlock:
            self.ser.write(ch.encode())

    def latest(self):
        with self.lock:
            return self.samples[-1] if self.samples else None

    def read_here(self):
        """Automatic mode: average the live readings; returns pos, volts, field, std."""
        time.sleep(SETTLE_S)
        t0 = time.time()
        time.sleep(WINDOW_S)
        with self.lock:
            pts = [s for s in self.samples if s.t >= t0]
        if len(pts) < 3:
            raise RuntimeError("no data coming from the scanner (is the new firmware uploaded?)")
        if any(p.dir != 0 for p in pts):
            raise RuntimeError("the carriage moved while reading - stop it (SPACE) and try again")
        fields = [p.field for p in pts]
        return (statistics.mean(p.pos for p in pts), statistics.mean(p.volts for p in pts),
                statistics.mean(fields), statistics.pstdev(fields))

    def close(self):
        self._stop = True
        self._reader.join(timeout=1)
        self._beat.join(timeout=1)
        try:
            self.ser.close()
        except Exception:
            pass


def save_session(folder, stamp, records, edges, rng, smooth=True, range_source=None, origin=0.0):
    """Writes scan_data.csv (just position and field), scan_info.txt and scan_graph.png.
    Positions are graph positions: rail position minus the zero point set with Z (0 = rail start)."""
    folder.mkdir(parents=True, exist_ok=True)
    rows = sorted(records)  # (rail position, field, std, volts)
    with open(folder / "scan_data.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["position (mm)", "Magnetic Field (mT)"])
        w.writerows((round(r[0] - origin, 3), r[1]) for r in rows)
    lim = f"{-origin:.3f} to {rng[1] - origin:.3f}" if rng else "not calibrated"
    (folder / "scan_info.txt").write_text(
        f"scan started {stamp}\npoints: {len(rows)}\n"
        f"edges_mm: {' '.join(f'{e[0] - origin:g}' for e in edges) or 'none'}\n"
        f"graph zero: {'the rail start' if not origin else f'set with Z at rail position {origin:.3f} mm'}\n"
        f"rail range_mm: {lim}"
        f"{' (found by the limit switches)' if rng and range_source == 'switch' else ' (set by hand)' if rng else ''}\n"
        f"field units: {FIELD_LABEL}\nreadings: {'typed from the meter' if MANUAL_FIELD else 'analog output'}\n"
    )
    fig = Figure(figsize=(10, 6))  # not attached to pyplot, safe to build anywhere
    ax = fig.subplots()
    cx, cy = curve_xy(records, smooth, origin)
    ax.plot(cx, cy, "-", lw=1.8, color="tab:blue", label="curve through points" if smooth and len(rows) >= 3 else None)
    ax.errorbar([r[0] - origin for r in rows], [r[1] for r in rows], yerr=[r[2] for r in rows],
                fmt="o", ms=5, capsize=2, color="tab:blue", label="recorded points")
    for i, e in enumerate(edges):
        draw_edge(ax, e[0] - origin, e[1], first=(i == 0))
    if rng:
        for i, x in enumerate(rng):
            ax.axvline(x - origin, ls=":", lw=1, color="gray", label="rail start / end" if i == 0 else None)
    if origin:
        ax.axvline(0, ls="-", lw=1, color="tab:green", alpha=0.7, label="zero point")
    ax.set_xlabel("position (mm)")
    ax.set_ylabel(FIELD_LABEL)
    ax.set_title(f"EMF scan  {stamp}")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(folder / "scan_graph.png", dpi=150)


class App:
    def __init__(self, link):
        self.link = link
        self.step_mm = DEFAULT_STEP_MM
        self.origin = 0.0        # rail position (mm) that counts as 0 on the graph (set with Z; 0 = rail start)
        self.fig = None
        self.records = []        # (position, field, std, volts)
        self.edges = []
        self.stamp = None        # one folder per session
        self.folder = None
        self.dirty = False
        self.discard = False
        self.smooth = SMOOTH_CURVE
        self.busy = threading.Lock()
        self.abort = threading.Event()
        self.last_key = {}
        self.notice = ("", 0.0)
        # typed-reading prompt
        self.entry = None        # None = not asking; otherwise the text typed so far
        self.entry_cancel = False
        self.entry_done = threading.Event()
        self.entry_label = ""

    def note(self, text):
        print("  " + text)
        self.notice = (text, time.time())

    # ---- background jobs (so the window never freezes) ----
    def _run(self, fn):
        try:
            fn()
        except RuntimeError as e:
            self.note(str(e))
        finally:
            self.busy.release()

    def start(self, fn):
        if not self.busy.acquire(blocking=False):
            self.note("still working on the last command (SPACE stops it)")
            return
        self.abort.clear()
        threading.Thread(target=self._run, args=(fn,), daemon=True).start()

    def wait_for(self, cond, timeout, what):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.abort.is_set():
                raise RuntimeError("stopped")
            c = self.link.latest()
            if c is not None and cond(c):
                return c
            time.sleep(0.05)
        raise RuntimeError(f"timed out waiting for {what}")

    def ask_field(self, pos):
        return self.ask_number(f"the meter reading in mT at {pos - self.origin:.2f} mm")

    def ask_number(self, label):
        self.entry, self.entry_label, self.entry_cancel = "", label, False
        self.entry_done.clear()
        try:
            while True:
                self.entry_done.wait()
                if self.entry_cancel or self.abort.is_set():
                    raise RuntimeError("entry cancelled")
                try:
                    return float(self.entry)
                except ValueError:
                    self.note("that isn't a number - type it, e.g. 12.5 or 10, then ENTER")
                    self.entry = ""
                    self.entry_done.clear()
        finally:
            self.entry = None

    def record(self, edge=False):
        s = self.link.latest()
        if s is None:
            raise RuntimeError("no data from the scanner yet")
        if s.dir != 0:
            raise RuntimeError("stop first (SPACE), then press R")
        if MANUAL_FIELD:
            time.sleep(SETTLE_S)
            c = self.link.latest()
            if c.dir != 0:
                raise RuntimeError("the carriage moved - stop it (SPACE) and try again")
            pos = c.pos
            field = self.ask_field(pos)
            volts, std = field / 10.0, 0.0
        else:
            pos, volts, field, std = self.link.read_here()
        self.records.append((pos, field, std, volts))
        if edge:
            self.edges.append((pos, field))
        self.dirty = True
        self.note(f"#{len(self.records)} {'EDGE ' if edge else ''}recorded  {pos - self.origin:.2f} mm   {field:.4f} mT" +
                  (f" +/- {std:.4f}" if std else ""))

    def goto_mm(self, mm):
        """Move to an absolute position (mm from zero) and wait until the carriage has stopped there."""
        s = self.link.latest()
        if s is None:
            raise RuntimeError("no data from the scanner yet")
        if s.dir != 0:
            raise RuntimeError("stop first (SPACE)")
        if abs(mm - s.pos) < 0.02:
            return
        t_cmd = time.time()
        self.link.send(f"g{round(mm * STEPS_PER_MM)}\n")
        timeout = abs(mm - s.pos) / 5.0 + 15       # even the slowest speed is about 1.9 mm/s
        c = self.wait_for(lambda c: c.t > t_cmd + 0.3 and c.dir == 0, timeout, f"the move to {mm:g} mm")
        if abs(c.pos - mm) > 0.15:
            raise RuntimeError(f"stopped at {c.pos:.2f} mm instead of {mm:g} mm "
                               "(far edge, calibration limit, or the firmware is the old version - upload the new main.cpp)")

    def advance(self):
        """Move forward by the step size (set with I)."""
        s = self.link.latest()
        if s is None:
            raise RuntimeError("no data from the scanner yet")
        self.goto_mm(s.pos + self.step_mm)

    def step_record(self):
        self.advance()
        self.record()

    def set_zero(self):
        """Z: make the probe's current spot position 0 on the graph (e.g. the middle or edge of the magnet)."""
        s = self.link.latest()
        if s is None:
            self.note("no data from the scanner yet")
            return
        self.origin = round(s.pos, 3)
        where = "the rail start" if not self.origin else f"rail position {self.origin:.2f} mm"
        self.note(f"zero point set here ({where}). Positions on the graph and in the file are now measured from this spot")

    def set_step(self):
        v = self.ask_number("the step size in mm (e.g. 1, 5, 10, 15)")
        if v <= 0 or v > 500:
            raise RuntimeError("the step must be more than 0 and at most 500 mm")
        self.step_mm = v
        self.note(f"step set to {v:g} mm: N and T now move {v:g}, {2 * v:g}, {3 * v:g}... mm")

    def test_run(self):
        rng = self.link.range
        if not rng:
            raise RuntimeError("calibrate first (C)")
        limit = rng[1] - TEST_EDGE_MARGIN_MM
        if limit <= 0:
            raise RuntimeError(f"the range must be longer than {TEST_EDGE_MARGIN_MM:g} mm for a test run")
        s = self.link.latest()
        if s is None or s.dir != 0:
            raise RuntimeError("stop first (SPACE), then press T")
        step = self.step_mm
        self.note("test run: going to zero first (SPACE stops)")
        self.link.send("v")
        t_cmd = time.time()
        self.wait_for(lambda c: c.t > t_cmd + 0.3 and c.dir == 0 and abs(c.pos) < 0.2, 180, "the carriage to reach zero")
        self.note(f"test run: a point every {step:g} mm up to {limit:.1f} mm (SPACE stops)")
        k = 0
        while True:
            self.record()
            k += 1
            if k * step > limit + 0.05:                     # never go past the margin
                break
            self.goto_mm(k * step)                          # absolute targets, so errors never add up
        self.note("test run complete - press ENTER to save")

    def save(self):
        if not self.records:
            self.note("nothing recorded yet (press R to record a point)")
            return
        if self.folder is None:
            self.stamp = time.strftime("%Y-%m-%d_%H-%M-%S")
            self.folder = OUT_DIR / self.stamp
        save_session(self.folder, self.stamp, self.records, self.edges, self.link.range, self.smooth, self.link.range_source, self.origin)
        self.dirty = False
        self.note(f"Saved {len(self.records)} points to {self.folder}")

    # ---- keyboard ----
    def handle_key(self, key):
        if not key:
            return
        k = " " if key in (" ", "space") else key.lower()

        if k == " ":
            self.abort.set()                      # stops tests / entry prompts as well as the motor

        if self.entry is not None:                # typing a meter reading
            if k in "0123456789.-" and len(k) == 1:
                self.entry += k
            elif k == "backspace":
                self.entry = self.entry[:-1]
            elif k == "enter":
                self.entry_done.set()
            elif k in ("escape", " "):
                self.entry_cancel = True
                self.entry_done.set()
                if k == " ":
                    self.link.send(" ")
            return

        if k in (" ", "c", "v", "enter"):
            now = time.time()
            if now - self.last_key.get(k, 0) < DEBOUNCE_S:
                return
            self.last_key[k] = now
        if k == "z":
            self.set_zero()
        elif k == "v" and self.origin:
            self.start(lambda: self.goto_mm(self.origin))     # back to the zero point you set with Z
        elif k in MOTION_KEYS:
            self.link.send(k)
        elif k == "r":
            self.start(self.record)
        elif k == "n":
            self.start(self.step_record)
        elif k == "i":
            self.start(self.set_step)
        elif k == "t":
            self.start(self.test_run)
        elif k == "m":
            self.start(lambda: self.record(edge=True))   # an edge is a recorded point, flagged red
        elif k == "l":
            self.smooth = not self.smooth
            self.note("curve: " + ("smooth" if self.smooth else "straight lines between points"))
        elif k == "u":
            if self.records:
                r = self.records.pop()
                if self.edges and self.edges[-1][0] == r[0]:
                    self.edges.pop()
                self.dirty = True
                self.note(f"removed the point at {r[0] - self.origin:.2f} mm ({len(self.records)} left)")
            else:
                self.note("nothing to undo")
        elif k == "enter":
            self.save()
        elif k == "x":
            self.discard = True
            self.note("discarding this session")
            if self.fig is not None:
                plt.close(self.fig)


def describe(s):
    if s.mode == 2:
        return ("CALIBRATING - going to the START switch" if s.zsw >= 0 and s.dir else
                "CALIBRATING - probe moving BACK" if s.dir else "CALIBRATING - probe stopped")
    if s.mode == 3:
        return ("CALIBRATING - going to the END switch" if s.zsw >= 0 and s.dir else
                "CALIBRATING - probe moving FORWARD" if s.dir else "CALIBRATING - probe stopped")
    if s.mode == 4:
        return "returning to zero"
    if s.dir:
        return "moving " + {1: "forward", -1: "back"}.get(s.dir, "")
    return "idle"


def next_step(app, s):
    """One plain-English sentence telling the user what to do right now."""
    sw = s is not None and s.zsw >= 0                 # firmware has the two edge switches turned on
    if app.entry is not None:
        return "Read the number on the gaussmeter, type it, then press ENTER  (ESC cancels)"
    if s is not None and s.mode == 2:
        if sw:
            return ("CALIBRATION 1/2: the probe is driving back to the START switch. It stops there by itself.\n"
                    "(Nothing to press. SPACE would set the start by hand instead.)") if s.dir else \
                   "CALIBRATION 1/2: stopped. Press SPACE to set this spot as position 0, or A / D to move it first."
        return ("CALIBRATION 1/2: the probe is moving back. Press SPACE when it reaches the START of the rail.\n"
                "That spot becomes position 0.  (A / D nudge it if you stopped too early.)") if s.dir else \
               "CALIBRATION 1/2: stopped. Press SPACE to set this spot as position 0 (the start), or A / D to move it first."
    if s is not None and s.mode == 3:
        if sw:
            return ("CALIBRATION 2/2: START switch found (position 0). Now driving forward to the END switch.\n"
                    "It stops there by itself. (SPACE would set the end by hand instead.)") if s.dir else \
                   "CALIBRATION 2/2: stopped. Press SPACE to set this spot as the END of the rail, or D / A to move it first."
        return ("CALIBRATION 2/2: the probe is moving forward. Press SPACE when it reaches the END of the rail.\n"
                "That spot is the far limit; the probe will never go past it.") if s.dir else \
               "CALIBRATION 2/2: stopped. Press SPACE to set this spot as the END of the rail, or D / A to move it first."
    if app.busy.locked():
        return "Scan in progress - the probe stops at each point and asks for a reading.  SPACE cancels."
    if not app.link.range:
        if sw:
            return ("STEP 1: calibrate the rail. Press C - the probe drives back until it presses the START switch,\n"
                    "then forward until it presses the END switch. That gives it the exact length of the rail.")
        return ("STEP 1: calibrate the rail. Press C - the probe drives backward.\n"
                "When it reaches the start of the rail, press SPACE.")
    if not app.records:
        return (f"STEP 2: record the field. Press T to scan the whole rail automatically, one point every "
                f"{app.step_mm:g} mm\n(press I first to change that distance), or move with D / A and press R "
                "to record a single point.")
    if app.dirty:
        return (f"{len(app.records)} points so far. Keep going with R / N / T, mark a magnet edge with M,\n"
                "or press ENTER to save the graph and data.")
    return f"Saved to {app.folder}.  You can keep recording, or close the window."


GUIDE = """MOVE THE PROBE
  D   forward, until you stop it
  A   backward, until you stop it
  SPACE  stop
  W / S  faster / slower
  . / ,  nudge 1 mm forward / back
  V   go back to zero (the rail start,
      or your Z point if you set one)

SET UP
  C   calibrate: finds the START and
      END switches by itself
  I   how far apart the points are
      (type 10 -> 10, 20, 30 ... mm)
  Z   set position 0 right here (say,
      the middle or edge of the magnet);
      the graph and file use it

RECORD POINTS
  T   scan the whole rail automatically
  R   record one point right here
  N   move one step, then record
  M   record a magnet EDGE point
      (shown as a red diamond)
  U   undo the last point

GRAPH AND FILES
  L   smooth curve / straight lines
  ENTER  save graph + data
  X   discard everything and close"""


def build_live(app):
    link = app.link
    typed = MANUAL_FIELD                            # readings typed by hand
    fig = plt.figure(figsize=(12.5, 7.2))
    try:
        fig.canvas.manager.set_window_title("EMF scanner")
    except Exception:
        pass
    ax = fig.add_axes([0.065, 0.17, 0.65, 0.63])
    (curve_line,) = ax.plot([], [], "-", lw=1.8, color="tab:blue", label="curve through points")
    (pts_line,) = ax.plot([], [], "o", ms=6, color="tab:blue", label="recorded points")
    probe_line = ax.axvline(0, color="tab:orange", ls="-", lw=1.5, label="probe position now")
    (probe_dot,) = ax.plot([], [], "o", mfc="none", mec="tab:orange", ms=10, mew=2)
    ax.set_xlabel("position along the rail (mm)")
    ax.set_ylabel(FIELD_LABEL)
    ax.set_title("Magnetic field along the rail", fontsize=12)
    ax.grid(True, alpha=0.3)
    empty = ax.text(0.5, 0.5, "No points yet.\nRecorded points will appear here\nand join up into a curve.",
                    transform=ax.transAxes, ha="center", va="center", fontsize=13, color="gray")
    ax.legend(loc="upper right", fontsize=8)

    guide_txt = fig.text(0.015, 0.955, "", ha="left", va="top", fontsize=11.5, fontweight="bold", color="#1f3b73")
    status = fig.text(0.065, 0.865, "connecting...", ha="left", va="bottom", fontsize=9.5, color="dimgray")
    notice = fig.text(0.065, 0.045, "", ha="left", va="center", fontsize=10, color="tab:blue")
    fig.text(0.74, 0.76, "KEYS  (click this window first)", ha="left", va="bottom", fontsize=10, fontweight="bold")
    fig.text(0.74, 0.745, GUIDE, ha="left", va="top", fontsize=8.6, family="monospace", linespacing=1.35)
    edge_artists, range_artists = [], []
    shown = {"range": None, "origin": 0.0}
    zero_line = ax.axvline(0, ls="-", lw=1.2, color="tab:green", alpha=0.7, label="zero point")
    zero_line.set_visible(False)
    zero_line.set_label("_nolegend_")

    def update(_):
        s = link.latest()
        guide_txt.set_text(next_step(app, s))
        if s:
            o = app.origin
            probe_line.set_xdata([s.pos - o, s.pos - o])
            if typed:
                probe_dot.set_data([], [])
            else:
                probe_dot.set_data([s.pos - o], [s.field])
            rate = max(s.rate, 1)
            parts = [f"Probe at {s.pos - o:.2f} mm", describe(s), f"{rate / STEPS_PER_MM:.1f} mm/s",
                     f"step {app.step_mm:g} mm", f"{len(app.records)} points"]
            if link.range:
                parts.insert(1, f"rail {-o:.1f} to {link.range[1] - o:.1f} mm")
            if o:
                parts.insert(1, f"zero set at rail {o:.1f} mm")
            if s.zsw >= 0:
                parts.append("switches " + ("START PRESSED" if s.zsw else "END PRESSED" if s.fsw else "open"))
            if not typed:
                parts.append(f"reading {s.field:.3f}")
            status.set_text("   |   ".join(parts))
        if app.entry is not None:
            notice.set_color("tab:orange")
            notice.set_text(f"TYPE {app.entry_label}, then ENTER (ESC cancels):  {app.entry}_")
        else:
            notice.set_color("tab:blue")
            msgs = [m for m in (link.last_msg, app.notice) if m[0]]
            if msgs:
                text, t = max(msgs, key=lambda m: m[1])
                notice.set_text(text if time.time() - t < 12 else "")
        pts = sorted(app.records)
        o = app.origin
        pts_line.set_data([p[0] - o for p in pts], [p[1] for p in pts])
        curve_line.set_data(*curve_xy(app.records, app.smooth, o))
        empty.set_visible(not app.records)
        if o != shown["origin"]:                         # Z moved the zero point: redraw everything positioned
            for pair in edge_artists:
                for a in pair:
                    a.remove()
            edge_artists.clear()
            shown["range"] = None
            shown["origin"] = o
            zero_line.set_visible(bool(o))
            zero_line.set_label("zero point" if o else "_nolegend_")
            zero_line.set_xdata([0, 0])
        while len(edge_artists) < len(app.edges):
            e = app.edges[len(edge_artists)]
            edge_artists.append(draw_edge(ax, e[0] - o, e[1], first=(len(edge_artists) == 0)))
        while len(edge_artists) > len(app.edges):        # after undo
            for a in edge_artists.pop():
                a.remove()
        if link.range != shown["range"]:
            for ln in range_artists:
                ln.remove()
            range_artists.clear()
            if link.range:
                for k, x in enumerate(link.range):
                    range_artists.append(ax.axvline(x - o, ls=":", lw=1.2, color="gray",
                                                    label=("rail start / end (limit switches)" if link.range_source == "switch"
                                                           else "rail start / end") if k == 0 else None))
            shown["range"] = link.range
        ax.relim()
        ax.autoscale_view()
        h, l = ax.get_legend_handles_labels()
        ax.legend(h, l, loc="upper right", fontsize=8)

    return fig, update


def parse_args(argv):
    import argparse
    ap = argparse.ArgumentParser(description="EMF scanner: live graph + keyboard control", add_help=True)
    ap.add_argument("port", nargs="?", default="auto", help="scanner port (default: find it automatically)")
    ap.add_argument("--list", action="store_true", help="show the serial ports and exit")
    ap.add_argument("--step", type=float, default=DEFAULT_STEP_MM, help="starting step in mm (change later with I)")
    return ap.parse_args(argv)


def main():
    global PORT, DEFAULT_STEP_MM
    args = parse_args(sys.argv[1:])
    if args.list:
        for p in list_ports.comports():
            print(f"{p.device:12} {p.description}")
        return
    PORT = args.port
    DEFAULT_STEP_MM = args.step
    try:
        ser = open_scanner(PORT)
    except serial.SerialException as e:
        print(f"Could not connect: {e}")
        print("Close PlatformIO's serial monitor and any other program using the port, then try again.")
        print("(Run  python emf_scan.py --list  to see the ports; force one with  python emf_scan.py COM7)")
        sys.exit(1)
    if PORT != "auto":
        time.sleep(2)  # the Nano resets when the port opens
    ser.reset_input_buffer()
    port_name = ser.port
    link = Link(ser)

    for _ in range(50):
        if link.latest():
            break
        time.sleep(0.1)
    else:
        print(f"No live data from the scanner on {port_name}.")
        print(f"  bytes received: {link.bytes_received}; last lines: {list(link.lines)}")
        print("  If you see text but no numbers, the OLD firmware is still on the Arduino.")
        print("  Upload the new src/main.cpp (close this program first - it holds the port), then run again.")
        link.close()
        sys.exit(1)

    for k in list(plt.rcParams.keys()):          # free D / A / S / C / V / R ... from matplotlib's shortcuts
        if k.startswith("keymap."):
            plt.rcParams[k] = []

    app = App(link)
    fig, update = build_live(app)
    app.fig = fig
    fig.canvas.mpl_connect("key_press_event", lambda e: app.handle_key(e.key))
    print(__doc__)
    print(f"Connected to {port_name}. Click the graph window, then press C to calibrate.\n")
    ani = animation.FuncAnimation(fig, update, interval=200, cache_frame_data=False)  # noqa: F841
    plt.show()  # blocks until the graph window is closed

    link.send(" ")  # make sure the motor is stopped
    if app.records and app.dirty and not app.discard:
        app.save()
        print(f"Window closed - your points were saved to {app.folder}")
    link.close()


if __name__ == "__main__":
    main()