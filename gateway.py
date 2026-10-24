"""
gateway.py — TankWatch serial gateway: main entry point.

Reads D/E lines from the Arduino Uno (or simulator / CSV replay),
applies a rolling median filter, runs three anomaly detectors,
and forwards readings + events to the cloud API.

Usage
-----
  # Real hardware (auto-detect port)
  python gateway.py

  # Real hardware, explicit port
  python gateway.py --port COM3

  # Simulation (no hardware needed)
  python gateway.py --sim leak --speed 10
  python gateway.py --sim normal
  python gateway.py --sim fill_drift --speed 5
  python gateway.py --sim pump_flat_supply --speed 20
  python gateway.py --sim sensor_dead --speed 20

  # Replay a recorded CSV log
  python gateway.py --replay raw_log.csv --speed 5

Dependencies: pyserial, requests (+ stdlib only)
"""

import argparse
import collections
import csv
import json
import logging
import os
import statistics
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Generator, Optional

import serial
import serial.tools.list_ports

from detectors import DrawdownDetector, FillTimeDriftDetector, PumpFlatDetector
from cloud_client import CloudClient
from simulator import scenario_generator

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------

def _setup_logging(verbose: bool = False) -> None:
    level = logging.DEBUG if verbose else logging.INFO
    fmt   = "%(asctime)s [%(levelname)-8s] %(name)s: %(message)s"
    handlers = [
        logging.StreamHandler(sys.stdout),
        logging.FileHandler("gateway.log", encoding="utf-8"),
    ]
    logging.basicConfig(level=level, format=fmt, handlers=handlers)


log = logging.getLogger("gateway")


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

DEFAULT_CONFIG = "config.json"


def load_config(path: str) -> dict:
    with open(path, encoding="utf-8") as fh:
        cfg = json.load(fh)
    log.info(f"Config loaded from {path}")
    return cfg


# ---------------------------------------------------------------------------
# Serial port auto-detection
# ---------------------------------------------------------------------------

def find_arduino_port() -> Optional[str]:
    """
    Heuristic search for the Arduino/CH340 USB serial port.
    Falls back to the first available port if none match by name.
    """
    keywords = ("arduino", "ch340", "ch341", "usb serial", "ftdi")
    for p in serial.tools.list_ports.comports():
        desc  = (p.description  or "").lower()
        manuf = (p.manufacturer or "").lower()
        if any(k in desc or k in manuf for k in keywords):
            log.info(f"Auto-detected port: {p.device} ({p.description})")
            return p.device

    ports = list(serial.tools.list_ports.comports())
    if ports:
        log.warning(
            f"No Arduino found by name; defaulting to {ports[0].device}. "
            f"Use --port to override."
        )
        return ports[0].device

    return None


# ---------------------------------------------------------------------------
# Serial reader (robust, auto-reconnect)
# ---------------------------------------------------------------------------

def serial_lines(
    port: str,
    baud: int = 115200,
) -> Generator[str, None, None]:
    """
    Yield text lines from the serial port indefinitely.

    Robustness features:
      • Byte-level buffering handles partial lines across read() calls.
      • Non-ASCII bytes are replaced with '?' (garbage from boot messages).
      • SerialException / OSError trigger automatic reconnect with 2 s delay.
      • The generator never raises — it either yields or sleeps and retries.
    """
    ser: Optional[serial.Serial] = None
    buf = b""

    while True:
        try:
            if ser is None or not ser.is_open:
                log.info(f"Connecting to {port} @ {baud} baud…")
                ser = serial.Serial(port, baud, timeout=2.0)
                log.info("Serial connection established.")
                buf = b""  # discard any partial line from before reconnect

            chunk = ser.read(256)   # blocks up to timeout=2s
            if not chunk:
                continue
            buf += chunk

            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                text = raw.decode("ascii", errors="replace").strip()
                if text:
                    yield text

        except serial.SerialException as exc:
            log.error(f"Serial error: {exc} — reconnecting in 2 s")
            _close_serial(ser)
            ser = None
            buf = b""
            time.sleep(2)

        except OSError as exc:
            log.error(f"OS error on serial port: {exc} — reconnecting in 2 s")
            _close_serial(ser)
            ser = None
            buf = b""
            time.sleep(2)


def _close_serial(ser: Optional[serial.Serial]) -> None:
    if ser is not None:
        try:
            ser.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# CSV logger (raw line archive for later replay)
# ---------------------------------------------------------------------------

class CsvLogger:
    """
    Appends every raw serial line + a UTC wall-clock timestamp to a CSV file.
    Buffered with line=1 so data is flushed frequently without being slow.
    """

    HEADER = ["wall_time_utc", "raw_line"]

    def __init__(self, path: str):
        exists = os.path.exists(path)
        self._f = open(path, "a", newline="", encoding="utf-8", buffering=1)
        self._w = csv.writer(self._f)
        if not exists:
            self._w.writerow(self.HEADER)

    def log(self, line: str) -> None:
        self._w.writerow([datetime.utcnow().isoformat() + "Z", line])

    def close(self) -> None:
        self._f.close()


# ---------------------------------------------------------------------------
# Line parser
# ---------------------------------------------------------------------------

class ParsedReading:
    __slots__ = ("uptime_ms", "dist_mm", "pump", "state")

    def __init__(self, uptime_ms: int, dist_mm: int, pump: bool, state: str):
        self.uptime_ms = uptime_ms
        self.dist_mm   = dist_mm
        self.pump      = pump
        self.state     = state


class ParsedEvent:
    __slots__ = ("uptime_ms", "code")

    def __init__(self, uptime_ms: int, code: str):
        self.uptime_ms = uptime_ms
        self.code      = code


def parse_line(line: str) -> Optional[object]:
    """
    Parse one serial line.  Returns ParsedReading, ParsedEvent, or None.

    D,<uptime_ms>,<dist_mm>,<pump 0|1>,<state>
    E,<uptime_ms>,<code>
    """
    if not line:
        return None
    parts = line.split(",")
    try:
        kind = parts[0]
        if kind == "D" and len(parts) >= 5:
            return ParsedReading(
                uptime_ms=int(parts[1]),
                dist_mm=int(parts[2]),
                pump=bool(int(parts[3])),
                state=parts[4].strip(),
            )
        if kind == "E" and len(parts) >= 3:
            return ParsedEvent(
                uptime_ms=int(parts[1]),
                code=parts[2].strip(),
            )
    except (ValueError, IndexError):
        pass
    return None


# ---------------------------------------------------------------------------
# Pre-processing: level conversion
# ---------------------------------------------------------------------------

def dist_to_level(dist_mm: float, cfg: dict) -> tuple[float, float]:
    """
    Convert a filtered sensor distance to level percentage and volume.

    level_pct = clamp(100 * (tank_height_mm - (dist_mm - sensor_offset_mm)) / tank_height_mm, 0, 100)
    Litres = level fraction * tank volume.

    Tank volume derivation:
      Area (cm²) * height (cm) = volume in cm³ (mL).
      With 1,000 cm³ = 1 Litre:
        tank_volume_l = tank_area_cm2 * (tank_height_mm / 10.0) / 1000.0
                      = (tank_area_cm2 * tank_height_mm) / 10,000.0
      Note on prompt specification:
        "tank_area_cm2 * tank_height_mm / 1000 / 1000" refers to converting mm³
        to Litres (1,000,000 mm³ per Litre), where area in mm² = tank_area_cm2 * 100.
        Thus (tank_area_cm2 * 100 * tank_height_mm) / 1,000,000 = tank_area_cm2 * tank_height_mm / 10,000.
    """
    water_depth_mm = cfg["tank_height_mm"] - (dist_mm - cfg["sensor_offset_mm"])
    level_pct = max(0.0, min(100.0, 100.0 * water_depth_mm / cfg["tank_height_mm"]))
    level_fraction = level_pct / 100.0
    tank_volume_l = (cfg["tank_area_cm2"] * cfg["tank_height_mm"]) / 10_000.0
    level_l = max(0.0, min(tank_volume_l, level_fraction * tank_volume_l))
    return level_pct, level_l


# ---------------------------------------------------------------------------
# Rolling median filter
# ---------------------------------------------------------------------------

class MedianFilter:
    """
    Rolling median over the last `window` *valid* distance readings.

    A reading is valid if:
      • dist_mm != -1   (not a sensor timeout)
      • sensor_offset_mm <= dist_mm <= sensor_offset_mm + tank_height_mm + 50
        (a 50 mm margin below the empty mark for sensor overshoot)

    Invalid readings are counted; the streak and zero-variance flag are
    passed to the PumpFlatDetector.
    """

    def __init__(self, window: int = 5):
        self._window = window
        self._buf    = collections.deque(maxlen=window)
        self._streak = 0   # consecutive invalid readings

    def update(self, dist_mm: int, cfg: dict) -> Optional[int]:
        """
        Push a reading.  Returns the filtered dist_mm, or None if rejected.
        """
        lo = cfg["sensor_offset_mm"]
        hi = cfg["sensor_offset_mm"] + cfg["tank_height_mm"] + 50

        if dist_mm == -1 or not (lo <= dist_mm <= hi):
            self._streak += 1
            return None

        self._streak = 0
        self._buf.append(dist_mm)

        if len(self._buf) < self._window:
            return dist_mm   # not enough samples yet; pass through raw

        return int(statistics.median(self._buf))

    @property
    def invalid_streak(self) -> int:
        """Consecutive invalid readings since the last good one."""
        return self._streak

    def zero_variance(self) -> bool:
        """
        True if every value in the current buffer is identical.
        Indicates a frozen / stuck sensor output.
        """
        buf = list(self._buf)
        return len(buf) >= 3 and len(set(buf)) == 1


# ---------------------------------------------------------------------------
# Replay reader
# ---------------------------------------------------------------------------

def replay_lines(
    csv_path: str,
    speed: float = 1.0,
) -> Generator[str, None, None]:
    """
    Replay a raw_log.csv at `speed`x compression.

    The inter-line timing is preserved from the wall_time_utc column.
    Delays > 5 s are capped to prevent long stalls on quiet periods.
    """
    rows: list[tuple[str, str]] = []
    with open(csv_path, encoding="utf-8") as fh:
        reader = csv.reader(fh)
        next(reader, None)   # skip header
        for row in reader:
            if len(row) >= 2:
                rows.append((row[0], row[1]))

    if not rows:
        log.error(f"Replay file {csv_path!r} is empty or has no data rows.")
        return

    log.info(f"Replaying {len(rows)} lines from {csv_path!r} at {speed}x speed")
    prev_ts: Optional[datetime] = None

    for wall_time_str, raw_line in rows:
        try:
            ts = datetime.fromisoformat(wall_time_str.rstrip("Z"))
        except ValueError:
            yield raw_line
            continue

        if prev_ts is not None:
            delay_s = (ts - prev_ts).total_seconds() / speed
            if 0.0 < delay_s < 5.0:
                time.sleep(delay_s)
        prev_ts = ts
        yield raw_line


# ---------------------------------------------------------------------------
# Core gateway loop
# ---------------------------------------------------------------------------

def run_gateway(
    cfg: dict,
    line_source: Generator[str, None, None],
    cloud: CloudClient,
    force_quiet: bool = False,
) -> None:
    """
    Main processing loop.  Pulls lines from `line_source`, filters them,
    runs detectors, and forwards results to the cloud client.

    This function blocks until KeyboardInterrupt or the source is exhausted.
    """
    # Detectors
    drawdown   = DrawdownDetector(cfg)
    fill_drift = FillTimeDriftDetector(cfg)
    pump_flat  = PumpFlatDetector(cfg)

    # Pre-seed the EWMA bins to near-full for demo mode.
    # In a real deployment the bins learn over days; here we jump-start them.
    full_dist = cfg["sensor_offset_mm"] + int(cfg["tank_height_mm"] * 0.1)
    drawdown.seed_hour_ewma(full_dist)

    # Median filter
    median_filter = MedianFilter(window=cfg.get("median_window", 5))

    # CSV archive
    csv_log = CsvLogger(cfg.get("log_csv", "raw_log.csv"))

    # Shared quiet_mode flag (initialized from force_quiet or cloud)
    quiet_mode = force_quiet
    _stop_poll = threading.Event()

    def _poll_quiet_mode() -> None:
        nonlocal quiet_mode
        interval = cfg.get("poll_interval_s", 2)
        while not _stop_poll.is_set():
            ctrl = cloud.get_control()
            if ctrl and "quiet_mode" in ctrl:
                new_val = bool(ctrl["quiet_mode"])
                if new_val != quiet_mode:
                    log.info(f"quiet_mode changed → {new_val}")
                quiet_mode = new_val
            _stop_poll.wait(timeout=interval)

    poll_thread = threading.Thread(
        target=_poll_quiet_mode,
        daemon=True,
        name="ctrl-poll",
    )
    poll_thread.start()

    log.info("─" * 60)
    log.info("TankWatch gateway running.  Press Ctrl+C to stop.")
    log.info("─" * 60)

    start_uptime_ms: Optional[int] = None
    start_wall_t: float = time.time()

    try:
        for raw_line in line_source:
            # Archive every line for replay
            csv_log.log(raw_line)

            parsed = parse_line(raw_line)
            if parsed is None:
                log.debug(f"Ignored unparseable: {raw_line!r}")
                continue

            t_now = time.time()
            if hasattr(parsed, "uptime_ms") and parsed.uptime_ms is not None:
                if start_uptime_ms is None:
                    start_uptime_ms = parsed.uptime_ms
                    start_wall_t = t_now
                sim_time = start_wall_t + (parsed.uptime_ms - start_uptime_ms) / 1000.0
            else:
                sim_time = t_now

            now_ts = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")

            # ── Event line (E,...) ──────────────────────────────────────
            if isinstance(parsed, ParsedEvent):
                code = parsed.code
                log.info(f"Uno event: {code}")

                # Fill-drift: needs PUMP_ON / PUMP_OFF_HIGH events
                for evt in fill_drift.on_event(now_ts, code, now=sim_time):
                    cloud.post_event(evt)

                # Forward max-runtime failsafe as a cloud event
                if code == "TRIP_MAXRUN":
                    cloud.post_event({
                        "ts":       now_ts,
                        "type":     "FAILSAFE_TRIP",
                        "severity": "critical",
                        "message": (
                            "Pump max-runtime failsafe triggered on the Uno controller. "
                            f"The pump ran for longer than {cfg.get('max_runtime_s', '?')}s "
                            "without the tank reaching HIGH level. "
                            "Possible causes: supply failure, stuck float switch, "
                            "or a serious leak."
                        ),
                        "data": {"code": code, "max_runtime_s": cfg.get("max_runtime_s")},
                    })
                continue

            # ── Data line (D,...) ──────────────────────────────────────
            assert isinstance(parsed, ParsedReading)
            raw_dist = parsed.dist_mm
            pump     = parsed.pump
            state    = parsed.state

            # Feed fill-drift with raw state before any filtering
            fill_drift.update_state(parsed.uptime_ms, raw_dist, pump, state)

            # Median filter
            filtered = median_filter.update(raw_dist, cfg)
            streak   = median_filter.invalid_streak
            frozen   = median_filter.zero_variance()

            if filtered is None:
                # Invalid reading — still let pump-flat check the sensor
                for evt in pump_flat.update(
                    now_ts, None, pump, streak, frozen, now=sim_time
                ):
                    cloud.post_event(evt)
                log.debug(
                    f"Rejected reading: dist={raw_dist}mm  "
                    f"streak={streak}  frozen={frozen}"
                )
                continue

            level_pct, level_l = dist_to_level(filtered, cfg)

            # Push telemetry to cloud
            cloud.post_reading({
                "ts":        now_ts,
                "dist_mm":   filtered,
                "level_pct": round(level_pct, 1),
                "level_l":   round(level_l, 2),
                "pump":      int(pump),
                "state":     state,
            })

            log.info(
                f"dist={filtered:3d}mm (raw={raw_dist:4d})  "
                f"level={level_pct:5.1f}%  {level_l:.2f}L  "
                f"pump={'ON ' if pump else 'OFF'}  state={state:<8s}  "
                f"quiet={quiet_mode}"
            )

            # ── Drawdown detector ──────────────────────────────────────
            for evt in drawdown.update(
                now_ts, float(filtered), level_pct, level_l, pump, state, quiet_mode, now=sim_time
            ):
                cloud.post_event(evt)

            # ── Fill-drift detector ────────────────────────────────────
            for evt in fill_drift.update(
                now_ts, float(filtered), level_pct, pump, state
            ):
                cloud.post_event(evt)

            # ── Pump-flat detector ─────────────────────────────────────
            for evt in pump_flat.update(
                now_ts, float(filtered), pump, streak, frozen, now=sim_time
            ):
                cloud.post_event(evt)

    except KeyboardInterrupt:
        log.info("KeyboardInterrupt — stopping gateway.")
    finally:
        _stop_poll.set()
        csv_log.close()
        cloud.flush()
        log.info("Gateway stopped cleanly.")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="TankWatch Gateway — serial reader + anomaly detector"
    )
    parser.add_argument(
        "--port", default=None,
        help="Serial port (e.g. COM3 or /dev/ttyUSB0).  Auto-detected if omitted."
    )
    parser.add_argument(
        "--config", default=DEFAULT_CONFIG,
        help=f"Path to config JSON (default: {DEFAULT_CONFIG})"
    )
    parser.add_argument(
        "--sim",
        choices=[
            "normal", "leak",
            "fill-drift", "fill_drift",
            "flat-supply", "pump_flat_supply",
            "dead-sensor", "sensor_dead",
        ],
        default=None,
        help="Run a built-in simulation scenario instead of real hardware."
    )
    parser.add_argument(
        "--speed", type=float, default=1.0,
        help="Time compression factor (default 1 = real time). Use with --sim or --replay."
    )
    parser.add_argument(
        "--replay", metavar="FILE", default=None,
        help="Replay a raw_log.csv file at adjustable speed."
    )
    parser.add_argument(
        "--quiet", action="store_true",
        help="Force quiet_mode=True (simulates 'nobody home' toggle)."
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true",
        help="Enable DEBUG-level logging."
    )
    args = parser.parse_args()

    _setup_logging(args.verbose)

    cfg   = load_config(args.config)
    cloud = CloudClient(cfg)

    force_quiet = args.quiet or (args.sim in ("leak",))

    if args.replay:
        log.info(f"Replay mode: {args.replay}  speed={args.speed}x")
        source = replay_lines(args.replay, speed=args.speed)

    elif args.sim:
        log.info(f"Simulation mode: {args.sim}  speed={args.speed}x (force_quiet={force_quiet})")
        source = scenario_generator(args.sim, cfg, speed=args.speed)

    else:
        port = args.port or find_arduino_port()
        if not port:
            log.error(
                "No serial port found.  Connect the Arduino and retry, "
                "or use --port COM3 / --sim <scenario>."
            )
            sys.exit(1)
        source = serial_lines(port)

    run_gateway(cfg, source, cloud, force_quiet=force_quiet)


if __name__ == "__main__":
    main()
