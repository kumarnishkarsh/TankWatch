"""
simulator.py — TankWatch fake serial source.

Emits realistic D/E lines that gateway.py can consume directly
(via --sim <scenario> --speed N) so the whole team can develop
and test without any hardware.

Scenarios
---------
normal          Standard pump cycles, no anomalies.
leak            Quiet hours; tank drains slowly while pump is off.
fill_drift      Fill time degrades across cycles (simulates foot-valve wear).
pump_flat_supply  Pump on but level stays flat (empty supply line).
sensor_dead     Pump on, HC-SR04 returns only -1 timeouts.

Public entry point
------------------
  from simulator import scenario_generator
  for line in scenario_generator("leak", cfg, speed=10):
      process(line)
"""

import logging
import random
import time
from typing import Generator

log = logging.getLogger("simulator")


# ---------------------------------------------------------------------------
# First-order tank physics model
# ---------------------------------------------------------------------------

class TankModel:
    """
    Minimal tank model that mimics the Uno's pump-control hysteresis loop.

    Coordinate convention (matches HC-SR04 mounted above the water):
        dist_mm increases  →  water level falls
        dist_mm decreases  →  water level rises

    dist_full = sensor_offset_mm              (tank 100 % full)
    dist_empty = sensor_offset_mm + tank_height_mm  (tank 0 % full)
    """

    def __init__(self, cfg: dict):
        h              = cfg["tank_height_mm"]
        off            = cfg["sensor_offset_mm"]
        self.tank_h    = h
        self.offset    = off
        self.area      = cfg["tank_area_cm2"]
        self.max_rt    = cfg["max_runtime_s"]

        self.dist_full  = float(off)
        self.dist_empty = float(off + h)

        # Hysteresis thresholds (in dist_mm)
        #   LOW level  → pump ON
        self.dist_low  = self.dist_empty - h * cfg["low_pct"]  / 100.0
        #   HIGH level → pump OFF
        self.dist_high = self.dist_empty - h * cfg["high_pct"] / 100.0

        # Current sensor distance (start just below LOW threshold → pump comes on)
        self.current_dist = self.dist_low + 5.0

        self.pump_on      = False
        self.pump_on_time = 0.0
        self.state        = "IDLE"

    def add_noise(self, dist: float) -> int:
        """HC-SR04 real-world noise: ±3 mm uniform."""
        return int(round(dist + random.uniform(-3.0, 3.0)))

    def step(
        self,
        dt: float,
        fill_rate_mm_s: float = 4.0,
        drain_rate_mm_s: float = 0.0,
    ) -> tuple[int, bool, str]:
        """
        Advance the model by `dt` real seconds (at speed=1, dt≈1).

        Returns
        -------
        (dist_mm_with_noise, pump_on, state_string)
        """
        # Hysteresis pump control (mirrors the Uno firmware)
        if not self.pump_on and self.current_dist >= self.dist_low:
            self.pump_on      = True
            self.pump_on_time = 0.0
            self.state        = "FILLING"

        if self.pump_on and self.current_dist <= self.dist_high:
            self.pump_on = False
            self.state   = "IDLE"

        # Max-runtime failsafe
        if self.pump_on:
            self.pump_on_time += dt
            if self.pump_on_time >= self.max_rt:
                self.pump_on = False
                self.state   = "TRIP"

        # Physics
        if self.pump_on:
            self.current_dist -= fill_rate_mm_s * dt   # level rises
        self.current_dist += drain_rate_mm_s * dt       # background drain

        # Clamp to physical limits
        self.current_dist = max(self.dist_full, min(self.dist_empty, self.current_dist))

        return self.add_noise(self.current_dist), self.pump_on, self.state

    @property
    def pct(self) -> float:
        return 100.0 * (self.dist_empty - self.current_dist) / self.tank_h


# ---------------------------------------------------------------------------
# Line builders (match the frozen serial contract)
# ---------------------------------------------------------------------------

def _d(uptime_ms: int, dist_mm: int, pump: bool, state: str) -> str:
    return f"D,{uptime_ms},{dist_mm},{int(pump)},{state}"


def _e(uptime_ms: int, code: str) -> str:
    return f"E,{uptime_ms},{code}"


# ---------------------------------------------------------------------------
# Scenario generators
# ---------------------------------------------------------------------------

def _normal(cfg: dict, speed: float) -> Generator[str, None, None]:
    """
    Healthy pump cycles.  Useful for verifying the baseline path and for
    seeding the fill-drift detector.
    """
    tank       = TankModel(cfg)
    tick       = 1.0 / speed
    uptime_ms  = 0
    prev_pump  = False

    while True:
        # If pump is on, fill at 5 mm/s; if pump is off, normal consumption drains at 3 mm/s
        drain_rate = 0.0 if tank.pump_on else 3.0
        dist, pump, state = tank.step(1.0, fill_rate_mm_s=5.0, drain_rate_mm_s=drain_rate)

        if pump and not prev_pump:
            yield _e(uptime_ms, "PUMP_ON")
        if not pump and prev_pump and state == "IDLE":
            yield _e(uptime_ms, "PUMP_OFF_HIGH")
        if not pump and prev_pump and state == "TRIP":
            yield _e(uptime_ms, "TRIP_MAXRUN")

        yield _d(uptime_ms, dist, pump, state)
        prev_pump  = pump
        uptime_ms += 1000
        time.sleep(tick)


def _leak(cfg: dict, speed: float) -> Generator[str, None, None]:
    """
    Demo scenario:
      1. Complete 1 normal cycle: pump fills tank to HIGH level (90%), then turns OFF.
      2. Pump rests and settles (satisfies pump_off_settle_s).
      3. Enter quiet hours: drain valve opens (leak begins).
      4. DrawdownDetector detects drop > threshold within drawdown_window_s and fires DRAWDOWN.
    """
    tank       = TankModel(cfg)
    tick       = 1.0 / speed
    uptime_ms  = 0
    prev_pump  = False
    cycles     = 0

    log.info("Simulator [leak]: running 1 normal fill cycle to reach HIGH level…")

    # Start near HIGH level (75% full) so demo fills to HIGH quickly
    h = cfg["tank_height_mm"]
    off = cfg["sensor_offset_mm"]
    tank.current_dist = (off + h) - h * 0.75
    tank.pump_on = True
    tank.state = "FILLING"
    yield _e(uptime_ms, "PUMP_ON")

    # Phase 1 — fill until HIGH level (90%), pump turns OFF
    while cycles < 1:
        dist, pump, state = tank.step(1.0, fill_rate_mm_s=5.0, drain_rate_mm_s=0.0)

        if pump and not prev_pump:
            yield _e(uptime_ms, "PUMP_ON")
        if not pump and prev_pump and state == "IDLE":
            yield _e(uptime_ms, "PUMP_OFF_HIGH")
            cycles += 1

        yield _d(uptime_ms, dist, pump, state)
        prev_pump  = pump
        uptime_ms += 1000
        time.sleep(tick)

    # Pump rests for settle period (default 5-10s)
    settle_s = cfg.get("pump_off_settle_s", 10)
    log.info(f"Simulator [leak]: pump reached HIGH level, resting {settle_s}s before leak…")
    for _ in range(settle_s):
        dist, _, _ = tank.step(1.0, fill_rate_mm_s=0.0, drain_rate_mm_s=0.0)
        yield _d(uptime_ms, dist, False, "IDLE")
        uptime_ms += 1000
        time.sleep(tick)

    # Phase 2 — quiet hours + leak!
    log.info("Simulator [leak]: entering leak phase (pump forced off, drain open)")
    leak_rate = cfg.get("sim_leak_rate_mm_per_s", 1.0)   # 1.0 mm/s leak

    for _ in range(600):   # 10 minutes of simulated time
        dist, _, _ = tank.step(1.0, fill_rate_mm_s=0.0, drain_rate_mm_s=leak_rate)
        # Force pump off to simulate nobody-home quiet window
        yield _d(uptime_ms, dist, False, "IDLE")
        uptime_ms += 1000
        time.sleep(tick)


def _fill_drift(cfg: dict, speed: float) -> Generator[str, None, None]:
    """
    Each fill cycle is slower than the last.
    Cycles 1-4 are normal (seed baseline at ~5 mm/s fill rate).
    From cycle 5 onwards the fill rate degrades by 15 % per cycle,
    so the fill time increases until FILL_DRIFT fires.
    """
    tank          = TankModel(cfg)
    tick          = 1.0 / speed
    uptime_ms     = 0
    prev_pump     = False
    cycle         = 0
    base_rate     = 5.0   # mm/s

    while True:
        # Degrade fill rate after the seeding phase
        fill_rate = base_rate * (0.85 ** max(0, cycle - 4))
        drain_rate = 0.0 if tank.pump_on else 4.0
        dist, pump, state = tank.step(1.0, fill_rate_mm_s=fill_rate, drain_rate_mm_s=drain_rate)

        if pump and not prev_pump:
            yield _e(uptime_ms, "PUMP_ON")
        if not pump and prev_pump and state == "IDLE":
            yield _e(uptime_ms, "PUMP_OFF_HIGH")
            cycle += 1
            log.info(f"Simulator [fill_drift]: cycle {cycle}, fill_rate={fill_rate:.2f} mm/s")
        if not pump and prev_pump and state == "TRIP":
            yield _e(uptime_ms, "TRIP_MAXRUN")
            cycle += 1

        yield _d(uptime_ms, dist, pump, state)
        prev_pump  = pump
        uptime_ms += 1000
        time.sleep(tick)


def _pump_flat_supply(cfg: dict, speed: float) -> Generator[str, None, None]:
    """
    First 2 normal cycles (pump works fine), then the supply line goes
    dry: pump runs but fill_rate = 0.  The PumpFlatDetector should fire
    PUMP_FLAT_SUPPLY within pump_flat_check_s seconds.
    """
    tank       = TankModel(cfg)
    tick       = 1.0 / speed
    uptime_ms  = 0
    prev_pump  = False
    cycles     = 0

    while True:
        rate = 5.0 if cycles < 2 else 0.0   # supply fails after 2 cycles
        drain_rate = 0.0 if tank.pump_on else 4.0
        dist, pump, state = tank.step(1.0, fill_rate_mm_s=rate, drain_rate_mm_s=drain_rate)

        if pump and not prev_pump:
            yield _e(uptime_ms, "PUMP_ON")
        if not pump and prev_pump:
            if state == "IDLE":
                yield _e(uptime_ms, "PUMP_OFF_HIGH")
            elif state == "TRIP":
                yield _e(uptime_ms, "TRIP_MAXRUN")
            cycles += 1

        yield _d(uptime_ms, dist, pump, state)
        prev_pump  = pump
        uptime_ms += 1000
        time.sleep(tick)


def _sensor_dead(cfg: dict, speed: float) -> Generator[str, None, None]:
    """
    First 2 normal cycles, then while the pump is on the sensor returns
    -1 (simulating a disconnected HC-SR04).  The PumpFlatDetector should
    fire PUMP_FLAT_SENSOR.
    """
    tank       = TankModel(cfg)
    tick       = 1.0 / speed
    uptime_ms  = 0
    prev_pump  = False
    cycles     = 0

    while True:
        drain_rate = 0.0 if tank.pump_on else 4.0
        dist, pump, state = tank.step(1.0, fill_rate_mm_s=5.0, drain_rate_mm_s=drain_rate)

        # Corrupt the reading after 2 normal cycles if pump is on
        if cycles >= 2 and pump:
            dist = -1

        if pump and not prev_pump:
            yield _e(uptime_ms, "PUMP_ON")
        if not pump and prev_pump:
            if state == "IDLE":
                yield _e(uptime_ms, "PUMP_OFF_HIGH")
            elif state == "TRIP":
                yield _e(uptime_ms, "TRIP_MAXRUN")
            cycles += 1

        yield _d(uptime_ms, dist, pump, state)
        prev_pump  = pump
        uptime_ms += 1000
        time.sleep(tick)


# ---------------------------------------------------------------------------
# Public entry point
# ---------------------------------------------------------------------------

_SCENARIOS: dict[str, object] = {
    "normal":           _normal,
    "leak":             _leak,
    "fill_drift":       _fill_drift,
    "fill-drift":       _fill_drift,
    "pump_flat_supply": _pump_flat_supply,
    "flat-supply":      _pump_flat_supply,
    "pump-flat-supply": _pump_flat_supply,
    "sensor_dead":      _sensor_dead,
    "dead-sensor":      _sensor_dead,
}


def scenario_generator(
    name: str,
    cfg: dict,
    speed: float = 1.0,
) -> Generator[str, None, None]:
    """
    Return a generator that yields serial-style lines for the named scenario.

    Parameters
    ----------
    name   One of: normal, leak, fill_drift, pump_flat_supply, sensor_dead.
    cfg    Config dict from config.json.
    speed  Time compression factor (speed=10 runs 10x faster than real time).
    """
    if name not in _SCENARIOS:
        raise ValueError(
            f"Unknown scenario {name!r}. "
            f"Valid choices: {list(_SCENARIOS)}"
        )
    log.info(f"Starting simulator scenario={name!r} speed={speed}x")
    yield from _SCENARIOS[name](cfg, speed)
