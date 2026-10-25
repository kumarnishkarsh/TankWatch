#!/usr/bin/env python3
"""Fake gateway. Usage: python simulate.py <scenario> [--url http://localhost:8000]
Scenarios: normal drawdown drift flat_supply flat_sensor failsafe recovered all
Env: API_KEY (must match the server). 1 reading per second, real timestamps."""
import os, time, random, argparse
import requests

p = argparse.ArgumentParser()
p.add_argument("scenario", choices=["normal", "drawdown", "drift", "flat_supply",
                                    "flat_sensor", "failsafe", "recovered", "all"])
p.add_argument("--url", default="http://localhost:8000")
a = p.parse_args()
H = {"X-API-Key": os.getenv("API_KEY", "")}
S = requests.Session()
TANK_H, OFFSET, CAP = 300, 30, 15.0   # mm, mm, litres at 100%


class T:  # tank state
    l, pump, state = 10.0, 0, "IDLE"


def post(path, body):
    try:
        S.post(a.url + path, json=body, headers=H, timeout=4).raise_for_status()
    except Exception as e:
        print("  POST failed", path, e)


def pct():
    return max(0.0, min(100.0, 100 * T.l / CAP))


def tick(inflow=0.0, outflow=0.0, sensor_ok=True):
    T.l = max(0.0, min(CAP, T.l + inflow - outflow))
    pc = pct()
    dist = int(OFFSET + TANK_H * (1 - pc / 100) + random.randint(-2, 2)) if sensor_ok else -1
    post("/api/reading", {"ts": time.time(), "dist_mm": dist, "level_pct": round(pc, 1),
                          "level_l": round(T.l, 2), "pump": T.pump, "state": T.state})
    time.sleep(1)


def event(t, sev, msg, **data):
    post("/api/event", {"ts": time.time(), "type": t, "severity": sev, "message": msg, "data": data})
    print(f"  EVENT {sev:8s} {t}: {msg}")


def cycle(inflow=1.0):
    """drain to 30%, then fill to 90%; returns fill seconds"""
    T.pump, T.state = 0, "IDLE"
    while pct() > 30:
        tick(outflow=0.6)
    T.pump, T.state = 1, "FILLING"
    t0 = time.time()
    while pct() < 90:
        tick(inflow=inflow)
    T.pump, T.state = 0, "IDLE"
    tick()
    return time.time() - t0


def normal(n=5):
    for i in range(n):
        print(f"  clean cycle {i+1}/{n}: fill {cycle():.0f}s")


def drift(baseline_cycles=4):
    normal(baseline_cycles)
    base = 9.0
    for inflow in (0.7, 0.55):
        s = cycle(inflow)
        print(f"  slow cycle: fill {s:.0f}s")
        event("FILL_DRIFT", "warning", f"Fill time {s:.0f}s vs baseline {base:.0f}s",
              fill_s=round(s), baseline_s=base, pct_slower=round(100 * (s / base - 1)))


def drawdown():
    post("/api/control", {"quiet_mode": True})
    T.l, T.pump, T.state = 12.0, 0, "IDLE"
    for _ in range(5):
        tick()
    for i in range(25):
        tick(outflow=0.05)
        if i == 20:
            event("DRAWDOWN", "warning", "Level falling during quiet hours with pump off",
                  loss_l_per_h=180, window_s=20)
    S.post(a.url + "/api/control", json={"quiet_mode": False}, headers=H, timeout=4)


def flat_supply():
    T.l, T.pump, T.state = 5.0, 1, "FILLING"
    for _ in range(20):
        tick()
    event("PUMP_FLAT_SUPPLY", "warning", "Pump on 20s, level flat, sensor readings valid", run_s=20)
    T.pump, T.state = 0, "IDLE"
    tick()


def flat_sensor():
    T.pump, T.state = 1, "FILLING"
    for _ in range(15):
        tick(sensor_ok=False)
    event("PUMP_FLAT_SENSOR", "warning", "Pump on, sensor timeouts (dist -1)", run_s=15)
    T.pump, T.state = 0, "IDLE"
    tick()


def failsafe():
    T.l, T.pump, T.state = 4.0, 1, "FILLING"
    for _ in range(25):
        tick(inflow=0.02)
    T.pump, T.state = 0, "TRIP"
    event("FAILSAFE_TRIP", "critical", "Pump stopped at max runtime", max_runtime_s=25)
    for _ in range(5):
        tick()


def recovered():
    T.pump, T.state = 0, "IDLE"
    event("RECOVERED", "info", "Trip reset, back to normal")
    for _ in range(3):
        tick()


SC = {"normal": normal, "drawdown": drawdown, "drift": drift, "flat_supply": flat_supply,
      "flat_sensor": flat_sensor, "failsafe": failsafe, "recovered": recovered}
print("scenario:", a.scenario, "->", a.url)
if a.scenario == "all":
    normal(5)
    for name in ("drift", "drawdown", "flat_supply", "flat_sensor", "failsafe", "recovered"):
        print("==", name)
        (lambda: drift(0))() if name == "drift" else SC[name]()
else:
    SC[a.scenario]()
print("done")
