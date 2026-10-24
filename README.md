# TankWatch — Python Gateway + Analytics

A robust, hackathon-tested Python 3.10+ gateway and statistical anomaly detection layer for the **TankWatch** water-tank controller.

TankWatch adds an intelligent early-diagnosis layer over an autonomous Arduino Uno hardware controller (HC-SR04 ultrasonic sensor, active-LOW relay, 16x2 I2C LCD, buzzer). The edge hardware maintains a failsafe hysteresis control loop with zero dependencies on the gateway or internet, while this Python gateway processes telemetry over USB serial, filters sensor noise, detects statistical anomalies, and forwards readings and alarms to the Cloud API.

---

## 1. Project Structure

```
Cloud Nexus/
├── config.json             # Calibration & threshold settings
├── detectors.py            # Three statistical anomaly detectors (Drawdown, FillDrift, PumpFlat)
├── gateway.py              # Serial reader, median preprocessor, cloud dispatcher, CLI
├── cloud_client.py         # Resilient HTTP client with local on-disk offline queue
├── simulator.py            # Hardware-free serial stream generator for all failure modes
├── requirements.txt        # Minimal dependencies (pyserial, requests, pytest)
├── README.md               # Setup, usage, tuning, and testing guide
└── tests/
    ├── __init__.py
    └── test_detectors.py   # Complete pytest test suite (27 unit tests)
```

---

## 2. Frozen Interface Contract

### Serial Interface (115200 baud, line-based)
- **Uno → PC Telemetry (every 1000 ms):**  
  `D,<uptime_ms>,<dist_mm>,<pump 0|1>,<state>`  
  *(where `dist_mm = -1` on ultrasonic echo timeout; `state` in `IDLE|FILLING|TRIP`)*
- **Uno → PC Events:**  
  `E,<uptime_ms>,<code>`  
  *(codes: `PUMP_ON`, `PUMP_OFF_HIGH`, `TRIP_MAXRUN`, `SENSOR_FAULT`)*
- **PC → Uno Commands (optional):**  
  `K,RESET_TRIP` | `K,PUMP_FORCE_OFF` | `K,PUMP_AUTO`

### Cloud API Contract
- `POST /api/reading`: `{"ts": str, "dist_mm": int, "level_pct": float, "level_l": float, "pump": int, "state": str}`
- `POST /api/event`: `{"ts": str, "type": str, "severity": str, "message": str, "data": dict}`
- `GET /api/state`
- `GET /api/readings?since=<ts>`
- `GET /api/events`
- `GET /api/control`: returns `{"quiet_mode": bool}`
- `POST /api/control`: accepts `{"quiet_mode": bool}`

**Event Types:** `DRAWDOWN`, `FILL_DRIFT`, `PUMP_FLAT_SUPPLY`, `PUMP_FLAT_SENSOR`, `FAILSAFE_TRIP`, `RECOVERED`.  
**Severities:** `info`, `warning`, `critical`.

### Level and Volume Formulas
$$\text{level\_pct} = \operatorname{clamp}\left(100 \times \frac{\text{tank\_height\_mm} - (\text{dist\_mm} - \text{sensor\_offset\_mm})}{\text{tank\_height\_mm}}, 0, 100\right)$$

$$\text{level\_fraction} = \frac{\text{level\_pct}}{100}$$

$$\text{tank\_volume\_l} = \frac{\text{tank\_area\_cm2} \times \text{tank\_height\_mm}}{10\,000}$$

$$\text{level\_l} = \text{level\_fraction} \times \text{tank\_volume\_l}$$

*Dimensional check:* $\text{tank\_area\_cm2} \times (\text{tank\_height\_mm} / 10\text{ cm/mm}) / 1\,000\text{ cm}^3/\text{L} = \frac{\text{tank\_area\_cm2} \times \text{tank\_height\_mm}}{10\,000}$. For a standard 1,000 cm² bench tank with 300 mm depth, total volume is exactly $30.0\text{ L}$.

---

## 3. Installation & Quick Start

### Prerequisites
- Python 3.10+ (tested on Python 3.12)
- Virtual environment recommended:

```bash
# Clone or navigate to the repository
cd "c:\Users\Nishkarsh\Desktop\Cloud Nexus"

# Install dependencies
pip install -r requirements.txt
```

### Running the Test Suite
```bash
pytest tests/test_detectors.py -v
```
All 27 unit tests will pass in < 0.5s.

---

## 4. Running the Gateway

### A. Real Hardware Mode
Plug in the Arduino Uno USB cable. The gateway auto-detects USB-Serial / CH340 devices:
```bash
python gateway.py
```
Or specify the port explicitly:
```bash
python gateway.py --port COM3
```

### B. Hardware-Free Simulation Mode
The built-in simulator enables the entire team to test without hardware:

```bash
# 1. Normal cycles (smoke test)
python gateway.py --sim normal --speed 10

# 2. Quiet hours leak / unauthorized drawdown
python gateway.py --sim leak --speed 20

# 3. Foot-valve wear / supply restriction (fill-time drift)
python gateway.py --sim fill-drift --speed 30

# 4. Supply failure (pump on, level flat, valid sensor)
python gateway.py --sim flat-supply --speed 20

# 5. Sensor failure (pump on, echo timeout / stuck sensor)
python gateway.py --sim dead-sensor --speed 20
```

*(Both hyphenated flags like `--sim fill-drift` and underscore names like `--sim fill_drift` are supported).*

### C. CSV Log Replay Mode
Every line from real runs is logged to `raw_log.csv`. Replay any session at variable speed:
```bash
python gateway.py --replay raw_log.csv --speed 5
```

---

## 5. Offline Queue & Cloud Resilience

- **Non-blocking Dispatch:** Telemetry readings and diagnostic events are buffered in memory; a background worker manages HTTP sessions to prevent I/O blocking on serial reads.
- **On-Disk Offline Queue (`offline_queue.jsonl`):** If the Cloud API goes offline or the network drops, diagnostic events are preserved on disk. They are retried in chronological sequence upon reconnection.
- **Standalone Execution:** If the cloud server is unreachable, the gateway continues full statistical anomaly detection locally and logs to standard output and `gateway.log`.

---

## 6. Threshold Tuning & HC-SR04 Noise Guide

1. **HC-SR04 Noise Envelope:** The raw sensor has approximately $\pm 3\text{ mm}$ jitter. The 5-sample rolling median filter reduces residual variance to $\approx \pm 1.5\text{ mm}$.
2. **`drawdown_threshold_mm` (default: 15 mm):** Comparing two median values gives a combined standard error $\sigma \approx \sqrt{1.5^2 + 1.5^2} \approx 2.1\text{ mm}$. A threshold of $15\text{ mm}$ corresponds to $> 7\sigma$, preventing noise false positives while reliably catching real drops.
3. **`fill_drift_ratio` (default: 1.3):** Requires two consecutive cycles exceeding $130\%$ of the EWMA baseline to trigger `FILL_DRIFT`. Single anomalous cycles (e.g., momentary pressure dips) do not trigger an alarm.
4. **`pump_flat_check_s` (default: 15 s) & `pump_flat_rise_mm` (default: 10 mm):** Verifies that the pump raises the level by at least $10\text{ mm}$ after $15\text{ s}$. Distinguishes between supply failure (constant readings) and sensor failure (timeout streak $\ge 5$ or zero variance).
