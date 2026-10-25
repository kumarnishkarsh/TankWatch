"""TankWatch cloud: FastAPI + SQLite. Run: python server.py
Env: API_KEY, TELEGRAM_TOKEN, TELEGRAM_CHAT_ID, (optional) PORT, DB_PATH,
     ALERT_COOLDOWN_WARN_S (default 120), ALERT_COOLDOWN_CRIT_S (default 60)
All timestamps are UTC epoch seconds. The browser converts to local time."""
import os, time, json, sqlite3, threading, logging
from pathlib import Path
from datetime import datetime, timezone
from typing import Literal, Optional

import requests
import uvicorn
from fastapi import FastAPI, Header, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse
from fastapi.encoders import jsonable_encoder
from fastapi.exceptions import RequestValidationError
from pydantic import BaseModel, field_validator

BASE = Path(__file__).parent
DB_PATH = os.getenv("DB_PATH", str(BASE / "tankwatch.db"))
API_KEY = os.getenv("API_KEY", "")
TG_TOKEN = os.getenv("TELEGRAM_TOKEN", "")
TG_CHAT = os.getenv("TELEGRAM_CHAT_ID", "")
COOLDOWN = {"warning": int(os.getenv("ALERT_COOLDOWN_WARN_S", "120")),
            "critical": int(os.getenv("ALERT_COOLDOWN_CRIT_S", "60"))}
KEEP_S = 24 * 3600
MIN_CYCLE_S = 3  # ignore pump blips shorter than this

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
log = logging.getLogger("tankwatch")
if not API_KEY:
    log.warning("API_KEY not set -> writes are UNPROTECTED (ok for local testing only)")
if not (TG_TOKEN and TG_CHAT):
    log.warning("Telegram not configured -> alerts will only be logged")

# ---------- database ----------
_lock = threading.Lock()
_db = sqlite3.connect(DB_PATH, check_same_thread=False)
_db.row_factory = sqlite3.Row
_db.executescript("""
PRAGMA journal_mode=WAL; PRAGMA synchronous=NORMAL;
CREATE TABLE IF NOT EXISTS readings(ts REAL PRIMARY KEY, dist_mm INTEGER, level_pct REAL,
  level_l REAL, pump INTEGER, state TEXT, rx REAL);
CREATE TABLE IF NOT EXISTS events(id INTEGER PRIMARY KEY AUTOINCREMENT, ts REAL, type TEXT,
  severity TEXT, message TEXT, data TEXT);
CREATE INDEX IF NOT EXISTS ev_ts ON events(ts);
CREATE TABLE IF NOT EXISTS cycles(start REAL PRIMARY KEY, dur_s REAL, ok INTEGER);
CREATE TABLE IF NOT EXISTS kv(k TEXT PRIMARY KEY, v TEXT);
""")


def q(sql, args=(), one=False, commit=False):
    with _lock:
        cur = _db.execute(sql, args)
        if commit:
            _db.commit()
        return (cur.fetchone() if one else cur.fetchall())


_last = q("SELECT pump FROM readings ORDER BY ts DESC LIMIT 1", one=True)
_pump_prev = _last["pump"] if _last else 0
_cycle_start = None
_insert_count = 0
_last_alert = {}

# ---------- models ----------
def to_epoch(v):
    """Contract says UTC epoch seconds. Also tolerate ISO strings ('2026-10-24T19:10:39Z') and epoch milliseconds."""
    if isinstance(v, str):
        try:
            return float(v)
        except ValueError:
            d = datetime.fromisoformat(v.strip().replace("Z", "+00:00"))
            if d.tzinfo is None:
                d = d.replace(tzinfo=timezone.utc)
            return d.timestamp()
    if isinstance(v, (int, float)) and v > 1e11:
        return v / 1000.0
    return v


class Reading(BaseModel):
    @field_validator("ts", mode="before")
    @classmethod
    def _ts(cls, v):
        return to_epoch(v)

    ts: float
    dist_mm: float  # filtered distances can be decimals (e.g. 188.5); stored rounded
    level_pct: float
    level_l: float
    pump: int
    state: Literal["IDLE", "FILLING", "TRIP"]


class Event(BaseModel):
    @field_validator("ts", mode="before")
    @classmethod
    def _ts(cls, v):
        return to_epoch(v)

    ts: float
    type: Literal["DRAWDOWN", "FILL_DRIFT", "PUMP_FLAT_SUPPLY", "PUMP_FLAT_SENSOR",
                  "FAILSAFE_TRIP", "RECOVERED"]
    severity: Literal["info", "warning", "critical"]
    message: str = ""
    data: dict = {}


class Control(BaseModel):
    quiet_mode: bool


def auth(key: Optional[str]):
    if API_KEY and key != API_KEY:
        raise HTTPException(401, "bad or missing X-API-Key")


# ---------- telegram ----------
EMOJI = {"warning": "\U0001F7E0", "critical": "\U0001F534"}
HYPO = {
    "DRAWDOWN": "Level falling while pump is off: leak, stuck float valve or unauthorised use.",
    "FILL_DRIFT": "Fills slower than baseline: foot-valve wear, air lock or pump degradation.",
    "PUMP_FLAT_SUPPLY": "Pump on, level flat, sensor healthy: water supply has failed.",
    "PUMP_FLAT_SENSOR": "Pump on, sensor timing out or stuck: level sensor fault.",
    "FAILSAFE_TRIP": "Edge stopped the pump at max runtime: check supply/sensor, then reset trip.",
    "RECOVERED": "Back to normal.",
}
NUMS = [("loss_l_per_h", "loss {} L/h"), ("pct_slower", "{}% slower"),
        ("fill_s", "fill {} s"), ("baseline_s", "baseline {} s"), ("run_s", "pump ran {} s"),
        ("max_runtime_s", "max run {} s")]


def send_tg(text):
    try:
        r = requests.post(f"https://api.telegram.org/bot{TG_TOKEN}/sendMessage",
                          json={"chat_id": TG_CHAT, "text": text}, timeout=6)
        if r.status_code != 200:
            log.warning("telegram HTTP %s: %s", r.status_code, r.text[:200])
    except Exception as e:  # never crash on alert failure
        log.warning("telegram failed: %s", e)


def maybe_alert(ev: Event):
    if ev.type == "RECOVERED":
        _last_alert.clear()  # next anomaly may alert immediately
    if ev.severity not in ("warning", "critical"):
        return
    now = time.time()
    if now - _last_alert.get(ev.type, 0) < COOLDOWN[ev.severity]:
        log.info("alert suppressed (cooldown): %s", ev.type)
        return
    _last_alert[ev.type] = now
    last = q("SELECT level_pct, level_l FROM readings ORDER BY ts DESC LIMIT 1", one=True)
    nums = [fmt.format(ev.data[k]) for k, fmt in NUMS if k in ev.data]
    if last:
        nums.append(f"level {last['level_pct']:.0f}% ({last['level_l']:.1f} L)")
    text = (f"{EMOJI[ev.severity]} TankWatch {ev.severity.upper()} - {ev.type}\n"
            f"{ev.message}\nLikely cause: {HYPO[ev.type]}\n" + ("Numbers: " + " | ".join(nums) if nums else ""))
    log.info("ALERT -> %s", text.replace("\n", " / "))
    if TG_TOKEN and TG_CHAT:
        threading.Thread(target=send_tg, args=(text,), daemon=True).start()


# ---------- app ----------
app = FastAPI(title="TankWatch cloud")
@app.exception_handler(RequestValidationError)
async def _bad_request(request, exc):  # say WHY a request was rejected (shows in the server window)
    why = "; ".join(f"{'.'.join(str(x) for x in e['loc'][1:])}: {e['msg']}" for e in exc.errors())
    log.warning("422 %s %s -> %s | body=%s", request.method, request.url.path, why, (await request.body())[:300])
    return JSONResponse(status_code=422, content={"detail": jsonable_encoder(exc.errors())})


app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])


@app.get("/api/health")
def health():
    return {"ok": True, "server_ts": time.time()}


@app.post("/api/reading")
def post_reading(r: Reading, x_api_key: Optional[str] = Header(None)):
    global _pump_prev, _cycle_start, _insert_count
    auth(x_api_key)
    now = time.time()
    q("INSERT OR IGNORE INTO readings VALUES(?,?,?,?,?,?,?)",
      (r.ts, int(round(r.dist_mm)), r.level_pct, r.level_l, r.pump, r.state, now), commit=True)
    # incremental fill-cycle tracking (pump 0->1->0)
    if r.pump == 1 and _pump_prev == 0:
        _cycle_start = r.ts
    elif r.pump == 0 and _pump_prev == 1 and _cycle_start is not None:
        dur = r.ts - _cycle_start
        if dur >= MIN_CYCLE_S:
            q("INSERT OR REPLACE INTO cycles VALUES(?,?,?)",
              (_cycle_start, dur, 0 if r.state == "TRIP" else 1), commit=True)
        _cycle_start = None
    _pump_prev = r.pump
    _insert_count += 1
    if _insert_count % 600 == 0:  # prune to last 24 h
        q("DELETE FROM readings WHERE ts < ?", (now - KEEP_S,), commit=True)
        q("DELETE FROM cycles WHERE start < ?", (now - KEEP_S,), commit=True)
    return {"ok": True}


@app.post("/api/event")
def post_event(e: Event, x_api_key: Optional[str] = Header(None)):
    auth(x_api_key)
    q("INSERT INTO events(ts,type,severity,message,data) VALUES(?,?,?,?,?)",
      (e.ts, e.type, e.severity, e.message, json.dumps(e.data)), commit=True)
    maybe_alert(e)
    return {"ok": True}


def ev_row(r):
    return {"id": r["id"], "ts": r["ts"], "type": r["type"], "severity": r["severity"],
            "message": r["message"], "data": json.loads(r["data"] or "{}")}


@app.get("/api/events")
def get_events(limit: int = 50, since: float = 0):
    rows = q("SELECT * FROM events WHERE ts > ? ORDER BY ts DESC, id DESC LIMIT ?", (since, min(limit, 500)))
    return [ev_row(r) for r in rows]


@app.get("/api/readings")
def get_readings(since: float = 0, limit: int = 5000):
    since = max(since, time.time() - KEEP_S)
    rows = q("SELECT ts,dist_mm,level_pct,level_l,pump,state FROM readings WHERE ts > ? "
             "ORDER BY ts ASC LIMIT ?", (since, min(limit, 20000)))
    return [dict(r) for r in rows]


def get_quiet():
    r = q("SELECT v FROM kv WHERE k='quiet_mode'", one=True)
    return bool(r and r["v"] == "1")


@app.get("/api/control")
def get_control():
    return {"quiet_mode": get_quiet()}


@app.post("/api/control")
def post_control(c: Control, x_api_key: Optional[str] = Header(None)):
    auth(x_api_key)
    q("INSERT OR REPLACE INTO kv VALUES('quiet_mode',?)", ("1" if c.quiet_mode else "0",), commit=True)
    return {"quiet_mode": c.quiet_mode}


@app.get("/api/state")
def get_state():
    now = time.time()
    latest = q("SELECT ts,dist_mm,level_pct,level_l,pump,state,rx FROM readings ORDER BY ts DESC LIMIT 1", one=True)
    cyc = q("SELECT start,dur_s,ok FROM cycles WHERE ok=1 ORDER BY start DESC LIMIT 15")
    cyc = [dict(r) for r in reversed(cyc)]
    first = q("SELECT dur_s FROM cycles WHERE ok=1 ORDER BY start ASC LIMIT 4")
    baseline = sum(r["dur_s"] for r in first) / len(first) if len(first) >= 4 else None
    drift = q("SELECT data FROM events WHERE type='FILL_DRIFT' ORDER BY ts DESC LIMIT 1", one=True)
    if drift:
        b = json.loads(drift["data"] or "{}").get("baseline_s")
        baseline = b if b else baseline  # gateway's own baseline wins if it sent one
    recent = q("SELECT 1 FROM cycles WHERE start + dur_s > ? LIMIT 1", (now - 600,), one=True)
    return {
        "server_ts": now,
        "latest": dict(latest) if latest else None,
        "gateway_age_s": (now - latest["rx"]) if latest else None,
        "quiet_mode": get_quiet(),
        "events": [ev_row(r) for r in q("SELECT * FROM events ORDER BY ts DESC, id DESC LIMIT 20")],
        "cycles": cyc,
        "baseline_s": baseline,
        "pump_active_recent": bool(recent) or bool(latest and latest["pump"] == 1),
    }


@app.post("/api/admin/reset")
def reset(x_api_key: Optional[str] = Header(None)):
    global _pump_prev, _cycle_start
    auth(x_api_key)
    for t in ("readings", "events", "cycles", "kv"):
        q(f"DELETE FROM {t}", commit=True)
    _pump_prev, _cycle_start = 0, None
    _last_alert.clear()
    return {"ok": True, "wiped": True}


@app.get("/")
def index():
    return FileResponse(BASE / "dashboard.html")


@app.get("/chart.umd.min.js")
def chartjs():  # local Chart.js fallback if the file is placed next to server.py
    f = BASE / "chart.umd.min.js"
    if not f.exists():
        raise HTTPException(404)
    return FileResponse(f, media_type="application/javascript")


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=int(os.getenv("PORT", "8000")))
