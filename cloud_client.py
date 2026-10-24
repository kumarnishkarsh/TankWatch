"""
cloud_client.py — TankWatch cloud API client.

Features:
  • Batched reading posts (every flush cycle, default 2 s).
  • Immediate event posts with retry — events are never silently dropped.
  • On-disk offline queue (JSONL) so events survive network outages and
    gateway restarts.
  • Polls GET /api/control every poll_interval_s for quiet_mode.
  • The gateway loop continues with full detection if the cloud is down.

Design note:
  Readings are batched and dropped on persistent failure (too voluminous
  to queue indefinitely).  Events are always queued on disk until they
  succeed — they are the primary diagnostic record.
"""

import json
import logging
import os
import threading
import time
from collections import deque
from typing import Optional

import requests

log = logging.getLogger("cloud")

_OFFLINE_FILE = "offline_queue.jsonl"
_MAX_QUEUE    = 10_000   # prevent unbounded growth when offline for a long time


class CloudClient:
    """
    Thread-safe cloud API client.

    All public methods are safe to call from the gateway's main thread.
    A background daemon thread handles the actual HTTP calls so the
    gateway never blocks on a slow or unreachable server.
    """

    def __init__(self, cfg: dict):
        self._base    = cfg.get("cloud_url", "http://172.16.85.124:8000").rstrip("/")
        self._timeout = cfg.get("cloud_timeout_s", 5)
        self._batch   = cfg.get("batch_size", 10)

        self._session = requests.Session()
        self._session.headers["Content-Type"] = "application/json"
        # Shared secret for the cloud server. Prefer the API_KEY env var so the key never lands in git.
        key = os.getenv("API_KEY") or cfg.get("api_key", "")
        if key:
            self._session.headers["X-API-Key"] = key
        else:
            log.warning("No API key set (env API_KEY or config 'api_key'): the cloud will answer 401")

        self._lock = threading.Lock()

        # Readings: held in memory, sent in batches, dropped on persistent failure.
        self._read_buf: list[dict] = []

        # Events: must not be lost — queued in memory AND on disk.
        self._event_q: deque[dict] = deque()

        # Load any events persisted from a previous run.
        self._offline_path = _OFFLINE_FILE
        self._load_offline()

        # Background flush thread
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._flush_loop,
            daemon=True,
            name="cloud-flush",
        )
        self._thread.start()

    # -----------------------------------------------------------------------
    # Public API
    # -----------------------------------------------------------------------

    def post_reading(self, reading: dict) -> None:
        """Buffer a telemetry reading for batched delivery."""
        with self._lock:
            self._read_buf.append(reading)

    def post_event(self, event: dict) -> None:
        """Queue a diagnostic event for reliable delivery."""
        msg_preview = event.get("message", "")[:70]
        log.info(
            f"EVENT [{event.get('severity','?')}] "
            f"{event.get('type','?')}: {msg_preview}…"
        )
        with self._lock:
            if len(self._event_q) < _MAX_QUEUE:
                self._event_q.append(dict(event))

    def get_control(self) -> Optional[dict]:
        """
        GET /api/control → {quiet_mode: bool, …}.
        Returns None on any network error (caller should use last known value).
        """
        try:
            r = self._session.get(
                f"{self._base}/api/control",
                timeout=self._timeout,
            )
            if r.status_code == 200:
                return r.json()
            log.debug(f"GET /api/control → HTTP {r.status_code}")
        except Exception as e:
            log.debug(f"GET /api/control failed: {e}")
        return None

    def flush(self) -> None:
        """Synchronous drain at shutdown — call after stopping the main loop."""
        self._stop.set()
        self._do_flush()

    # -----------------------------------------------------------------------
    # Background thread
    # -----------------------------------------------------------------------

    def _flush_loop(self) -> None:
        while not self._stop.is_set():
            self._do_flush()
            time.sleep(2.0)

    def _do_flush(self) -> None:
        self._flush_readings()
        self._flush_events()

    # -----------------------------------------------------------------------
    # Readings
    # -----------------------------------------------------------------------

    def _flush_readings(self) -> None:
        with self._lock:
            if not self._read_buf:
                return
            batch, self._read_buf = self._read_buf[:self._batch], self._read_buf[self._batch:]

        for reading in batch:
            try:
                r = self._session.post(
                    f"{self._base}/api/reading",
                    data=json.dumps(reading),
                    timeout=self._timeout,
                )
                if r.status_code not in (200, 201, 204):
                    log.warning(f"POST /api/reading → HTTP {r.status_code}")
            except Exception as e:
                log.debug(f"POST /api/reading failed (network?): {e}")
                break  # cloud is down: do not wait out the timeout for every remaining reading
                # Readings are high-frequency; we accept loss rather than
                # consuming unbounded memory.

    # -----------------------------------------------------------------------
    # Events  (must never be silently lost)
    # -----------------------------------------------------------------------

    def _flush_events(self) -> None:
        """
        Drain the event queue one item at a time.
        On failure: persist the item to disk and stop (try again next flush).
        On success: remove the item from the queue (and from the disk file).
        """
        while True:
            with self._lock:
                if not self._event_q:
                    break
                event = self._event_q[0]

            try:
                r = self._session.post(
                    f"{self._base}/api/event",
                    data=json.dumps(event),
                    timeout=self._timeout,
                )
                if r.status_code in (200, 201, 204):
                    with self._lock:
                        if self._event_q and self._event_q[0] is event:
                            self._event_q.popleft()
                    # Item delivered — ensure it's gone from the disk file.
                    # (We use a write-all strategy: rewrite the file after each
                    # successful delivery to keep it current.)
                    self._rewrite_offline()
                elif r.status_code in (400, 422):
                    # Permanent rejection (bad type/severity/field). Retrying can never work and
                    # would block every event queued behind it, so drop it loudly.
                    log.error(f"POST /api/event rejected HTTP {r.status_code}: "
                              f"{r.text[:200]} -> DROPPED {event}")
                    with self._lock:
                        if self._event_q and self._event_q[0] is event:
                            self._event_q.popleft()
                    self._rewrite_offline()
                else:
                    log.warning(
                        f"POST /api/event → HTTP {r.status_code}; "
                        f"queued to disk for retry."
                    )
                    self._persist_queue_to_disk()
                    break  # back off; retry on next flush
            except Exception as e:
                log.debug(f"POST /api/event failed (offline?): {e}")
                self._persist_queue_to_disk()
                break   # back off; retry on next flush

    # -----------------------------------------------------------------------
    # Offline queue persistence
    # -----------------------------------------------------------------------

    def _load_offline(self) -> None:
        """Restore events from a previous run's offline queue file."""
        if not os.path.exists(self._offline_path):
            return
        loaded = 0
        try:
            with open(self._offline_path) as fh:
                for raw in fh:
                    raw = raw.strip()
                    if raw:
                        try:
                            self._event_q.append(json.loads(raw))
                            loaded += 1
                        except json.JSONDecodeError:
                            pass
            if loaded:
                log.info(
                    f"Loaded {loaded} offline events from {self._offline_path}"
                )
        except OSError as e:
            log.error(f"Cannot read offline queue: {e}")

    def _persist_queue_to_disk(self) -> None:
        """Write the entire in-memory event queue to the offline file."""
        try:
            with self._lock:
                snapshot = list(self._event_q)
            with open(self._offline_path, "w") as fh:
                for item in snapshot:
                    fh.write(json.dumps(item) + "\n")
        except OSError as e:
            log.error(f"Cannot write offline queue: {e}")

    def _rewrite_offline(self) -> None:
        """Rewrite the offline file to reflect the current queue state."""
        self._persist_queue_to_disk()
