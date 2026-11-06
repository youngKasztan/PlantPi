"""SQLite log: samples, watering events and manual watering requests."""
from __future__ import annotations

import sqlite3
from datetime import datetime, timezone
from pathlib import Path

SCHEMA = """
CREATE TABLE IF NOT EXISTS samples (
    id        INTEGER PRIMARY KEY,
    ts_utc    TEXT NOT NULL,          -- wall clock; trust only when synced = 1
    synced    INTEGER NOT NULL,       -- 1 if NTP had synchronised the clock
    node_ok   INTEGER NOT NULL,       -- 1 if fresh data from the Pico sensor node
    node_seq  INTEGER,
    tank_mv   INTEGER,
    tank_pct  REAL,                   -- NULL when not calibrated or invalid
    air_c     REAL,                   -- BME280/BMP280 on the node; NULL if none
    air_rh    REAL,                   -- NULL with a BMP280 (no humidity sensor)
    air_hpa   REAL
);
CREATE TABLE IF NOT EXISTS soil (
    sample_id INTEGER NOT NULL REFERENCES samples(id),
    pot       INTEGER NOT NULL,       -- 0 = Pot 1, 1 = Pot 2
    sensor    TEXT NOT NULL,          -- name sent by the node, e.g. pot1
    mv        INTEGER,                -- millivolts from the Pico
    pct       REAL,                   -- NULL when not calibrated or invalid
    valid     INTEGER NOT NULL
);
CREATE TABLE IF NOT EXISTS events (
    id        INTEGER PRIMARY KEY,
    ts_utc    TEXT NOT NULL,
    synced    INTEGER NOT NULL,
    pot       INTEGER,
    kind      TEXT NOT NULL,          -- watered | refused | fault | start | stop
    seconds   REAL,
    ml        REAL,
    reason    TEXT
);
CREATE TABLE IF NOT EXISTS requests (
    id        INTEGER PRIMARY KEY,
    ts_utc    TEXT NOT NULL,
    pot       INTEGER NOT NULL,
    ml        REAL NOT NULL,
    status    TEXT NOT NULL DEFAULT 'pending',   -- pending | done | refused
    note      TEXT
);
"""


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


class Store:
    def __init__(self, path: str | Path):
        if str(path) != ":memory:":
            Path(path).parent.mkdir(parents=True, exist_ok=True)
        self.db = sqlite3.connect(str(path), timeout=10)
        self.db.row_factory = sqlite3.Row
        self.db.execute("PRAGMA journal_mode=WAL")
        if self.db.execute("SELECT 1 FROM sqlite_master WHERE name='samples'").fetchone():
            cols = {r[1] for r in self.db.execute("PRAGMA table_info(samples)")}
            if "node_ok" not in cols:
                raise RuntimeError(f"{path} was made by PlantPi v1 - move it away and start a new log")
        self.db.executescript(SCHEMA)
        self.db.commit()

    def log_sample(self, synced: bool, node_ok: bool, node_seq, tank_mv, tank_pct,
                   soil: list[dict], env: dict | None = None) -> int:
        env = env or {}
        cur = self.db.execute(
            "INSERT INTO samples (ts_utc, synced, node_ok, node_seq, tank_mv, tank_pct, "
            "air_c, air_rh, air_hpa) VALUES (?,?,?,?,?,?,?,?,?)",
            (utc_now(), int(synced), int(node_ok), node_seq, tank_mv, tank_pct,
             env.get("t_c"), env.get("rh"), env.get("hpa")))
        sid = cur.lastrowid
        self.db.executemany(
            "INSERT INTO soil (sample_id, pot, sensor, mv, pct, valid) VALUES (?,?,?,?,?,?)",
            [(sid, s["pot"], s["sensor"], s["mv"], s["pct"], int(s["valid"])) for s in soil])
        self.db.commit()
        return sid

    def log_event(self, synced: bool, kind: str, pot=None, seconds=None, ml=None, reason="") -> None:
        self.db.execute(
            "INSERT INTO events (ts_utc, synced, pot, kind, seconds, ml, reason) VALUES (?,?,?,?,?,?,?)",
            (utc_now(), int(synced), pot, kind, seconds, ml, reason))
        self.db.commit()

    def recent_doses(self, pot: int, within_s: float) -> list[float]:
        """Ages in seconds of automatic doses of `pot` logged with a synced clock."""
        rows = self.db.execute(
            "SELECT ts_utc FROM events WHERE kind='watered' AND pot=? AND synced=1 "
            "AND ml > 0 AND reason NOT LIKE 'manual:%' ORDER BY id DESC LIMIT 50", (pot,)).fetchall()
        now = datetime.now(timezone.utc)
        ages = [(now - datetime.fromisoformat(r["ts_utc"])).total_seconds() for r in rows]
        return [a for a in ages if 0 <= a <= within_s]

    # manual requests (CLI today, web page or another node later)
    def add_request(self, pot: int, ml: float) -> int:
        cur = self.db.execute("INSERT INTO requests (ts_utc, pot, ml) VALUES (?,?,?)",
                              (utc_now(), pot, ml))
        self.db.commit()
        return cur.lastrowid

    def pending_requests(self) -> list[sqlite3.Row]:
        return self.db.execute(
            "SELECT * FROM requests WHERE status='pending' ORDER BY id").fetchall()

    def finish_request(self, rid: int, status: str, note: str = "") -> None:
        self.db.execute("UPDATE requests SET status=?, note=? WHERE id=?", (status, note, rid))
        self.db.commit()

    # reading back
    def last_samples(self, n: int = 5):
        rows = self.db.execute("SELECT * FROM samples ORDER BY id DESC LIMIT ?", (n,)).fetchall()
        out = []
        for r in rows:
            soil = self.db.execute("SELECT * FROM soil WHERE sample_id=? ORDER BY pot",
                                   (r["id"],)).fetchall()
            out.append((r, soil))
        return out

    def last_events(self, n: int = 10):
        return self.db.execute("SELECT * FROM events ORDER BY id DESC LIMIT ?", (n,)).fetchall()

    def close(self) -> None:
        self.db.close()
