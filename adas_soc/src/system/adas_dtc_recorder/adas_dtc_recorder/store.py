"""Hardware-independent bounded DTC persistence."""

import json
import os
import pathlib
import tempfile


class DtcStore:
    SCHEMA_VERSION = 1

    def __init__(self, path, catalog, max_records=128):
        self.path = pathlib.Path(path)
        self.catalog = {item["code"]: item for item in catalog}
        self.max_records = max_records
        self.records = {}
        self.dirty = False
        self._load()

    def _load(self):
        if not self.path.exists():
            return
        try:
            payload = json.loads(self.path.read_text(encoding="utf-8"))
            if payload.get("schema_version") != self.SCHEMA_VERSION:
                return
            for record in payload.get("records", []):
                if record.get("code") in self.catalog:
                    self.records[record["code"]] = record
        except (OSError, ValueError, TypeError):
            # A corrupt history is not allowed to prevent control-stack startup.
            self.records = {}

    def observe(self, code, timestamp, freeze_frame):
        definition = self.catalog[code]
        record = self.records.get(code)
        if record is None:
            record = {
                "code": code,
                "name": definition["name"],
                "source": definition["source"],
                "severity": definition["severity"],
                "safety_action": definition["safety_action"],
                "latched": bool(definition["latched"]),
                "occurrences": 0,
                "first_seen_unix": timestamp,
                "last_seen_unix": timestamp,
                "active": False,
                "active_since_unix": None,
                "total_active_s": 0.0,
                "first_freeze_frame": freeze_frame,
                "last_freeze_frame": freeze_frame,
            }
            self.records[code] = record
        if not record["active"]:
            record["occurrences"] += 1
            record["active"] = True
            record["active_since_unix"] = timestamp
            record["first_seen_unix"] = min(record["first_seen_unix"], timestamp)
        record["last_seen_unix"] = timestamp
        record["last_freeze_frame"] = freeze_frame
        self.dirty = True

    def clear(self, code, timestamp):
        record = self.records.get(code)
        if record is None or not record["active"] or record["latched"]:
            return
        started = record.get("active_since_unix")
        if started is not None:
            record["total_active_s"] += max(0.0, timestamp - started)
        record["active"] = False
        record["active_since_unix"] = None
        record["last_seen_unix"] = timestamp
        self.dirty = True

    def snapshot(self):
        ordered = sorted(self.records.values(), key=lambda item: item["last_seen_unix"],
                         reverse=True)[:self.max_records]
        return {"schema_version": self.SCHEMA_VERSION, "records": ordered}

    def flush(self):
        if not self.dirty:
            return
        self.path.parent.mkdir(parents=True, exist_ok=True)
        payload = json.dumps(self.snapshot(), ensure_ascii=False, indent=2) + "\n"
        fd, temporary = tempfile.mkstemp(prefix=self.path.name + ".", dir=self.path.parent)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as stream:
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.path)
            self.dirty = False
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
