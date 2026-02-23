import copy
import time
from collections import deque
from threading import Lock
from typing import Any, Deque, Dict, List, Optional

from .parser import ParsedEvent


class TelnetUiStateStore:
    def __init__(self, recent_logs_limit: int = 500) -> None:
        self._lock = Lock()
        self._recent_logs: Deque[Dict[str, Any]] = deque(maxlen=recent_logs_limit)
        self._state: Dict[str, Any] = {
            "connection": {
                "connected": False,
                "host": None,
                "port": None,
                "reason": "idle",
                "updated_at": time.time(),
            },
            "net": {},
            "steer": {},
            "pid": {},
            "spid": {},
            "speed": {},
            "comms": {},
            "drive_log": {},
            "streams": {
                "speed": {"state": "OFF", "period_ms": None, "updated_at": None},
                "spid": {"state": "OFF", "period_ms": None, "updated_at": None},
            },
            "recent_logs": [],
        }

    def set_connection(self,
                       connected: bool,
                       host: Optional[str] = None,
                       port: Optional[int] = None,
                       reason: Optional[str] = None) -> Dict[str, Any]:
        with self._lock:
            current = self._state["connection"]
            current["connected"] = connected
            current["host"] = host if host is not None else current.get("host")
            current["port"] = port if port is not None else current.get("port")
            current["reason"] = reason or ("connected" if connected else "disconnected")
            current["updated_at"] = time.time()
            return {"connection": copy.deepcopy(current)}

    def apply_event(self, event: ParsedEvent) -> Dict[str, Any]:
        patch: Dict[str, Any] = {}
        now = time.time()

        with self._lock:
            log_entry = {
                "ts": now,
                "line": event.line,
                "tags": event.tags,
                "section": event.section,
            }
            self._recent_logs.append(log_entry)
            self._state["recent_logs"] = list(self._recent_logs)
            patch["recent_logs"] = self._state["recent_logs"]

            section_payload = {
                "raw": event.line,
                "tags": event.tags,
                "fields": event.fields,
                "message": event.message,
                "updated_at": now,
            }

            if event.section in ("net", "steer", "pid", "spid", "speed", "comms", "drive_log"):
                self._state[event.section] = section_payload
                patch[event.section] = copy.deepcopy(section_payload)

                if event.section == "drive_log":
                    if "state" in event.fields:
                        self._state["drive_log"]["enabled"] = (event.fields["state"] == "ON")
                    elif "ON" in event.line:
                        self._state["drive_log"]["enabled"] = True
                    elif "OFF" in event.line:
                        self._state["drive_log"]["enabled"] = False
                    patch["drive_log"] = copy.deepcopy(self._state["drive_log"])

            elif event.section == "streams":
                stream_key = event.fields.get("stream")
                if stream_key in self._state["streams"]:
                    current_stream = self._state["streams"][stream_key]
                    current_stream["raw"] = event.line
                    current_stream["tags"] = event.tags
                    current_stream["updated_at"] = now
                    current_stream["state"] = event.fields.get("state", current_stream.get("state", "OFF"))
                    if "periodo" in event.fields:
                        period_text = event.fields["periodo"].replace("ms", "")
                        try:
                            current_stream["period_ms"] = int(period_text)
                        except ValueError:
                            current_stream["period_ms"] = event.fields["periodo"]
                    patch["streams"] = copy.deepcopy(self._state["streams"])

            # Keep comms/log visibility for lines that are tagged but not fully parsed.
            elif event.section in ("logs", "unknown"):
                self._state[event.section] = section_payload
                patch[event.section] = copy.deepcopy(section_payload)

        return patch

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            data = copy.deepcopy(self._state)
            data["recent_logs"] = list(self._recent_logs)
            return data

    def clear_recent_logs(self) -> Dict[str, Any]:
        with self._lock:
            self._recent_logs.clear()
            self._state["recent_logs"] = []
            return {"recent_logs": []}

    def latest_lines(self, limit: int = 50) -> List[Dict[str, Any]]:
        with self._lock:
            return list(self._recent_logs)[-limit:]
