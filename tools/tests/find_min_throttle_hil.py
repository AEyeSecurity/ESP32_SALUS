#!/usr/bin/env python3
"""Find minimum effective throttle command using speed PID + Hall feedback."""

from __future__ import annotations

import argparse
import json
import os
import re
import telnetlib
import time
from datetime import datetime, timezone
from typing import Dict, List, Optional, Tuple

RE_SPLIT = re.compile(r"(?<!\n)(?<!\])(?=\[[A-Z])")
RE_DRIVE_TOKENS = re.compile(r"([A-Za-z]+)=([^\s]+)")


def now_iso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def to_float(v: Optional[str]) -> Optional[float]:
    if v is None:
        return None
    s = v.strip().rstrip(",}%])")
    for suffix in ("m/s2", "m/s", "%", "km/h", "deg", "ms"):
        if s.endswith(suffix):
            s = s[: -len(suffix)]
            break
    s = s.strip()
    if not s:
        return None
    try:
        return float(s)
    except ValueError:
        return None


def to_int(v: Optional[str]) -> Optional[int]:
    if v is None:
        return None
    s = v.strip().rstrip(",")
    if "/" in s:
        s = s.split("/", 1)[0]
    try:
        return int(s)
    except ValueError:
        f = to_float(s)
        return int(round(f)) if f is not None else None


class TelnetClient:
    def __init__(self, host: str, port: int, timeout: float):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.tn: Optional[telnetlib.Telnet] = None

    def connect(self) -> List[Tuple[float, str]]:
        self.tn = telnetlib.Telnet(self.host, self.port, timeout=self.timeout)
        time.sleep(0.7)
        return self.read_events()

    def read_events(self) -> List[Tuple[float, str]]:
        if self.tn is None:
            return []
        try:
            data = self.tn.read_very_eager()
        except EOFError:
            return []
        if not data:
            return []
        text = RE_SPLIT.sub("\n", data.decode("utf-8", errors="ignore"))
        ts = time.monotonic()
        out: List[Tuple[float, str]] = []
        for ln in text.splitlines():
            line = ln.strip()
            if line:
                out.append((ts, line))
        return out

    def send(self, cmd: str, wait_s: float = 0.30) -> List[Tuple[float, str]]:
        if self.tn is None:
            raise RuntimeError("telnet not connected")
        self.tn.write((cmd + "\n").encode("utf-8"))
        time.sleep(wait_s)
        return self.read_events()

    def collect(self, duration_s: float, poll_s: float = 0.08) -> List[Tuple[float, str]]:
        out: List[Tuple[float, str]] = []
        end = time.monotonic() + duration_s
        while time.monotonic() < end:
            out.extend(self.read_events())
            time.sleep(poll_s)
        out.extend(self.read_events())
        return out

    def close(self) -> None:
        if self.tn is None:
            return
        try:
            self.tn.close()
        except Exception:
            pass
        self.tn = None


def choose_last(events: List[Tuple[float, str]], prefix: str) -> Optional[str]:
    for _, line in reversed(events):
        if line.startswith(prefix):
            return line
    return None


def parse_spid_status(line: str) -> Dict[str, float]:
    out: Dict[str, float] = {}
    for key in ("kp", "ki", "kd", "max", "ramp", "cap", "hys"):
        m = re.search(r"\b" + key + r"=([-+]?\d+(?:\.\d+)?)", line)
        if m:
            out[key] = float(m.group(1))
    return out


def parse_drive(line: str) -> Dict[str, object]:
    data: Dict[str, str] = {}
    for k, v in RE_DRIVE_TOKENS.findall(line):
        data[k] = v
    parsed = {
        "cmd": to_int(data.get("cmd")),
        "duty": to_int(data.get("duty")),
        "src": data.get("src"),
        "speed_mps": to_float(data.get("speed")),
        "target_mps": to_float(data.get("target")),
        "pid_out_pct": to_float(data.get("pidOut")),
        "fb": data.get("fb"),
        "fs": data.get("fs"),
        "mode": data.get("mode"),
        "raw_line": line,
    }
    if parsed["speed_mps"] is None:
        parsed["speed_mps"] = to_float(data.get("speedMps"))
    if parsed["target_mps"] is None:
        parsed["target_mps"] = to_float(data.get("targetMps"))
    if parsed["pid_out_pct"] is None:
        parsed["pid_out_pct"] = to_float(data.get("pidOutPct"))
    return parsed


def write_md(path: str, report: Dict[str, object]) -> None:
    lines = [
        "# Min Throttle HIL",
        "",
        f"- Start (UTC): {report['started_at_utc']}",
        f"- End (UTC): {report['finished_at_utc']}",
        f"- Target: `{report['host']}:{report['port']}`",
        "",
        "## Resultado",
        "",
        f"- movement_detected: **{report['movement_detected']}**",
        f"- threshold_mps: {report['movement_threshold_mps']}",
        f"- first_motion_time_s: {report['first_motion_time_s']}",
        f"- min_effective_cmd: {report['min_effective_cmd']}",
        f"- min_effective_pidOut_pct: {report['min_effective_pidOut_pct']}",
        f"- max_speed_drive_mps: {report['max_speed_drive_mps']}",
        f"- max_speed_status_mps: {report['max_speed_status_mps']}",
        f"- failsafe_seen: {report['failsafe_seen']}",
        "",
    ]
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Find minimum effective throttle for wheel motion")
    p.add_argument("--host", default="esp32-salus.local")
    p.add_argument("--port", type=int, default=23)
    p.add_argument("--timeout", type=float, default=5.0)
    p.add_argument("--kp-test", type=float, default=80.0)
    p.add_argument("--ki-test", type=float, default=0.0)
    p.add_argument("--kd-test", type=float, default=0.0)
    p.add_argument("--ramp-test", type=float, default=30.0)
    p.add_argument("--run-seconds", type=float, default=8.0)
    p.add_argument("--movement-threshold", type=float, default=0.15)
    p.add_argument("--out-json", default="artifacts/min_throttle_hil_report.json")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    os.makedirs(os.path.dirname(args.out_json), exist_ok=True)
    out_md = os.path.splitext(args.out_json)[0] + ".md"

    report: Dict[str, object] = {
        "started_at_utc": now_iso(),
        "host": args.host,
        "port": args.port,
        "movement_threshold_mps": args.movement_threshold,
    }

    c = TelnetClient(args.host, args.port, args.timeout)
    events: List[Tuple[float, str]] = []
    spid_orig: Dict[str, float] = {}

    try:
        events.extend(c.connect())

        for cmd in ("comms.status", "spid.status", "speed.status"):
            events.extend(c.send(cmd, wait_s=0.35))

        spid_line = choose_last(events, "[SPID] state{")
        if not spid_line:
            raise RuntimeError("No se pudo leer spid.status")
        spid_orig = parse_spid_status(spid_line)
        max_speed = spid_orig.get("max", 4.17)

        # Setup test profile.
        for cmd in (
            "spid.target off",
            "spid.stream on 100",
            "drive.log on",
            "speed.reset",
            f"spid.set {args.kp_test:.3f} {args.ki_test:.3f} {args.kd_test:.3f}",
            f"spid.ramp {args.ramp_test:.3f}",
            f"spid.target {max_speed:.2f}",
        ):
            events.extend(c.send(cmd, wait_s=0.30))

        run_start = time.monotonic()
        next_speed_poll = run_start + 0.5
        while time.monotonic() - run_start < args.run_seconds:
            events.extend(c.collect(0.12, poll_s=0.05))
            if time.monotonic() >= next_speed_poll:
                events.extend(c.send("speed.status", wait_s=0.05))
                next_speed_poll = time.monotonic() + 0.5

        # Ramp down and cleanup commands.
        for cmd in ("spid.target 0.00", "spid.target off"):
            events.extend(c.send(cmd, wait_s=0.35))

        # Parse test data.
        drive_events: List[Tuple[float, Dict[str, object]]] = []
        max_speed_drive = 0.0
        max_speed_status = 0.0
        failsafe_seen = False

        for ts, line in events:
            if line.startswith("[DRIVE]"):
                parsed = parse_drive(line)
                drive_events.append((ts, parsed))
                s = parsed.get("speed_mps")
                if isinstance(s, float):
                    max_speed_drive = max(max_speed_drive, s)
                if parsed.get("fs") == "Y":
                    failsafe_seen = True
            elif line.startswith("[SPD][STATUS]"):
                m = re.search(r"\s([0-9.]+)m/s", line)
                if m:
                    max_speed_status = max(max_speed_status, float(m.group(1)))

        first_motion = None
        for ts, d in drive_events:
            s = d.get("speed_mps")
            src = d.get("src")
            if isinstance(s, float) and s >= args.movement_threshold and src in ("TEL", "RC", "PI"):
                first_motion = (ts, d)
                break

        movement_detected = first_motion is not None
        first_motion_time = None
        min_cmd = None
        min_pid = None
        if first_motion is not None:
            first_motion_time = round(first_motion[0] - run_start, 3)
            d = first_motion[1]
            min_cmd = d.get("cmd")
            min_pid = d.get("pid_out_pct")

        report.update(
            {
                "movement_detected": movement_detected,
                "first_motion_time_s": first_motion_time,
                "min_effective_cmd": min_cmd,
                "min_effective_pidOut_pct": min_pid,
                "max_speed_drive_mps": round(max_speed_drive, 3),
                "max_speed_status_mps": round(max_speed_status, 3),
                "failsafe_seen": failsafe_seen,
                "samples_drive": len(drive_events),
            }
        )

    except Exception as exc:
        report["error"] = str(exc)
    finally:
        # Always restore runtime config.
        try:
            c.send("spid.target 0.00", wait_s=0.2)
            c.send("spid.target off", wait_s=0.2)
            if spid_orig:
                c.send(
                    f"spid.set {spid_orig.get('kp', 10.0):.3f} {spid_orig.get('ki', 2.0):.3f} {spid_orig.get('kd', 0.0):.3f}",
                    wait_s=0.25,
                )
                c.send(f"spid.ramp {spid_orig.get('ramp', 2.0):.3f}", wait_s=0.25)
                c.send(f"spid.max {spid_orig.get('max', 4.17):.3f}", wait_s=0.25)
                c.send(f"spid.brakecap {spid_orig.get('cap', 30.0):.3f}", wait_s=0.25)
                c.send(f"spid.hys {spid_orig.get('hys', 0.3):.3f}", wait_s=0.25)
            c.send("spid.stream off", wait_s=0.2)
            c.send("drive.log off", wait_s=0.2)
            c.send("speed.status", wait_s=0.2)
            c.send("spid.status", wait_s=0.2)
        except Exception:
            pass
        c.close()

    report["finished_at_utc"] = now_iso()
    with open(args.out_json, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
    write_md(out_md, report)

    print(f"Report JSON: {args.out_json}")
    print(f"Report MD:   {out_md}")
    print(f"Movement detected: {report.get('movement_detected')}")

    if report.get("error"):
        return 1
    return 0 if report.get("movement_detected") else 2


if __name__ == "__main__":
    raise SystemExit(main())
