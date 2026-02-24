#!/usr/bin/env python3
"""Hall + speed PID HIL health check via Telnet (diagnostic-first).

Focuses on Hall feedback stability for speed PID before running autotune.
Generates structured reports and correlation evidence with [DRIVE][PIDTRACE].
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import re
import subprocess
import sys
import telnetlib
import time
from dataclasses import dataclass, asdict
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional, Tuple

RE_SPLIT = re.compile(r"(?<!\n)(?<!\])(?=\[[A-Z])")
RE_SPD = re.compile(
    r"driver=(?P<driver>[A-Z_]+).*?speed=(?P<kmh>[0-9.]+)km/h\s+(?P<mps>[0-9.]+)m/s"
    r".*?ageUs=(?P<ageUs>\d+).*?ok=(?P<ok>\d+).*?invState=(?P<invState>\d+).*?invJump=(?P<invJump>\d+).*?isr=(?P<isr>\d+)"
)
RE_SPID = re.compile(
    r"state\{init=(?P<init>[YN]) en=(?P<en>[YN]) fb=(?P<fb>[YN]) failsafe=(?P<fs>[YN]) overspeed=(?P<ovs>[YN]) mode=(?P<mode>[A-Z]+)\}"
)
RE_TOKENS = re.compile(r"([A-Za-z_]+)=([^\s]+)")
RE_COMMS_LAST_FRAME = re.compile(r"lastFrame=(NONE|\d+ms)")
RE_PING_PKT = re.compile(r"(?P<tx>\d+) packets transmitted, (?P<rx>\d+) received")
RE_PING_RTT = re.compile(r"rtt min/avg/max/mdev = ([0-9.]+)/([0-9.]+)/([0-9.]+)/([0-9.]+) ms")


@dataclass
class SpeedSnap:
    t: float
    driver: str
    kmh: float
    mps: float
    age_us: int
    ok: int
    inv_state: int
    inv_jump: int
    isr: int
    raw_line: str


@dataclass
class SpidSnap:
    t: float
    fb: bool
    failsafe: bool
    overspeed: bool
    mode: str
    target: float
    speed: float
    throttle: float
    brake: float
    raw_line: str


@dataclass
class PidTraceSnap:
    t: float
    src: str
    mode: str
    target_mps: float
    speed_mps: float
    pid_out_pct: float
    throttle_filt_pct: float
    throttle_raw_pct: float
    pwm_duty_pct: float
    auto_brake_filt_pct: float
    fb: bool
    fs: bool
    ovs: bool
    inhibit: str
    raw_line: str


class TelnetClient:
    def __init__(self, host: str, port: int, timeout: float = 5.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.tn: Optional[telnetlib.Telnet] = None

    def connect(self) -> List[Tuple[float, str]]:
        self.tn = telnetlib.Telnet(self.host, self.port, timeout=self.timeout)
        time.sleep(0.7)
        return self.read_lines()

    def close(self) -> None:
        if self.tn is None:
            return
        try:
            for cmd in (b"spid.target off\n", b"spid.stream off\n", b"drive.log pid off\n", b"drive.log off\n"):
                self.tn.write(cmd)
                time.sleep(0.08)
        except Exception:
            pass
        try:
            self.tn.close()
        except Exception:
            pass
        self.tn = None

    def read_lines(self) -> List[Tuple[float, str]]:
        if self.tn is None:
            return []
        try:
            data = self.tn.read_very_eager()
        except EOFError:
            return []
        if not data:
            return []
        raw = RE_SPLIT.sub("\n", data.decode("utf-8", errors="ignore"))
        ts = time.monotonic()
        out: List[Tuple[float, str]] = []
        for ln in raw.splitlines():
            line = ln.strip()
            if line:
                out.append((ts, line))
        return out

    def send(self, cmd: str, wait: float = 0.3) -> List[Tuple[float, str]]:
        if self.tn is None:
            raise RuntimeError("not connected")
        self.tn.write((cmd + "\n").encode("utf-8"))
        time.sleep(wait)
        return self.read_lines()

    def collect(self, secs: float, poll: float = 0.07) -> List[Tuple[float, str]]:
        out: List[Tuple[float, str]] = []
        end = time.monotonic() + secs
        while time.monotonic() < end:
            out.extend(self.read_lines())
            time.sleep(poll)
        out.extend(self.read_lines())
        return out


def log_progress(enabled: bool, msg: str) -> None:
    if not enabled:
        return
    ts = time.strftime("%H:%M:%S", time.localtime())
    print(f"[HALLHIL][{ts}] {msg}", flush=True)


def now_iso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def to_bool_yn(v: Optional[str]) -> Optional[bool]:
    if v == "Y":
        return True
    if v == "N":
        return False
    return None


def to_float(v: Optional[str]) -> float:
    if v is None:
        return 0.0
    s = v.strip().rstrip(",}%])")
    for suffix in ("m/s2", "m/s", "%", "km/h", "ms"):
        if s.endswith(suffix):
            s = s[: -len(suffix)]
            break
    try:
        return float(s)
    except ValueError:
        return 0.0


def parse_speed(line_t: Tuple[float, str]) -> Optional[SpeedSnap]:
    t, line = line_t
    if "[SPD][STATUS]" not in line:
        return None
    m = RE_SPD.search(line)
    if not m:
        return None
    return SpeedSnap(
        t=t,
        driver=m.group("driver"),
        kmh=float(m.group("kmh")),
        mps=float(m.group("mps")),
        age_us=int(m.group("ageUs")),
        ok=int(m.group("ok")),
        inv_state=int(m.group("invState")),
        inv_jump=int(m.group("invJump")),
        isr=int(m.group("isr")),
        raw_line=line,
    )


def parse_spid(line_t: Tuple[float, str]) -> Optional[SpidSnap]:
    t, line = line_t
    if not line.startswith("[SPID] state{"):
        return None
    m = RE_SPID.search(line)
    if not m:
        return None

    def f(key: str) -> float:
        mm = re.search(r"\b" + key + r"=([^\s]+)", line)
        return to_float(mm.group(1) if mm else None)

    return SpidSnap(
        t=t,
        fb=(m.group("fb") == "Y"),
        failsafe=(m.group("fs") == "Y"),
        overspeed=(m.group("ovs") == "Y"),
        mode=m.group("mode"),
        target=f("target"),
        speed=f("speed"),
        throttle=f("throttle"),
        brake=f("brake"),
        raw_line=line,
    )


def parse_pidtrace(line_t: Tuple[float, str]) -> Optional[PidTraceSnap]:
    t, line = line_t
    if not line.startswith("[DRIVE][PIDTRACE]"):
        return None
    tokens = {k: v for k, v in RE_TOKENS.findall(line)}
    return PidTraceSnap(
        t=t,
        src=tokens.get("src", "UNK"),
        mode=tokens.get("mode", "UNKNOWN"),
        target_mps=to_float(tokens.get("targetMps")),
        speed_mps=to_float(tokens.get("speedMps")),
        pid_out_pct=to_float(tokens.get("pidOutPct")),
        throttle_filt_pct=to_float(tokens.get("throttleFiltPct")),
        throttle_raw_pct=to_float(tokens.get("throttleRawPct")),
        pwm_duty_pct=to_float(tokens.get("pwmDutyPct")),
        auto_brake_filt_pct=to_float(tokens.get("autoBrakeFiltPct")),
        fb=(tokens.get("fb") == "Y"),
        fs=(tokens.get("fs") == "Y"),
        ovs=(tokens.get("ovs") == "Y"),
        inhibit=tokens.get("inhibit", "NONE"),
        raw_line=line,
    )


def parse_comms_status(line: str) -> Dict[str, Any]:
    out: Dict[str, Any] = {"raw_line": line}
    m = RE_COMMS_LAST_FRAME.search(line)
    if m:
        lf = m.group(1)
        out["lastFrame"] = lf
        if lf == "NONE":
            out["piFresh"] = False
            out["lastFrameMs"] = None
        else:
            ms = int(lf[:-2])
            out["lastFrameMs"] = ms
            out["piFresh"] = ms <= 120
    mm = re.search(r"\bestop=([YN])", line)
    if mm:
        out["estop"] = (mm.group(1) == "Y")
    return out


def count_true_events(samples: List[Any], attr: str) -> int:
    prev = False
    n = 0
    for s in samples:
        cur = bool(getattr(s, attr))
        if cur and not prev:
            n += 1
        prev = cur
    return n


def count_true_duration(samples: List[Any], attr: str) -> float:
    if len(samples) < 2:
        return 0.0
    total = 0.0
    for i, s in enumerate(samples[:-1]):
        dt = max(0.0, samples[i + 1].t - s.t)
        if bool(getattr(s, attr)):
            total += dt
    return total


def nearest_pidtrace(pidtraces: List[PidTraceSnap], t: float, max_dt: float = 0.35) -> Optional[PidTraceSnap]:
    best: Optional[PidTraceSnap] = None
    best_dt = max_dt
    for p in pidtraces:
        d = abs(p.t - t)
        if d <= best_dt:
            best_dt = d
            best = p
    return best


def nearest_speed(speeds: List[SpeedSnap], t: float, max_dt: float = 0.35) -> Optional[SpeedSnap]:
    best: Optional[SpeedSnap] = None
    best_dt = max_dt
    for s in speeds:
        d = abs(s.t - t)
        if d <= best_dt:
            best_dt = d
            best = s
    return best


def ping_check(host: str, count: int = 4, timeout_s: int = 6) -> Dict[str, Any]:
    out: Dict[str, Any] = {"host": host, "count": count}
    try:
        proc = subprocess.run(["ping", "-c", str(count), host], capture_output=True, text=True, timeout=timeout_s)
        txt = proc.stdout + proc.stderr
        out["exit_code"] = proc.returncode
        out["raw_tail"] = txt.strip().splitlines()[-6:]
        pkt = RE_PING_PKT.search(txt)
        if pkt:
            tx, rx = int(pkt.group("tx")), int(pkt.group("rx"))
            out["tx"] = tx
            out["rx"] = rx
            out["loss_pct"] = round((1.0 - (rx / tx)) * 100.0, 2) if tx else None
        rtt = RE_PING_RTT.search(txt)
        if rtt:
            out["rtt_min_ms"] = float(rtt.group(1))
            out["rtt_avg_ms"] = float(rtt.group(2))
            out["rtt_max_ms"] = float(rtt.group(3))
            out["rtt_mdev_ms"] = float(rtt.group(4))
    except Exception as exc:
        out["error"] = str(exc)
    return out


def telnet_handshake(host: str, port: int, timeout: float, retries: int, progress: bool) -> Dict[str, Any]:
    attempts: List[Dict[str, Any]] = []
    for i in range(retries):
        t0 = time.monotonic()
        try:
            tn = telnetlib.Telnet(host, port, timeout=timeout)
            time.sleep(0.7)
            banner = tn.read_very_eager().decode("utf-8", errors="ignore")
            tn.write(b"net.status\n")
            time.sleep(0.4)
            net = tn.read_very_eager().decode("utf-8", errors="ignore")
            tn.close()
            dt = time.monotonic() - t0
            attempts.append({"attempt": i + 1, "ok": True, "connect_s": round(dt, 3), "banner_tail": banner.splitlines()[-4:], "net_tail": net.splitlines()[-4:]})
            log_progress(progress, f"Telnet handshake OK intento {i+1} ({dt:.2f}s)")
            return {"ok": True, "attempts": attempts}
        except Exception as exc:
            dt = time.monotonic() - t0
            attempts.append({"attempt": i + 1, "ok": False, "connect_s": round(dt, 3), "error": str(exc)})
            log_progress(progress, f"Telnet handshake FAIL intento {i+1}: {exc}")
            time.sleep(min(1.0 + i * 0.8, 3.0))
    return {"ok": False, "attempts": attempts}


def filter_speed_series(
    speeds: List[SpeedSnap],
    max_valid_speed_mps: float,
    max_speed_jump_mps_per_s: float,
) -> Tuple[List[SpeedSnap], Dict[str, Any]]:
    kept: List[SpeedSnap] = []
    stats = {
        "input_count": len(speeds),
        "outlier_count": 0,
        "outlier_ratio": 0.0,
        "outlier_reasons": {"abs": 0, "jump": 0},
    }
    prev: Optional[SpeedSnap] = None
    for s in speeds:
        drop = False
        if s.mps < -0.2 or s.mps > max_valid_speed_mps:
            stats["outlier_count"] += 1
            stats["outlier_reasons"]["abs"] += 1
            drop = True
        elif prev is not None:
            dt = max(1e-3, s.t - prev.t)
            jump = abs(s.mps - prev.mps) / dt
            if jump > max_speed_jump_mps_per_s:
                stats["outlier_count"] += 1
                stats["outlier_reasons"]["jump"] += 1
                drop = True
        if not drop:
            kept.append(s)
            prev = s
    if speeds:
        stats["outlier_ratio"] = stats["outlier_count"] / len(speeds)
    return kept, stats


def filter_spid_series(
    spids: List[SpidSnap],
    max_valid_speed_mps: float,
    max_speed_jump_mps_per_s: float,
) -> Tuple[List[SpidSnap], Dict[str, Any]]:
    kept: List[SpidSnap] = []
    stats = {
        "input_count": len(spids),
        "outlier_count": 0,
        "outlier_ratio": 0.0,
        "outlier_reasons": {"abs": 0, "jump": 0},
    }
    prev: Optional[SpidSnap] = None
    for s in spids:
        drop = False
        if s.speed < -0.2 or s.speed > max_valid_speed_mps:
            stats["outlier_count"] += 1
            stats["outlier_reasons"]["abs"] += 1
            drop = True
        elif prev is not None:
            dt = max(1e-3, s.t - prev.t)
            jump = abs(s.speed - prev.speed) / dt
            if jump > max_speed_jump_mps_per_s:
                stats["outlier_count"] += 1
                stats["outlier_reasons"]["jump"] += 1
                drop = True
        if not drop:
            kept.append(s)
            prev = s
    if spids:
        stats["outlier_ratio"] = stats["outlier_count"] / len(spids)
    return kept, stats


def ensure_parent(path: str) -> None:
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)


def parse_args(argv: List[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Hall + speed PID HIL diagnostic")
    p.add_argument("--host", default="192.168.1.120")
    p.add_argument("--port", type=int, default=23)
    p.add_argument("--timeout", type=float, default=8.0)
    p.add_argument("--out-json", default="artifacts/hall_hil_check_report.json")
    p.add_argument("--out-md", default="")
    p.add_argument("--out-csv", default="artifacts/hall_hil_check_timeseries.csv")
    p.add_argument("--out-correlation", default="artifacts/hall_pidtrace_correlation.json")
    p.add_argument("--mode", choices=["quick", "full"], default="full")
    p.add_argument("--progress-interval-s", type=float, default=2.0)
    p.add_argument("--retries", type=int, default=3)
    p.add_argument("--max-valid-speed-mps", type=float, default=8.5)
    p.add_argument("--max-speed-jump-mps2", type=float, default=45.0)
    p.add_argument("--rest-max-mps", type=float, default=0.15)
    p.add_argument("--rest-threshold-mps", type=float, default=0.12)
    p.add_argument("--require-reach-frac", type=float, default=0.60)
    p.add_argument("--failsafe-sustain-threshold-s", type=float, default=1.0)
    p.add_argument("--probe-target-mps", type=float, default=1.94)
    p.add_argument("--probe-run-s", type=float, default=8.0)
    p.add_argument("--probe-decay-s", type=float, default=6.0)
    p.add_argument("--probe-grace-s", type=float, default=3.0)
    p.add_argument("--probe-min-target-mps", type=float, default=0.5)
    p.add_argument("--probe-min-throttle-pct", type=float, default=10.0)
    p.add_argument("--probe-min-pwm-pct", type=float, default=1.0)
    p.add_argument("--probe-min-motion-mps", type=float, default=0.10)
    p.add_argument("--probe-setup-minthrottle-pct", type=float, default=45.0)
    p.add_argument("--probe-setup-thslewup", type=float, default=70.0)
    p.add_argument("--probe-setup-thslewdown", type=float, default=85.0)
    p.add_argument("--probe-setup-launchwin-ms", type=int, default=2600)
    return p.parse_args(argv)


def write_csv(path: str, rows: List[Dict[str, Any]]) -> None:
    ensure_parent(path)
    fields = [
        "ts", "phase", "segment", "kind", "speed_mps", "target_mps", "throttle_pct", "brake_pct",
        "fb", "fs", "ovs", "mode", "inhibit", "age_us", "raw",
    ]
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=fields)
        w.writeheader()
        for r in rows:
            w.writerow(r)


def append_rows(rows: List[Dict[str, Any]], phase: str, segment: str,
                speeds: List[SpeedSnap], spids: List[SpidSnap], pids: List[PidTraceSnap]) -> None:
    for s in speeds:
        rows.append({
            "ts": f"{s.t:.6f}", "phase": phase, "segment": segment, "kind": "speed",
            "speed_mps": f"{s.mps:.3f}", "target_mps": "", "throttle_pct": "", "brake_pct": "",
            "fb": "", "fs": "", "ovs": "", "mode": "", "inhibit": "", "age_us": s.age_us,
            "raw": s.raw_line,
        })
    for s in spids:
        rows.append({
            "ts": f"{s.t:.6f}", "phase": phase, "segment": segment, "kind": "spid",
            "speed_mps": f"{s.speed:.3f}", "target_mps": f"{s.target:.3f}", "throttle_pct": f"{s.throttle:.2f}",
            "brake_pct": f"{s.brake:.2f}", "fb": int(s.fb), "fs": int(s.failsafe), "ovs": int(s.overspeed),
            "mode": s.mode, "inhibit": "", "age_us": "", "raw": s.raw_line,
        })
    for p in pids:
        rows.append({
            "ts": f"{p.t:.6f}", "phase": phase, "segment": segment, "kind": "pidtrace",
            "speed_mps": f"{p.speed_mps:.3f}", "target_mps": f"{p.target_mps:.3f}", "throttle_pct": f"{p.throttle_filt_pct:.2f}",
            "brake_pct": f"{p.auto_brake_filt_pct:.2f}", "fb": int(p.fb), "fs": int(p.fs), "ovs": int(p.ovs),
            "mode": p.mode, "inhibit": p.inhibit, "age_us": "", "raw": p.raw_line,
        })


def summarize_actuation_probe(
    target_cmd_mps: float,
    spids: List[SpidSnap],
    speeds: List[SpeedSnap],
    pidtraces: List[PidTraceSnap],
    grace_s: float,
    min_target_mps: float,
    min_throttle_pct: float,
    min_pwm_pct: float,
    min_motion_mps: float,
) -> Dict[str, Any]:
    t0_candidates = [xs[0].t for xs in (pidtraces, spids, speeds) if xs]
    t0 = min(t0_candidates) if t0_candidates else time.monotonic()

    pid_after = [p for p in pidtraces if (p.t - t0) >= grace_s]
    spid_after = [s for s in spids if (s.t - t0) >= grace_s]
    speed_after = [s for s in speeds if (s.t - t0) >= grace_s]
    pid_eval = pid_after or pidtraces
    spid_eval = spid_after or spids
    speed_eval = speed_after or speeds

    max_pid_target = max((p.target_mps for p in pid_eval), default=0.0)
    max_pid_out = max((p.pid_out_pct for p in pid_eval), default=0.0)
    max_thr_f = max((p.throttle_filt_pct for p in pid_eval), default=0.0)
    max_thr_r = max((p.throttle_raw_pct for p in pid_eval), default=0.0)
    max_pwm = max((p.pwm_duty_pct for p in pid_eval), default=0.0)
    max_spid_thr = max((s.throttle for s in spid_eval), default=0.0)
    max_spid_speed = max((s.speed for s in spid_eval), default=0.0)
    max_speed_status = max((s.mps for s in speed_eval), default=0.0)

    target_seen = bool(max_pid_target >= min_target_mps or any(s.target >= min_target_mps for s in spid_eval))
    throttle_command_seen = bool(
        max_pid_out >= min_throttle_pct
        or max_thr_f >= min_throttle_pct
        or max_thr_r >= min_throttle_pct
        or max_spid_thr >= min_throttle_pct
        or max_pwm >= min_pwm_pct
    )
    motion_seen = bool(max_spid_speed >= min_motion_mps or max_speed_status >= min_motion_mps)

    inhibit_counts: Dict[str, int] = {}
    for p in pid_eval:
        inhibit_counts[p.inhibit] = inhibit_counts.get(p.inhibit, 0) + 1
    dominant_inhibit = max(inhibit_counts, key=inhibit_counts.get) if inhibit_counts else "NONE"

    first_pid = pid_eval[0] if pid_eval else None
    first_spid = spid_eval[0] if spid_eval else None
    first_speed = speed_eval[0] if speed_eval else None

    return {
        "target_cmd_mps": round(target_cmd_mps, 3),
        "grace_s": round(grace_s, 3),
        "samples_pidtrace": len(pidtraces),
        "samples_spid": len(spids),
        "samples_speed": len(speeds),
        "target_seen": target_seen,
        "throttle_command_seen": throttle_command_seen,
        "motion_seen": motion_seen,
        "no_actuation_command": bool(target_seen and not throttle_command_seen),
        "pass": bool(target_seen and throttle_command_seen),
        "max_pid_target_mps": round(max_pid_target, 3),
        "max_pid_out_pct": round(max_pid_out, 2),
        "max_throttle_filt_pct": round(max_thr_f, 2),
        "max_throttle_raw_pct": round(max_thr_r, 2),
        "max_pwm_duty_pct": round(max_pwm, 2),
        "max_spid_throttle_pct": round(max_spid_thr, 2),
        "max_spid_speed_mps": round(max_spid_speed, 3),
        "max_speed_status_mps": round(max_speed_status, 3),
        "dominant_inhibit_after_grace": dominant_inhibit,
        "first_pidtrace_after_grace": None if first_pid is None else {
            "src": first_pid.src,
            "mode": first_pid.mode,
            "target_mps": round(first_pid.target_mps, 3),
            "speed_mps": round(first_pid.speed_mps, 3),
            "pid_out_pct": round(first_pid.pid_out_pct, 2),
            "throttle_filt_pct": round(first_pid.throttle_filt_pct, 2),
            "pwm_duty_pct": round(first_pid.pwm_duty_pct, 2),
            "inhibit": first_pid.inhibit,
            "fb": first_pid.fb,
            "fs": first_pid.fs,
        },
        "first_spid_after_grace": None if first_spid is None else {
            "target_mps": round(first_spid.target, 3),
            "speed_mps": round(first_spid.speed, 3),
            "throttle_pct": round(first_spid.throttle, 2),
            "mode": first_spid.mode,
            "fb": first_spid.fb,
            "failsafe": first_spid.failsafe,
        },
        "first_speed_after_grace": None if first_speed is None else {
            "mps": round(first_speed.mps, 3),
            "ok": first_speed.ok,
            "inv_state": first_speed.inv_state,
            "age_us": first_speed.age_us,
        },
    }


def summarize_step(
    target: float,
    spids: List[SpidSnap],
    speeds: List[SpeedSnap],
    pidtraces: List[PidTraceSnap],
    require_reach_frac: float,
    rest_threshold_mps: float,
    failsafe_sustain_threshold_s: float,
    decay_duration_s: float,
) -> Dict[str, Any]:
    max_spid_speed = max((s.speed for s in spids), default=0.0)
    max_speed_status = max((s.mps for s in speeds), default=0.0)
    fb_lost = 0.0 if not spids else sum(max(0.0, spids[i+1].t - spids[i].t) for i in range(len(spids)-1) if not spids[i].fb)
    fs_events = count_true_events(spids, "failsafe")
    ovs_events = count_true_events(spids, "overspeed")
    fs_duration = count_true_duration(spids, "failsafe")
    fb_false_ratio = (fb_lost / max((spids[-1].t - spids[0].t), 1e-6)) if len(spids) >= 2 else 0.0

    first_transition_time = None
    for s in speeds:
        if s.mps > 0.05:
            first_transition_time = max(0.0, s.t - speeds[0].t) if speeds else None
            break

    reached_rest = False
    rest_time = None
    if speeds:
        t0 = speeds[0].t
        for s in reversed(speeds):
            if s.mps <= rest_threshold_mps:
                reached_rest = True
                rest_time = max(0.0, s.t - t0)
                break

    fail_ctx = None
    prev_fs = False
    for s in spids:
        if s.failsafe and not prev_fs:
            p = nearest_pidtrace(pidtraces, s.t)
            v = nearest_speed(speeds, s.t)
            fail_ctx = {
                "t_rel_s": round(max(0.0, s.t - (spids[0].t if spids else s.t)), 3),
                "spid_speed_mps": round(s.speed, 3),
                "spid_target_mps": round(s.target, 3),
                "spid_throttle_pct": round(s.throttle, 2),
                "pidtrace": None if p is None else {
                    "speed_mps": round(p.speed_mps, 3),
                    "target_mps": round(p.target_mps, 3),
                    "throttleFiltPct": round(p.throttle_filt_pct, 2),
                    "inhibit": p.inhibit,
                    "mode": p.mode,
                    "fb": p.fb,
                },
                "speed_status": None if v is None else {
                    "mps": round(v.mps, 3), "age_us": v.age_us, "ok": v.ok, "inv_state": v.inv_state,
                },
            }
            break
        prev_fs = s.failsafe

    return {
        "target_mps": target,
        "max_spid_speed_mps": round(max_spid_speed, 3),
        "max_speed_status_mps": round(max_speed_status, 3),
        "feedback_lost_s": round(fb_lost, 3),
        "fb_false_ratio": round(fb_false_ratio, 3),
        "failsafe_events": fs_events,
        "failsafe_sustain_s": round(fs_duration, 3),
        "failsafe_sustained": bool(fs_duration > failsafe_sustain_threshold_s),
        "overspeed_events": ovs_events,
        "reached_target_60pct": bool(max_spid_speed >= require_reach_frac * target),
        "first_valid_transition_s": None if first_transition_time is None else round(first_transition_time, 3),
        "reached_rest_after_zero": reached_rest,
        "rest_time_s": None if rest_time is None else round(min(rest_time, decay_duration_s), 3),
        "samples_spid": len(spids),
        "samples_speed": len(speeds),
        "samples_pidtrace": len(pidtraces),
        "failsafe_first_context": fail_ctx,
    }


def run_collection_segment(
    c: TelnetClient,
    segment_name: str,
    target: float,
    run_s: float,
    decay_s: float,
    progress_enabled: bool,
    progress_interval_s: float,
) -> List[Tuple[float, str]]:
    events: List[Tuple[float, str]] = []
    events.extend(c.send(f"spid.target {target:.2f}", wait=0.2))
    t0 = time.monotonic()
    next_speed = t0
    next_spid = t0
    last_p = t0
    log_progress(progress_enabled, f"seg={segment_name} target={target:.2f} run={run_s:.1f}s")
    while time.monotonic() - t0 < run_s:
        events.extend(c.collect(0.12, poll=0.05))
        now = time.monotonic()
        if now >= next_speed:
            events.extend(c.send("speed.status", wait=0.03))
            next_speed = now + 0.5
        if now >= next_spid:
            events.extend(c.send("spid.status", wait=0.03))
            next_spid = now + 0.5
        if now - last_p >= progress_interval_s:
            log_progress(progress_enabled, f"seg={segment_name} run t={now - t0:.1f}/{run_s:.1f}s")
            last_p = now

    events.extend(c.send("spid.target 0.00", wait=0.2))
    td = time.monotonic()
    next_speed = td
    next_spid = td
    while time.monotonic() - td < decay_s:
        events.extend(c.collect(0.12, poll=0.05))
        now = time.monotonic()
        if now >= next_speed:
            events.extend(c.send("speed.status", wait=0.03))
            next_speed = now + 0.5
        if now >= next_spid:
            events.extend(c.send("spid.status", wait=0.03))
            next_spid = now + 0.5
        if now - last_p >= progress_interval_s:
            log_progress(progress_enabled, f"seg={segment_name} decay t={now - td:.1f}/{decay_s:.1f}s")
            last_p = now
    return events


def classify_cause(report: Dict[str, Any]) -> Tuple[str, List[str]]:
    notes: List[str] = []
    net = report.get("network", {})
    pre = report.get("preflight", {})
    outliers = report.get("outliers", {})
    act = (report.get("tests", {}) or {}).get("ACTUATION_PROBE", {})
    t0 = (report.get("tests", {}) or {}).get("T0_rest_noise", {})
    t1 = (report.get("tests", {}) or {}).get("T1_start_steps", [])
    t2 = (report.get("tests", {}) or {}).get("T2_holds", [])

    if not net.get("telnet", {}).get("ok", True):
        return "NETWORK_TELNET_INSTABILITY", ["Handshake Telnet inestable o fallido"]
    if pre.get("precheck_code") and pre.get("precheck_code") != "OK":
        notes.append(f"Precheck fallo: {pre.get('precheck_code')}")

    out_ratio = max(
        (outliers.get("speed_status") or {}).get("outlier_ratio", 0.0),
        (outliers.get("spid") or {}).get("outlier_ratio", 0.0),
    )
    if out_ratio >= 0.10:
        return "HALL_OUTLIER_SPIKES", [f"Outlier ratio alto ({out_ratio:.2f})"]

    if act:
        if act.get("no_actuation_command"):
            return "NO_ACTUATION_COMMAND", [
                "Se recibió target pero no hubo comando suficiente de throttle/PWM",
                f"inhibit dominante={act.get('dominant_inhibit_after_grace')}",
            ]
        if act.get("throttle_command_seen") and not act.get("motion_seen"):
            return "NO_MOTION_WITH_ACTUATION", [
                "Hubo comando de aceleración pero no se detectó movimiento",
                "Revisar Hall/cableado/actuación mecánica antes de seguir tuneando PID",
            ]

    if t0 and not t0.get("pass", False):
        return "HALL_NOISE_REST", ["Ruido Hall en reposo fuera de tolerancia"]

    low_speed_fails = 0
    accel_fails = 0
    for s in t1:
        if s.get("failsafe_events", 0) > 0:
            if s.get("target_mps", 0) <= 1.4:
                low_speed_fails += 1
            else:
                accel_fails += 1
    hold_fails = sum(1 for s in t2 if s.get("failsafe_events", 0) > 0 or s.get("failsafe_sustained", False))

    if hold_fails > 0 and (accel_fails > 0 or low_speed_fails > 0):
        notes.append(f"fails en holds={hold_fails}, low={low_speed_fails}, accel={accel_fails}")
        if low_speed_fails > accel_fails:
            return "HALL_FEEDBACK_LOSS_LOW_SPEED", notes
        return "HALL_FEEDBACK_LOSS_UNDER_ACCEL", notes

    if hold_fails > 0:
        return "HALL_FEEDBACK_LOSS_UNDER_ACCEL", [f"FAILSAFE en holds ({hold_fails})"]

    if (report.get("summary", {}) or {}).get("hall_diagnostic_ok"):
        return "HALL_OK_PID_ISSUE", ["Hall pasa gates; el problema restante es tuning/control"]

    return "MIXED_CAUSE", notes or ["Causa mixta/no concluyente"]


def run(args: argparse.Namespace) -> int:
    out_md = args.out_md or (os.path.splitext(args.out_json)[0] + ".md")
    ensure_parent(args.out_json)
    ensure_parent(args.out_csv)
    ensure_parent(args.out_correlation)

    report: Dict[str, Any] = {
        "started_at_utc": now_iso(),
        "host": args.host,
        "port": args.port,
        "mode": args.mode,
        "network": {},
        "preflight": {},
        "outliers": {},
        "tests": {},
        "summary": {},
        "notes": [],
    }
    csv_rows: List[Dict[str, Any]] = []
    corr_events: List[Dict[str, Any]] = []

    # Network preflight
    log_progress(True, "network preflight: ping + telnet handshake")
    report["network"]["ping"] = ping_check(args.host)
    report["network"]["telnet"] = telnet_handshake(args.host, args.port, args.timeout, args.retries, True)
    if not report["network"]["telnet"].get("ok", False):
        report["preflight"] = {"precheck_code": "PRECHECK_TELNET_FAIL"}
        report["finished_at_utc"] = now_iso()
        primary, notes = classify_cause(report)
        report["summary"] = {"hall_diagnostic_ok": False, "primary_cause": primary}
        report["notes"].extend(notes)
        with open(args.out_json, "w", encoding="utf-8") as f:
            json.dump(report, f, ensure_ascii=False, indent=2)
        with open(args.out_correlation, "w", encoding="utf-8") as f:
            json.dump({"events": corr_events}, f, ensure_ascii=False, indent=2)
        write_csv(args.out_csv, csv_rows)
        with open(out_md, "w", encoding="utf-8") as f:
            f.write("# Hall HIL Check\n\n- Precheck Telnet FAIL\n")
        print(f"Report JSON: {args.out_json}")
        print(f"Overall OK:  False")
        return 3

    c = TelnetClient(args.host, args.port, timeout=args.timeout)
    try:
        banner = c.connect()
        pre_events: List[Tuple[float, str]] = list(banner)
        for cmd in ("comms.status", "spid.status", "speed.status"):
            pre_events.extend(c.send(cmd, wait=0.35))

        comms_line = next((ln for _, ln in reversed(pre_events) if ln.startswith("[PI][STATUS]")), "")
        spid_line = next((ln for _, ln in reversed(pre_events) if ln.startswith("[SPID] state{")), "")
        speed_line = next((ln for _, ln in reversed(pre_events) if ln.startswith("[SPD][STATUS]")), "")
        comms = parse_comms_status(comms_line) if comms_line else {}
        spid_m = RE_SPID.search(spid_line) if spid_line else None
        speed_snap = parse_speed((time.monotonic(), speed_line)) if speed_line else None

        precheck_code = "OK"
        if comms.get("piFresh"):
            precheck_code = "PRECHECK_PI_FRESH"
        elif comms.get("estop"):
            precheck_code = "PRECHECK_ESTOP"
        elif not spid_m or spid_m.group("init") != "Y":
            precheck_code = "PRECHECK_SPID_NOT_INIT"
        elif speed_snap is None or speed_snap.driver != "READY":
            precheck_code = "PRECHECK_HALL_NOT_READY"

        report["preflight"] = {
            "precheck_code": precheck_code,
            "comms": comms,
            "spid_status_raw": spid_line,
            "speed_status_raw": speed_line,
        }
        if precheck_code != "OK":
            raise RuntimeError(precheck_code)

        # Runtime setup
        for cmd in (
            "spid.target off",
            "spid.stream on 100",
            "drive.log pid on 100",
            "speed.reset",
        ):
            c.send(cmd, wait=0.2)

        # P0: actuation probe before Hall diagnosis.
        # This avoids false "Hall fault" conclusions when the test never commanded acceleration.
        for cmd in (
            "spid.reset",
            "spid.max 4.17",
            "spid.ramp 2.0",
            "spid.set 6 0.8 0",
            f"spid.minthrottle {args.probe_setup_minthrottle_pct:.2f}",
            f"spid.thslewup {args.probe_setup_thslewup:.2f}",
            f"spid.thslewdown {args.probe_setup_thslewdown:.2f}",
            "spid.minth.spd 0.45",
            f"spid.launchwin {int(args.probe_setup_launchwin_ms)}",
            "spid.iunwind 0.40",
            "spid.dfilter 2.8",
            "speed.reset",
        ):
            c.send(cmd, wait=0.12)
        log_progress(True, "P0 ACTUATION_PROBE (target/throttle/PWM)")
        p0_events = run_collection_segment(
            c,
            "P0_actuation",
            args.probe_target_mps,
            run_s=args.probe_run_s,
            decay_s=args.probe_decay_s,
            progress_enabled=True,
            progress_interval_s=args.progress_interval_s,
        )
        p0_spids_raw = [s for e in p0_events if (s := parse_spid(e)) is not None]
        p0_speeds_raw = [s for e in p0_events if (s := parse_speed(e)) is not None]
        p0_pid_raw = [p for e in p0_events if (p := parse_pidtrace(e)) is not None]
        p0_spids, p0_spid_out = filter_spid_series(p0_spids_raw, args.max_valid_speed_mps, args.max_speed_jump_mps2)
        p0_speeds, p0_speed_out = filter_speed_series(p0_speeds_raw, args.max_valid_speed_mps, args.max_speed_jump_mps2)
        append_rows(csv_rows, "P0", "actuation_probe", p0_speeds, p0_spids, p0_pid_raw)
        p0 = summarize_actuation_probe(
            target_cmd_mps=args.probe_target_mps,
            spids=p0_spids,
            speeds=p0_speeds,
            pidtraces=p0_pid_raw,
            grace_s=args.probe_grace_s,
            min_target_mps=args.probe_min_target_mps,
            min_throttle_pct=args.probe_min_throttle_pct,
            min_pwm_pct=args.probe_min_pwm_pct,
            min_motion_mps=args.probe_min_motion_mps,
        )
        p0["outliers"] = {"speed_status": p0_speed_out, "spid": p0_spid_out}
        report["tests"]["ACTUATION_PROBE"] = p0
        if p0.get("no_actuation_command"):
            report["preflight"]["precheck_code"] = "PRECHECK_NO_ACTUATION_COMMAND"
            raise RuntimeError("PRECHECK_NO_ACTUATION_COMMAND")

        # T0 rest noise
        t0_events: List[Tuple[float, str]] = []
        log_progress(True, "T0 reposo/ruido Hall")
        t0_start = time.monotonic()
        next_speed_poll = t0_start
        next_spid_poll = t0_start
        last_p = t0_start
        while time.monotonic() - t0_start < 8.0:
            t0_events.extend(c.collect(0.15, poll=0.05))
            now = time.monotonic()
            if now >= next_speed_poll:
                t0_events.extend(c.send("speed.status", wait=0.03))
                next_speed_poll = now + 0.5
            if now >= next_spid_poll:
                t0_events.extend(c.send("spid.status", wait=0.03))
                next_spid_poll = now + 0.5
            if now - last_p >= args.progress_interval_s:
                log_progress(True, f"T0 t={now - t0_start:.1f}/8.0s")
                last_p = now

        t0_speeds_raw = [s for e in t0_events if (s := parse_speed(e)) is not None]
        t0_spids_raw = [s for e in t0_events if (s := parse_spid(e)) is not None]
        t0_pid_raw = [p for e in t0_events if (p := parse_pidtrace(e)) is not None]
        t0_speeds, t0_speed_out = filter_speed_series(t0_speeds_raw, args.max_valid_speed_mps, args.max_speed_jump_mps2)
        t0_spids, t0_spid_out = filter_spid_series(t0_spids_raw, args.max_valid_speed_mps, args.max_speed_jump_mps2)
        append_rows(csv_rows, "T0", "rest_noise", t0_speeds, t0_spids, t0_pid_raw)

        t0: Dict[str, Any] = {"samples_speed": len(t0_speeds), "samples_spid": len(t0_spids)}
        if len(t0_speeds) >= 2:
            d_ok = t0_speeds[-1].ok - t0_speeds[0].ok
            d_inv = t0_speeds[-1].inv_state - t0_speeds[0].inv_state
            d_jump = t0_speeds[-1].inv_jump - t0_speeds[0].inv_jump
            d_isr = t0_speeds[-1].isr - t0_speeds[0].isr
            max_mps = max(s.mps for s in t0_speeds)
            ratio = d_inv / max(1, d_ok)
            t0.update(
                {
                    "ok_delta": d_ok,
                    "inv_state_delta": d_inv,
                    "inv_jump_delta": d_jump,
                    "isr_delta": d_isr,
                    "max_speed_mps": round(max_mps, 3),
                    "inv_ok_ratio": round(ratio, 3),
                }
            )
            t0["pass"] = bool(max_mps < args.rest_max_mps and d_jump == 0 and ratio <= 1.0)
        else:
            t0["pass"] = False
            t0["error"] = "sin muestras suficientes"
        report["tests"]["T0_rest_noise"] = t0
        report["outliers"]["speed_status_t0"] = t0_speed_out
        report["outliers"]["spid_t0"] = t0_spid_out

        # T1 start steps (quick/full)
        t1_steps = [0.8, 1.4, 1.94] if args.mode == "quick" else [0.8, 1.4, 1.94, 2.5]
        t1_results: List[Dict[str, Any]] = []
        t2_results: List[Dict[str, Any]] = []
        t3_result: Dict[str, Any] = {}

        for target in t1_steps:
            seg_name = f"T1_step_{target:.2f}"
            events = run_collection_segment(c, seg_name, target, run_s=5.5, decay_s=8.0, progress_enabled=True, progress_interval_s=args.progress_interval_s)
            spids_raw = [s for e in events if (s := parse_spid(e)) is not None]
            speeds_raw = [s for e in events if (s := parse_speed(e)) is not None]
            pids = [p for e in events if (p := parse_pidtrace(e)) is not None]
            spids, spid_out = filter_spid_series(spids_raw, args.max_valid_speed_mps, args.max_speed_jump_mps2)
            speeds, speed_out = filter_speed_series(speeds_raw, args.max_valid_speed_mps, args.max_speed_jump_mps2)
            append_rows(csv_rows, "T1", seg_name, speeds, spids, pids)
            res = summarize_step(target, spids, speeds, pids, args.require_reach_frac, args.rest_threshold_mps,
                                 args.failsafe_sustain_threshold_s, decay_duration_s=8.0)
            res["outliers"] = {"speed_status": speed_out, "spid": spid_out}
            t1_results.append(res)
        report["tests"]["T1_start_steps"] = t1_results

        # T2 holds (full mode only)
        if args.mode == "full":
            for target, hold_s in [(1.4, 15.0), (1.94, 20.0), (2.8, 15.0)]:
                seg_name = f"T2_hold_{target:.2f}"
                events = run_collection_segment(c, seg_name, target, run_s=hold_s, decay_s=6.0, progress_enabled=True, progress_interval_s=args.progress_interval_s)
                spids_raw = [s for e in events if (s := parse_spid(e)) is not None]
                speeds_raw = [s for e in events if (s := parse_speed(e)) is not None]
                pids = [p for e in events if (p := parse_pidtrace(e)) is not None]
                spids, spid_out = filter_spid_series(spids_raw, args.max_valid_speed_mps, args.max_speed_jump_mps2)
                speeds, speed_out = filter_speed_series(speeds_raw, args.max_valid_speed_mps, args.max_speed_jump_mps2)
                append_rows(csv_rows, "T2", seg_name, speeds, spids, pids)
                res = summarize_step(target, spids, speeds, pids, args.require_reach_frac, args.rest_threshold_mps,
                                     args.failsafe_sustain_threshold_s, decay_duration_s=6.0)
                # Hold-specific metrics
                hold_spids = [s for s in spids if s.target > 0.05]
                hold_pid = [p for p in pids if p.target_mps > 0.05]
                res["hold_avg_speed_mps"] = round(sum(s.speed for s in hold_spids) / len(hold_spids), 3) if hold_spids else 0.0
                res["hold_avg_throttle_pct"] = round(sum(p.throttle_filt_pct for p in hold_pid) / len(hold_pid), 2) if hold_pid else 0.0
                res["hold_dominant_inhibit"] = max(
                    ({k: sum(1 for p in hold_pid if p.inhibit == k) for k in set(p.inhibit for p in hold_pid)} or {"NONE": 0}),
                    key=lambda k: ({kk: sum(1 for p in hold_pid if p.inhibit == kk) for kk in set(p.inhibit for p in hold_pid)} or {"NONE": 0})[k],
                ) if hold_pid else "NONE"
                res["outliers"] = {"speed_status": speed_out, "spid": spid_out}
                t2_results.append(res)
            report["tests"]["T2_holds"] = t2_results

        # T3 release/inertia (always)
        events = run_collection_segment(c, "T3_release", 1.94, run_s=5.0, decay_s=10.0, progress_enabled=True, progress_interval_s=args.progress_interval_s)
        spids_raw = [s for e in events if (s := parse_spid(e)) is not None]
        speeds_raw = [s for e in events if (s := parse_speed(e)) is not None]
        pids = [p for e in events if (p := parse_pidtrace(e)) is not None]
        spids, spid_out = filter_spid_series(spids_raw, args.max_valid_speed_mps, args.max_speed_jump_mps2)
        speeds, speed_out = filter_speed_series(speeds_raw, args.max_valid_speed_mps, args.max_speed_jump_mps2)
        append_rows(csv_rows, "T3", "release_zero", speeds, spids, pids)
        t3_result = summarize_step(1.94, spids, speeds, pids, args.require_reach_frac, args.rest_threshold_mps,
                                   args.failsafe_sustain_threshold_s, decay_duration_s=10.0)
        t3_result["outliers"] = {"speed_status": speed_out, "spid": spid_out}
        report["tests"]["T3_release_to_zero"] = t3_result

        # T4 correlation from all captured rows in full mode (and quick if fs appears)
        if args.mode == "full" or any(s.get("failsafe_events", 0) > 0 for s in t1_results):
            # Build correlation only from T1/T2/T3 by replaying csv rows isn't ideal; use parsed rows from CSV later is enough,
            # but here we collect fail events from current summaries for structured evidence.
            corr: List[Dict[str, Any]] = []
            for bucket_name in ("T1_start_steps", "T2_holds", "T3_release_to_zero"):
                bucket = report["tests"].get(bucket_name)
                if isinstance(bucket, list):
                    for item in bucket:
                        ctx = item.get("failsafe_first_context")
                        if ctx is not None:
                            corr.append({"bucket": bucket_name, "target_mps": item.get("target_mps"), "event": "FAILSAFE_ENTER", "context": ctx})
                elif isinstance(bucket, dict):
                    ctx = bucket.get("failsafe_first_context")
                    if ctx is not None:
                        corr.append({"bucket": bucket_name, "target_mps": bucket.get("target_mps"), "event": "FAILSAFE_ENTER", "context": ctx})
            corr_events.extend(corr)
            report["tests"]["T4_pidtrace_correlation"] = {
                "events": len(corr),
                "has_failsafe_correlation": bool(corr),
                "examples": corr[:8],
            }

        # Summary + gate
        ping_stats = report.get("network", {}).get("ping", {})
        ping_loss = ping_stats.get("loss_pct")
        network_stable = bool(report.get("network", {}).get("telnet", {}).get("ok", False)) and (ping_loss is None or ping_loss < 60.0)

        t1_fail_count = 0
        for s in t1_results:
            if (not s.get("reached_target_60pct", False)) or s.get("failsafe_sustained", False):
                t1_fail_count += 1

        hold_gate_fail = 0
        for s in t2_results:
            if s.get("target_mps") in (1.4, 1.94) and (s.get("failsafe_events", 0) > 0 or s.get("failsafe_sustained", False)):
                hold_gate_fail += 1

        outlier_ratio_global = max(
            [0.0]
            + [((s.get("outliers", {}).get("speed_status", {}) or {}).get("outlier_ratio", 0.0)) for s in t1_results]
            + [((s.get("outliers", {}).get("spid", {}) or {}).get("outlier_ratio", 0.0)) for s in t1_results]
            + [((s.get("outliers", {}).get("speed_status", {}) or {}).get("outlier_ratio", 0.0)) for s in t2_results]
            + [((s.get("outliers", {}).get("spid", {}) or {}).get("outlier_ratio", 0.0)) for s in t2_results]
            + [((t3_result.get("outliers", {}).get("speed_status", {}) or {}).get("outlier_ratio", 0.0))]
            + [((t3_result.get("outliers", {}).get("spid", {}) or {}).get("outlier_ratio", 0.0))]
        )

        act_probe = (report.get("tests", {}) or {}).get("ACTUATION_PROBE", {})

        hall_diagnostic_ok = bool(
            act_probe.get("pass", False)
            and not act_probe.get("no_actuation_command", False)
            and t0.get("pass", False)
            and t1_fail_count == 0
            and (args.mode == "quick" or hold_gate_fail == 0)
            and outlier_ratio_global < 0.10
            and network_stable
        )

        primary_cause, notes = classify_cause(report)
        report["notes"].extend(notes)
        report["summary"] = {
            "network_stable": network_stable,
            "actuation_probe_pass": bool(act_probe.get("pass", False)),
            "actuation_probe_motion_seen": bool(act_probe.get("motion_seen", False)),
            "t1_fail_count": t1_fail_count,
            "hold_gate_fail": hold_gate_fail,
            "outlier_ratio_global": round(outlier_ratio_global, 4),
            "hall_diagnostic_ok": hall_diagnostic_ok,
            "primary_cause": primary_cause,
        }

    except Exception as exc:
        report.setdefault("notes", []).append(str(exc))
        if "preflight" not in report:
            report["preflight"] = {}
        report["preflight"].setdefault("precheck_code", "PRECHECK_EXCEPTION")
        primary_cause, notes = classify_cause(report)
        report["notes"].extend(notes)
        report["summary"] = {"hall_diagnostic_ok": False, "primary_cause": primary_cause}
    finally:
        c.close()

    # Correlation artifact finalize
    with open(args.out_correlation, "w", encoding="utf-8") as f:
        json.dump({"generated_at_utc": now_iso(), "events": corr_events}, f, ensure_ascii=False, indent=2)
    write_csv(args.out_csv, csv_rows)

    report["finished_at_utc"] = now_iso()
    with open(args.out_json, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)

    # Markdown summary
    lines = [
        "# Hall HIL Check",
        "",
        f"- Start (UTC): {report.get('started_at_utc')}",
        f"- End (UTC): {report.get('finished_at_utc')}",
        f"- Target: `{args.host}:{args.port}`",
        f"- Mode: `{args.mode}`",
        "",
        "## Summary",
        "",
        f"- hall_diagnostic_ok: **{(report.get('summary') or {}).get('hall_diagnostic_ok')}**",
        f"- primary_cause: `{(report.get('summary') or {}).get('primary_cause')}`",
        f"- network_stable: `{(report.get('summary') or {}).get('network_stable')}`",
        f"- outlier_ratio_global: `{(report.get('summary') or {}).get('outlier_ratio_global')}`",
        "",
        "## Preflight",
        "",
        f"- precheck_code: `{(report.get('preflight') or {}).get('precheck_code')}`",
        "",
    ]
    act = (report.get("tests") or {}).get("ACTUATION_PROBE", {})
    if act:
        lines.append("## Actuation Probe")
        lines.append("")
        for k in (
            "pass", "target_seen", "throttle_command_seen", "motion_seen", "no_actuation_command",
            "max_pid_target_mps", "max_pid_out_pct", "max_throttle_filt_pct", "max_pwm_duty_pct",
            "max_spid_speed_mps", "max_speed_status_mps", "dominant_inhibit_after_grace",
        ):
            if k in act:
                lines.append(f"- {k}: {act[k]}")
        lines.append("")
    t0 = (report.get("tests") or {}).get("T0_rest_noise", {})
    if t0:
        lines.append("## T0 Rest Noise")
        lines.append("")
        for k in ("pass", "max_speed_mps", "ok_delta", "inv_state_delta", "inv_jump_delta", "isr_delta", "inv_ok_ratio"):
            if k in t0:
                lines.append(f"- {k}: {t0[k]}")
        lines.append("")
    if (report.get("tests") or {}).get("T1_start_steps"):
        lines.append("## T1 Steps")
        lines.append("")
        for s in report["tests"]["T1_start_steps"]:
            lines.append(
                f"- target={s['target_mps']:.2f} maxSpid={s['max_spid_speed_mps']:.2f} maxHall={s['max_speed_status_mps']:.2f} "
                f"fsEv={s['failsafe_events']} fsSustain={s['failsafe_sustain_s']} reach60={s['reached_target_60pct']}"
            )
        lines.append("")
    if (report.get("tests") or {}).get("T2_holds"):
        lines.append("## T2 Holds")
        lines.append("")
        for s in report["tests"]["T2_holds"]:
            lines.append(
                f"- target={s['target_mps']:.2f} fsEv={s['failsafe_events']} fsSustain={s['failsafe_sustain_s']} "
                f"fbFalse={s['fb_false_ratio']} avgSpeed={s.get('hold_avg_speed_mps')} inhibit={s.get('hold_dominant_inhibit')}"
            )
        lines.append("")
    with open(out_md, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    print(f"Report JSON: {args.out_json}")
    print(f"Report MD:   {out_md}")
    print(f"Report CSV:  {args.out_csv}")
    print(f"Corr JSON:   {args.out_correlation}")
    print(f"Hall OK:     {(report.get('summary') or {}).get('hall_diagnostic_ok')}")
    return 0 if (report.get("summary") or {}).get("hall_diagnostic_ok") else 2


def main(argv: List[str]) -> int:
    return run(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
