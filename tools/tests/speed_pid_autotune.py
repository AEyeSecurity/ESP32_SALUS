#!/usr/bin/env python3
"""Automatic Speed PID calibration using [DRIVE][PIDTRACE] telemetry."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import statistics
import subprocess
import sys
import telnetlib
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from typing import Any, Dict, Iterable, List, Optional, Tuple

RE_SPLIT_MARKER = re.compile(r"(?<!\n)(?<!\])(?=\[[A-Z])")
RE_TOKENS = re.compile(r"([A-Za-z_]+)=([^\s]+)")
RE_COMMS_LAST_FRAME = re.compile(r"lastFrame=(NONE|\d+ms)")


@dataclass
class Tunings:
    kp: float
    ki: float
    kd: float


@dataclass
class Structure:
    thslewup: float
    thslewdown: float
    minth_spd: float
    launchwin_ms: int
    iunwind: float
    dfilter_hz: float


@dataclass
class Candidate:
    phase: str
    label: str
    tunings: Tunings
    structure: Structure


@dataclass
class TraceSample:
    ts: float
    target_cmd_mps: float
    segment: str
    src: str
    mode: str
    target_raw_mps: float
    target_mps: float
    speed_mps: float
    err_mps: float
    p: float
    i: float
    d: float
    pid_unsat: float
    pid_out_pct: float
    pid_sat_pct: float
    throttle_raw_pct: float
    throttle_filt_pct: float
    auto_brake_raw_pct: float
    auto_brake_filt_pct: float
    brake_applied_pct: float
    fb: bool
    fs: bool
    ovs: bool
    estop: bool
    inhibit: str
    launch_assist: bool
    throttle_saturated: bool
    integrator_clamped: bool
    raw_line: str


@dataclass
class SegmentMetrics:
    segment: str
    target_cmd_mps: float
    duration_s: float
    overshoot_pct: float
    settling_time_5pct_s: Optional[float]
    steady_state_rms_error_mps: float
    speed_ripple_p95_mps: float
    throttle_jerk_p95_pct_per_100ms: float
    throttle_burst_count: int
    brake_delta_p95_pct: float
    overspeed_transition_rate_per_min: float
    failsafe_events: int
    inhibit_toggle_rate_per_min: float


@dataclass
class RunMetrics:
    score: float
    overshoot_max_pct: float
    settling_time_5pct_max_s: Optional[float]
    steady_state_rms_error_mps: float
    speed_ripple_p95_mps: float
    throttle_jerk_p95_pct_per_100ms: float
    throttle_burst_count_per_min: float
    brake_delta_p95_pct: float
    overspeed_transition_rate_per_min: float
    failsafe_events: int
    inhibit_toggle_rate_per_min: float


@dataclass
class RunResult:
  candidate: Candidate
  profile_name: str
  segments: List[SegmentMetrics]
  metrics: RunMetrics
  abort_reason: Optional[str] = None
  abort_class: Optional[str] = None
  abort_detail: Dict[str, Any] = field(default_factory=dict)


def log_progress(msg: str) -> None:
    ts = time.strftime("%H:%M:%S", time.localtime())
    print(f"[AUTOTUNE][{ts}] {msg}", flush=True)


def now_utc_iso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def ensure_parent(path: str) -> None:
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)


def to_float(text: Optional[str]) -> Optional[float]:
    if text is None:
        return None
    s = text.strip().rstrip(",}])")
    for suffix in ("m/s2", "m/s", "%", "deg", "ms", "km/h"):
        if s.endswith(suffix):
            s = s[: -len(suffix)].strip()
            break
    if not s:
        return None
    try:
        return float(s)
    except ValueError:
        return None


def to_bool_yn(text: Optional[str]) -> Optional[bool]:
    if text == "Y":
        return True
    if text == "N":
        return False
    return None


def percentile(values: List[float], p: float) -> float:
    if not values:
        return 0.0
    if p <= 0:
        return min(values)
    if p >= 100:
        return max(values)
    vals = sorted(values)
    k = (len(vals) - 1) * (p / 100.0)
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return vals[int(k)]
    return vals[f] + (vals[c] - vals[f]) * (k - f)


def safe_mean(values: Iterable[float], default: float = 0.0) -> float:
  vals = list(values)
  return float(sum(vals) / len(vals)) if vals else default


def clamp_value(v: float, lo: float, hi: float) -> float:
  return lo if v < lo else hi if v > hi else v


class TelnetRunner:
    def __init__(self, host: str, port: int, timeout: float):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.tn: Optional[telnetlib.Telnet] = None

    def connect(self) -> List[Tuple[float, str]]:
        self.tn = telnetlib.Telnet(self.host, self.port, timeout=self.timeout)
        time.sleep(0.7)
        return self.read_events()

    def close(self) -> None:
        if self.tn is None:
            return
        try:
            self.tn.write(b"exit\n")
        except Exception:
            pass
        try:
            self.tn.close()
        except Exception:
            pass
        self.tn = None

    def read_events(self) -> List[Tuple[float, str]]:
        if self.tn is None:
            return []
        try:
            data = self.tn.read_very_eager()
        except EOFError:
            return []
        if not data:
            return []
        decoded = RE_SPLIT_MARKER.sub("\n", data.decode("utf-8", errors="ignore"))
        ts = time.monotonic()
        out: List[Tuple[float, str]] = []
        for ln in decoded.splitlines():
            line = ln.strip()
            if line:
                out.append((ts, line))
        return out

    def send(self, cmd: str, wait_s: float = 0.30) -> List[Tuple[float, str]]:
        if self.tn is None:
            raise RuntimeError("Telnet not connected")
        self.tn.write((cmd + "\n").encode("utf-8"))
        time.sleep(wait_s)
        return self.read_events()

    def collect(self, duration_s: float, poll_s: float = 0.06) -> List[Tuple[float, str]]:
        out: List[Tuple[float, str]] = []
        end = time.monotonic() + duration_s
        while time.monotonic() < end:
            out.extend(self.read_events())
            time.sleep(poll_s)
        out.extend(self.read_events())
        return out


def parse_comms_status(line: str) -> Dict[str, Any]:
    out: Dict[str, Any] = {"raw_line": line}
    m = RE_COMMS_LAST_FRAME.search(line)
    if m:
        lf = m.group(1)
        out["lastFrame"] = lf
        if lf == "NONE":
            out["lastFrameMs"] = None
            out["piFresh"] = False
        else:
            ms = int(lf[:-2])
            out["lastFrameMs"] = ms
            out["piFresh"] = ms <= 120
    mm = re.search(r"\bestop=([YN])", line)
    if mm:
        out["estop"] = to_bool_yn(mm.group(1))
    mm = re.search(r"\bbrake=(\d+)", line)
    if mm:
        out["brake"] = int(mm.group(1))
    return out


def parse_speed_status(line: str) -> Dict[str, Any]:
    out: Dict[str, Any] = {"raw_line": line}
    mm = re.search(r"\bdriver=([A-Z_]+)", line)
    if mm:
        out["driver"] = mm.group(1)
    return out


def parse_spid_status(line: str) -> Dict[str, Any]:
    out: Dict[str, Any] = {"raw_line": line}
    keys = [
        "kp", "ki", "kd", "max", "ramp", "ilim", "db", "tmin", "thsup", "thsdown", "minspd",
        "lwin", "iunw", "dfhz", "cap", "hys", "brsu", "brsd", "brh", "brdb"
    ]
    for key in keys:
        m = re.search(r"\b" + re.escape(key) + r"=([-+]?\d+(?:\.\d+)?)", line)
        if m:
            out[key] = float(m.group(1))
    out["init"] = "state{init=Y" in line
    return out


def parse_pidtrace(ts: float, line: str, target_cmd_mps: float, segment: str) -> Optional[TraceSample]:
    if not line.startswith("[DRIVE][PIDTRACE]"):
        return None
    data: Dict[str, str] = {k: v for k, v in RE_TOKENS.findall(line)}

    def f(name: str, default: float = 0.0) -> float:
        return to_float(data.get(name)) if to_float(data.get(name)) is not None else default

    def b(name: str, default: bool = False) -> bool:
        parsed = to_bool_yn(data.get(name))
        return parsed if parsed is not None else default

    return TraceSample(
        ts=ts,
        target_cmd_mps=target_cmd_mps,
        segment=segment,
        src=data.get("src", "NONE"),
        mode=data.get("mode", "UNKNOWN"),
        target_raw_mps=f("targetRawMps"),
        target_mps=f("targetMps"),
        speed_mps=f("speedMps"),
        err_mps=f("errMps"),
        p=f("p"),
        i=f("i"),
        d=f("d"),
        pid_unsat=f("pidUnsat"),
        pid_out_pct=f("pidOutPct"),
        pid_sat_pct=f("pidSatPct"),
        throttle_raw_pct=f("throttleRawPct"),
        throttle_filt_pct=f("throttleFiltPct"),
        auto_brake_raw_pct=f("autoBrakeRawPct"),
        auto_brake_filt_pct=f("autoBrakeFiltPct"),
        brake_applied_pct=f("brakeAppliedPct"),
        fb=b("fb", True),
        fs=b("fs", False),
        ovs=b("ovs", False),
        estop=b("estop", False),
        inhibit=data.get("inhibit", "NONE"),
        launch_assist=b("launchAssistActive", b("launch", False)),
        throttle_saturated=b("throttleSaturated", b("sat", False)),
        integrator_clamped=b("integratorClamped", b("iclamp", False)),
        raw_line=line,
    )


def choose_last(events: List[Tuple[float, str]], prefix: str) -> Optional[str]:
    for _, line in reversed(events):
        if line.startswith(prefix):
            return line
    return None


def apply_candidate(tn: TelnetRunner, candidate: Candidate) -> None:
    cmds = [
        f"spid.set {candidate.tunings.kp:.6f} {candidate.tunings.ki:.6f} {candidate.tunings.kd:.6f}",
        f"spid.thslewup {candidate.structure.thslewup:.3f}",
        f"spid.thslewdown {candidate.structure.thslewdown:.3f}",
        f"spid.minth.spd {candidate.structure.minth_spd:.3f}",
        f"spid.launchwin {candidate.structure.launchwin_ms}",
        f"spid.iunwind {candidate.structure.iunwind:.3f}",
        f"spid.dfilter {candidate.structure.dfilter_hz:.3f}",
    ]
    for cmd in cmds:
        tn.send(cmd, wait_s=0.20)


def run_profile(
    tn: TelnetRunner,
    candidate: Candidate,
    profile_name: str,
    segments: List[Tuple[str, float, float]],
    csv_rows: List[Dict[str, Any]],
    progress_interval_s: float,
    movement_threshold_mps: float,
    no_motion_grace_s: float,
    no_motion_min_target_mps: float,
    no_motion_min_throttle_pct: float,
    max_valid_speed_mps: float,
) -> RunResult:
    apply_candidate(tn, candidate)
    tn.send("spid.target off", wait_s=0.15)
    tn.send("speed.reset", wait_s=0.15)

    traces: List[TraceSample] = []
    abort_reason: Optional[str] = None
    abort_class: Optional[str] = None
    abort_detail: Dict[str, Any] = {}

    for seg_name, target, duration_s in segments:
        log_progress(
            f"{candidate.phase}/{candidate.label} seg={seg_name} start "
            f"target={target:.2f}m/s dur={duration_s:.1f}s"
        )
        tn.send(f"spid.target {target:.3f}", wait_s=0.12)
        seg_start = time.monotonic()
        seg_end = seg_start + duration_s
        seg_peak_speed = 0.0
        seg_peak_throttle = 0.0
        seg_sample_count = 0
        seg_fs_count = 0
        seg_inhibit_count = 0
        seg_fb_bad_count = 0
        last_progress = seg_start
        last_sample: Optional[TraceSample] = None

        while time.monotonic() < seg_end:
            remaining = max(0.0, seg_end - time.monotonic())
            if remaining <= 0:
                break
            raw_events = tn.collect(min(0.22, remaining), poll_s=0.05)
            for ts, line in raw_events:
                sample = parse_pidtrace(ts, line, target_cmd_mps=target, segment=seg_name)
                if sample is None:
                    continue
                if sample.speed_mps > max_valid_speed_mps or sample.speed_mps < -0.2:
                    abort_detail.setdefault("speed_outlier_samples", 0)
                    abort_detail["speed_outlier_samples"] = int(abort_detail["speed_outlier_samples"]) + 1
                    continue
                last_sample = sample
                seg_peak_speed = max(seg_peak_speed, sample.speed_mps)
                seg_peak_throttle = max(seg_peak_throttle, sample.throttle_filt_pct)
                seg_sample_count += 1
                if sample.fs:
                    seg_fs_count += 1
                if sample.inhibit != "NONE":
                    seg_inhibit_count += 1
                if not sample.fb:
                    seg_fb_bad_count += 1
                traces.append(sample)
                csv_rows.append(
                    {
                        "profile": profile_name,
                        "phase": candidate.phase,
                        "candidate": candidate.label,
                        "segment": seg_name,
                        "ts": f"{sample.ts:.6f}",
                        "target_cmd_mps": f"{target:.3f}",
                        "target_raw_mps": f"{sample.target_raw_mps:.3f}",
                        "target_mps": f"{sample.target_mps:.3f}",
                        "speed_mps": f"{sample.speed_mps:.3f}",
                        "err_mps": f"{sample.err_mps:.3f}",
                        "mode": sample.mode,
                        "pid_out_pct": f"{sample.pid_out_pct:.3f}",
                        "throttle_raw_pct": f"{sample.throttle_raw_pct:.3f}",
                        "throttle_filt_pct": f"{sample.throttle_filt_pct:.3f}",
                        "auto_brake_filt_pct": f"{sample.auto_brake_filt_pct:.3f}",
                        "fb": int(sample.fb),
                        "fs": int(sample.fs),
                        "ovs": int(sample.ovs),
                        "inhibit": sample.inhibit,
                        "launch": int(sample.launch_assist),
                        "sat": int(sample.throttle_saturated),
                        "iclamp": int(sample.integrator_clamped),
                    }
                )
                if sample.estop:
                    abort_reason = "ESTOP activo durante corrida"
                    abort_class = "ESTOP"
                    abort_detail = {"segment": seg_name}

            now_mono = time.monotonic()
            if now_mono - last_progress >= progress_interval_s:
                elapsed = now_mono - seg_start
                if last_sample is not None:
                    log_progress(
                        f"{candidate.label} seg={seg_name} t={elapsed:.1f}/{duration_s:.1f}s "
                        f"speed={last_sample.speed_mps:.2f} target={last_sample.target_mps:.2f} "
                        f"thr={last_sample.throttle_filt_pct:.1f}% br={last_sample.auto_brake_filt_pct:.1f}% "
                        f"mode={last_sample.mode} fs={'Y' if last_sample.fs else 'N'}"
                    )
                else:
                    log_progress(
                        f"{candidate.label} seg={seg_name} t={elapsed:.1f}/{duration_s:.1f}s sin PIDTRACE"
                    )
                last_progress = now_mono

            if target >= no_motion_min_target_mps and (now_mono - seg_start) >= no_motion_grace_s:
                fs_ratio = (seg_fs_count / seg_sample_count) if seg_sample_count > 0 else 0.0
                inhibit_ratio = (seg_inhibit_count / seg_sample_count) if seg_sample_count > 0 else 0.0
                fb_bad_ratio = (seg_fb_bad_count / seg_sample_count) if seg_sample_count > 0 else 0.0
                no_motion_detected = seg_peak_speed < movement_threshold_mps
                enough_effort = seg_peak_throttle >= no_motion_min_throttle_pct
                failsafe_dominant = fs_ratio >= 0.50
                inhibit_dominant = inhibit_ratio >= 0.50
                if no_motion_detected and (enough_effort or failsafe_dominant or inhibit_dominant):
                    if fs_ratio >= 0.45:
                        abort_class = "NO_MOTION_FAILSAFE"
                    elif inhibit_ratio >= 0.45:
                        abort_class = "NO_MOTION_INHIBIT"
                    else:
                        abort_class = "NO_MOTION_TRUE"
                    abort_detail = {
                        "segment": seg_name,
                        "target_mps": target,
                        "peak_speed_mps": seg_peak_speed,
                        "peak_throttle_pct": seg_peak_throttle,
                        "sample_count": seg_sample_count,
                        "fs_ratio": fs_ratio,
                        "inhibit_ratio": inhibit_ratio,
                        "fb_bad_ratio": fb_bad_ratio,
                    }
                    abort_reason = (
                        f"{abort_class} en {seg_name}: peak={seg_peak_speed:.2f}m/s "
                        f"< thr={movement_threshold_mps:.2f}m/s thrPeak={seg_peak_throttle:.1f}% "
                        f"fs={fs_ratio:.2f} inh={inhibit_ratio:.2f} fbBad={fb_bad_ratio:.2f} "
                        f"effort={'Y' if enough_effort else 'N'}"
                    )
                    break
            if abort_reason is not None:
                break

        log_progress(
            f"{candidate.label} seg={seg_name} end peak={seg_peak_speed:.2f}m/s "
            f"abort={'none' if abort_reason is None else abort_reason}"
        )
        if abort_reason is not None:
            break

    tn.send("spid.target 0.000", wait_s=0.12)
    tn.send("spid.target off", wait_s=0.12)

    seg_metrics = compute_metrics(traces)
    run_metrics = aggregate_metrics(seg_metrics)

    return RunResult(
        candidate=candidate,
        profile_name=profile_name,
        segments=seg_metrics,
        metrics=run_metrics,
        abort_reason=abort_reason,
        abort_class=abort_class,
        abort_detail=abort_detail,
    )


def compute_metrics(samples: List[TraceSample]) -> List[SegmentMetrics]:
    out: List[SegmentMetrics] = []
    by_segment: Dict[str, List[TraceSample]] = {}
    for s in samples:
        by_segment.setdefault(s.segment, []).append(s)

    for segment, seg in by_segment.items():
        if not seg:
            continue
        seg = sorted(seg, key=lambda x: x.ts)
        target = seg[0].target_cmd_mps
        if target <= 0.01:
            continue

        duration_s = max(0.001, seg[-1].ts - seg[0].ts)
        speeds = [s.speed_mps for s in seg]
        errors = [target - s.speed_mps for s in seg]
        abs_errors = [abs(e) for e in errors]
        overshoot_pct = max(0.0, (max(speeds) - target) / max(target, 1e-6) * 100.0)

        tol = 0.05 * max(target, 0.05)
        settling_time = None
        for i in range(len(seg)):
            tail = seg[i:]
            if tail and all(abs(target - t.speed_mps) <= tol for t in tail):
                settling_time = max(0.0, seg[i].ts - seg[0].ts)
                break

        steady_start = seg[0].ts + duration_s * 0.7
        steady = [s for s in seg if s.ts >= steady_start]
        if not steady:
            steady = seg[-max(8, len(seg) // 3):]
        ss_errors = [target - s.speed_mps for s in steady]
        steady_rms = math.sqrt(safe_mean([e * e for e in ss_errors], 0.0))

        steady_speeds = [s.speed_mps for s in steady]
        baseline_speed = statistics.median(steady_speeds) if steady_speeds else 0.0
        speed_ripple = percentile([abs(v - baseline_speed) for v in steady_speeds], 95)

        jerk_values: List[float] = []
        burst_count = 0
        brake_deltas: List[float] = []
        overspeed_transitions = 0
        failsafe_events = 0
        inhibit_toggles = 0
        prev_mode = seg[0].mode
        prev_fs = seg[0].fs
        prev_inhibit = seg[0].inhibit

        for idx in range(1, len(seg)):
            curr = seg[idx]
            prev = seg[idx - 1]
            dt = max(1e-3, curr.ts - prev.ts)

            throttle_delta = curr.throttle_filt_pct - prev.throttle_filt_pct
            jerk_100ms = abs(throttle_delta) * (0.1 / dt)
            jerk_values.append(jerk_100ms)
            if abs(throttle_delta) > 20.0 and dt <= 0.2:
                burst_count += 1

            brake_deltas.append(abs(curr.auto_brake_filt_pct - prev.auto_brake_filt_pct))

            if curr.mode != prev_mode and (curr.mode == "OVERSPEED" or prev_mode == "OVERSPEED"):
                overspeed_transitions += 1
            prev_mode = curr.mode

            if curr.fs and not prev_fs:
                failsafe_events += 1
            prev_fs = curr.fs

            if curr.inhibit != prev_inhibit:
                inhibit_toggles += 1
            prev_inhibit = curr.inhibit

        minutes = duration_s / 60.0
        overspeed_rate = overspeed_transitions / minutes if minutes > 0 else 0.0
        inhibit_rate = inhibit_toggles / minutes if minutes > 0 else 0.0

        out.append(
            SegmentMetrics(
                segment=segment,
                target_cmd_mps=target,
                duration_s=duration_s,
                overshoot_pct=overshoot_pct,
                settling_time_5pct_s=settling_time,
                steady_state_rms_error_mps=steady_rms,
                speed_ripple_p95_mps=speed_ripple,
                throttle_jerk_p95_pct_per_100ms=percentile(jerk_values, 95),
                throttle_burst_count=burst_count,
                brake_delta_p95_pct=percentile(brake_deltas, 95),
                overspeed_transition_rate_per_min=overspeed_rate,
                failsafe_events=failsafe_events,
                inhibit_toggle_rate_per_min=inhibit_rate,
            )
        )

    return out


def aggregate_metrics(segments: List[SegmentMetrics]) -> RunMetrics:
    if not segments:
        return RunMetrics(
            score=1e12,
            overshoot_max_pct=999.0,
            settling_time_5pct_max_s=None,
            steady_state_rms_error_mps=999.0,
            speed_ripple_p95_mps=999.0,
            throttle_jerk_p95_pct_per_100ms=999.0,
            throttle_burst_count_per_min=999.0,
            brake_delta_p95_pct=999.0,
            overspeed_transition_rate_per_min=999.0,
            failsafe_events=999,
            inhibit_toggle_rate_per_min=999.0,
        )

    total_duration_s = sum(s.duration_s for s in segments)
    total_minutes = max(total_duration_s / 60.0, 1e-6)
    overshoot_max = max(s.overshoot_pct for s in segments)
    settling_values = [s.settling_time_5pct_s for s in segments if s.settling_time_5pct_s is not None]
    settle_max = max(settling_values) if settling_values else None

    rms_max = max(s.steady_state_rms_error_mps for s in segments)
    ripple_max = max(s.speed_ripple_p95_mps for s in segments)
    jerk_max = max(s.throttle_jerk_p95_pct_per_100ms for s in segments)
    brake_delta_max = max(s.brake_delta_p95_pct for s in segments)

    burst_per_min = sum(s.throttle_burst_count for s in segments) / total_minutes
    overspeed_rate = safe_mean([s.overspeed_transition_rate_per_min for s in segments], 0.0)
    inhibit_rate = safe_mean([s.inhibit_toggle_rate_per_min for s in segments], 0.0)
    failsafe_total = sum(s.failsafe_events for s in segments)

    score = 0.0
    score += overshoot_max * 4.0
    score += rms_max * 260.0
    score += ripple_max * 180.0
    score += jerk_max * 2.4
    score += burst_per_min * 100.0
    score += brake_delta_max * 6.0
    score += overspeed_rate * 8.0
    score += inhibit_rate * 4.0
    if settle_max is not None:
        score += settle_max * 1.2
    else:
        score += 120.0
    if failsafe_total > 0:
        score += 50000.0 + failsafe_total * 20000.0

    return RunMetrics(
        score=score,
        overshoot_max_pct=overshoot_max,
        settling_time_5pct_max_s=settle_max,
        steady_state_rms_error_mps=rms_max,
        speed_ripple_p95_mps=ripple_max,
        throttle_jerk_p95_pct_per_100ms=jerk_max,
        throttle_burst_count_per_min=burst_per_min,
        brake_delta_p95_pct=brake_delta_max,
        overspeed_transition_rate_per_min=overspeed_rate,
        failsafe_events=failsafe_total,
        inhibit_toggle_rate_per_min=inhibit_rate,
    )


def acceptance_check(metrics_a: RunMetrics, metrics_b: RunMetrics, baseline: RunMetrics) -> Dict[str, Any]:
    avg_score = (metrics_a.score + metrics_b.score) / 2.0
    score_var = abs(metrics_a.score - metrics_b.score) / max(avg_score, 1e-6)

    overspeed_baseline = baseline.overspeed_transition_rate_per_min
    overspeed_candidate = (metrics_a.overspeed_transition_rate_per_min + metrics_b.overspeed_transition_rate_per_min) / 2.0
    if overspeed_baseline <= 0.05:
        overspeed_reduction_ok = overspeed_candidate <= 0.10
    else:
        overspeed_reduction_ok = overspeed_candidate <= (overspeed_baseline * 0.60)

    accepted = (
        metrics_a.failsafe_events == 0
        and metrics_b.failsafe_events == 0
        and metrics_a.overshoot_max_pct <= 20.0
        and metrics_b.overshoot_max_pct <= 20.0
        and metrics_a.steady_state_rms_error_mps <= 0.35
        and metrics_b.steady_state_rms_error_mps <= 0.35
        and metrics_a.speed_ripple_p95_mps <= 0.45
        and metrics_b.speed_ripple_p95_mps <= 0.45
        and metrics_a.throttle_burst_count_per_min <= 1.0
        and metrics_b.throttle_burst_count_per_min <= 1.0
        and metrics_a.throttle_jerk_p95_pct_per_100ms <= 12.0
        and metrics_b.throttle_jerk_p95_pct_per_100ms <= 12.0
        and metrics_a.brake_delta_p95_pct <= 6.0
        and metrics_b.brake_delta_p95_pct <= 6.0
        and overspeed_reduction_ok
        and score_var < 0.15
    )

    return {
        "accepted": accepted,
        "score_variation": score_var,
        "overspeed_reduction_ok": overspeed_reduction_ok,
        "baseline_overspeed_rate_per_min": overspeed_baseline,
        "candidate_overspeed_rate_per_min": overspeed_candidate,
    }


def run_find_min_throttle(host: str, port: int, timeout: float) -> Dict[str, Any]:
    out_json = "artifacts/min_throttle_hil_report.json"
    cmd = [
        sys.executable,
        "tools/tests/find_min_throttle_hil.py",
        "--host",
        host,
        "--port",
        str(port),
        "--timeout",
        str(timeout),
        "--out-json",
        out_json,
    ]
    try:
        proc = subprocess.run(cmd, check=False, capture_output=True, text=True)
        result: Dict[str, Any] = {
            "exit_code": proc.returncode,
            "stdout_tail": proc.stdout.strip().splitlines()[-8:],
            "stderr_tail": proc.stderr.strip().splitlines()[-8:],
        }
        if os.path.exists(out_json):
            with open(out_json, "r", encoding="utf-8") as fh:
                result["report"] = json.load(fh)
        return result
    except Exception as exc:
        return {"error": str(exc)}


def parse_args(argv: List[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Automatic speed PID calibration using PIDTRACE")
    p.add_argument("--host", default="esp32-salus.local")
    p.add_argument("--port", type=int, default=23)
    p.add_argument("--timeout", type=float, default=5.0)
    p.add_argument("--sample-ms", type=int, default=100)
    p.add_argument("--out-json", default="artifacts/speed_pid_autotune_report.json")
    p.add_argument("--out-csv", default="artifacts/speed_pid_autotune_timeseries.csv")
    p.add_argument("--out-compare", default="artifacts/speed_pid_baseline_vs_candidate.json")
    p.add_argument("--out-failure-json", default="artifacts/speed_pid_failure_diagnostics.json")
    p.add_argument(
        "--hall-report-json",
        default="artifacts/hall_hil_check_report.json",
        help="Ruta al reporte JSON de hall_hil_check.py para gate opcional",
    )
    p.add_argument(
        "--require-hall-ok",
        action="store_true",
        help="Aborta autotune si el reporte Hall previo no existe o hall_diagnostic_ok != true",
    )
    p.add_argument("--progress-interval-s", type=float, default=2.0)
    p.add_argument("--movement-threshold-mps", type=float, default=0.15)
    p.add_argument("--no-motion-grace-s", type=float, default=4.5)
    p.add_argument("--no-motion-min-target-mps", type=float, default=1.4)
    p.add_argument("--no-motion-min-throttle-pct", type=float, default=30.0)
    p.add_argument("--max-consecutive-no-motion", type=int, default=2)
    p.add_argument("--max-valid-speed-mps", type=float, default=8.5)
    p.add_argument(
        "--startup-minthrottle-pct",
        type=float,
        default=0.0,
        help="0=auto from phase A, otherwise force fixed startup minthrottle for campaign",
    )
    return p.parse_args(argv)


def load_hall_gate_status(path: str) -> Dict[str, Any]:
    info: Dict[str, Any] = {
        "path": path,
        "exists": False,
        "loaded": False,
        "hall_diagnostic_ok": None,
        "primary_cause": None,
        "generated_at_utc": None,
        "error": None,
    }
    if not os.path.exists(path):
        info["error"] = "missing_report"
        return info
    info["exists"] = True
    try:
        with open(path, "r", encoding="utf-8") as fh:
            data = json.load(fh)
    except Exception as exc:
        info["error"] = f"load_error:{exc}"
        return info
    summary = data.get("summary", {}) if isinstance(data, dict) else {}
    info["loaded"] = True
    info["hall_diagnostic_ok"] = bool(summary.get("hall_diagnostic_ok")) if "hall_diagnostic_ok" in summary else None
    info["primary_cause"] = summary.get("primary_cause")
    info["generated_at_utc"] = data.get("finished_at_utc") or data.get("generated_at_utc")
    return info


def write_csv(path: str, rows: List[Dict[str, Any]]) -> None:
    ensure_parent(path)
    fields = [
        "profile", "phase", "candidate", "segment", "ts", "target_cmd_mps", "target_raw_mps", "target_mps",
        "speed_mps", "err_mps", "mode", "pid_out_pct", "throttle_raw_pct", "throttle_filt_pct",
        "auto_brake_filt_pct", "fb", "fs", "ovs", "inhibit", "launch", "sat", "iclamp"
    ]
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=fields)
        w.writeheader()
        for r in rows:
            w.writerow(r)


def write_markdown(path: str, report: Dict[str, Any]) -> None:
    ensure_parent(path)
    lines: List[str] = []
    lines.append("# Speed PID Autotune (PIDTRACE)")
    lines.append("")
    lines.append(f"- Start (UTC): {report['started_at_utc']}")
    lines.append(f"- End (UTC): {report['finished_at_utc']}")
    lines.append(f"- Target: `{report['host']}:{report['port']}`")
    lines.append("")
    acc = report.get("acceptance", {})
    lines.append("## Resultado")
    lines.append("")
    lines.append(f"- Accepted: **{acc.get('accepted')}**")
    lines.append(f"- Score variation: `{acc.get('score_variation')}`")
    lines.append(f"- Overspeed reduction ok: `{acc.get('overspeed_reduction_ok')}`")
    if report.get("discard_counters"):
        lines.append(f"- Discard counters: `{report.get('discard_counters')}`")
    rec = report.get("recommended_commands", [])
    lines.append("")
    lines.append("## Recomendación (manual, sin save automático)")
    lines.append("")
    if rec:
        for cmd in rec:
            lines.append(f"- `{cmd}`")
    else:
        lines.append("- N/A")
    lines.append("")
    if report.get("error"):
        lines.append("## Error")
        lines.append("")
        lines.append(f"- {report['error']}")

    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")


def build_failure_diagnostics(report: Dict[str, Any], rows: List[Dict[str, Any]]) -> Dict[str, Any]:
    diag: Dict[str, Any] = {
        "generated_at_utc": now_utc_iso(),
        "error": report.get("error"),
        "discard_counters": report.get("discard_counters", {}),
        "phase_abort_classes": {},
        "signals": {},
        "primary_cause": "UNKNOWN",
        "recommended_next_attempt": [],
        "evidence": [],
    }

    abort_classes: Dict[str, int] = {}
    evidence: List[Dict[str, Any]] = []
    for phase_key in ("phase_b", "phase_c", "phase_d"):
        for entry in report.get(phase_key, []) or []:
            cls = entry.get("abort_class")
            if not cls:
                continue
            abort_classes[cls] = abort_classes.get(cls, 0) + 1
            if len(evidence) < 8:
                evidence.append(
                    {
                        "phase": phase_key,
                        "candidate": entry.get("candidate", {}).get("label"),
                        "abort_class": cls,
                        "abort_reason": entry.get("abort_reason"),
                        "abort_detail": entry.get("abort_detail"),
                    }
                )
    diag["phase_abort_classes"] = abort_classes
    diag["evidence"] = evidence

    total_rows = len(rows)
    fs_count = 0
    fb_bad_count = 0
    inhibit_count = 0
    high_pid_low_speed = 0
    mode_counts: Dict[str, int] = {}
    inhibit_counts: Dict[str, int] = {}

    for r in rows:
        try:
            fs = int(r.get("fs", "0")) == 1
            fb = int(r.get("fb", "1")) == 1
            speed = float(r.get("speed_mps", "0") or 0.0)
            pid_out = float(r.get("pid_out_pct", "0") or 0.0)
        except ValueError:
            continue
        inhibit = r.get("inhibit", "NONE")
        mode = r.get("mode", "UNKNOWN")
        mode_counts[mode] = mode_counts.get(mode, 0) + 1
        inhibit_counts[inhibit] = inhibit_counts.get(inhibit, 0) + 1
        if fs:
            fs_count += 1
        if not fb:
            fb_bad_count += 1
        if inhibit != "NONE":
            inhibit_count += 1
        if pid_out >= 35.0 and speed < 0.15:
            high_pid_low_speed += 1

    signals = {
        "rows": total_rows,
        "fs_ratio": (fs_count / total_rows) if total_rows else 0.0,
        "fb_bad_ratio": (fb_bad_count / total_rows) if total_rows else 0.0,
        "inhibit_ratio": (inhibit_count / total_rows) if total_rows else 0.0,
        "high_pid_low_speed_ratio": (high_pid_low_speed / total_rows) if total_rows else 0.0,
        "mode_counts": mode_counts,
        "inhibit_counts": inhibit_counts,
    }
    diag["signals"] = signals

    if abort_classes.get("NO_MOTION_FAILSAFE", 0) > 0 or signals["fs_ratio"] >= 0.25:
        diag["primary_cause"] = "HALL_FEEDBACK_FAILSAFE"
        diag["recommended_next_attempt"] = [
            "Revisar transiciones Hall/ruido/cableado; confirmar speed.status estable bajo carga.",
            "Aumentar temporalmente launchwin o ajuste de arranque para cruzar umbral de transición antes del grace.",
        ]
    elif abort_classes.get("NO_MOTION_INHIBIT", 0) > 0 or signals["inhibit_ratio"] >= 0.25:
        diag["primary_cause"] = "THROTTLE_INHIBITED_EXTERNALLY"
        diag["recommended_next_attempt"] = [
            "Validar que no haya freno manual activo, Pi fresca o ESTOP durante campaña.",
            "Revisar campo inhibit en PIDTRACE para identificar origen dominante.",
        ]
    elif signals["high_pid_low_speed_ratio"] >= 0.25:
        diag["primary_cause"] = "LOW_THROTTLE_AUTHORITY_OR_ACTUATION"
        diag["recommended_next_attempt"] = [
            "Ajustar minthrottle/minth.spd y revisar respuesta real de PWM/ESC.",
            "Correlacionar pwmDutyPct vs speedMps para detectar falta de tracción o calibración ESC.",
        ]
    elif report.get("acceptance", {}).get("accepted") is False:
        diag["primary_cause"] = "CONTROL_AGGRESSIVE_OR_OSCILLATORY"
        diag["recommended_next_attempt"] = [
            "Reducir thslewup o KI y aumentar suavizado derivativo/freno para bajar burst/jerk.",
        ]

    return diag


def main(argv: List[str]) -> int:
    args = parse_args(argv)
    started = now_utc_iso()
    rows: List[Dict[str, Any]] = []
    report: Dict[str, Any] = {
        "started_at_utc": started,
        "finished_at_utc": None,
        "host": args.host,
        "port": args.port,
        "sample_ms": args.sample_ms,
        "preflight": {},
        "phase_a": {},
        "phase_b": [],
        "phase_c": [],
        "phase_d": [],
        "selected": None,
        "acceptance": {},
    "recommended_commands": [],
    "discard_counters": {
      "NO_MOTION_TRUE": 0,
      "NO_MOTION_FAILSAFE": 0,
      "NO_MOTION_INHIBIT": 0,
      "ESTOP": 0,
      "phase_stop_no_motion": 0,
      "candidates_aborted": 0,
    },
        "run_config": {
            "progress_interval_s": args.progress_interval_s,
            "movement_threshold_mps": args.movement_threshold_mps,
            "no_motion_grace_s": args.no_motion_grace_s,
            "no_motion_min_target_mps": args.no_motion_min_target_mps,
            "no_motion_min_throttle_pct": args.no_motion_min_throttle_pct,
            "max_consecutive_no_motion": args.max_consecutive_no_motion,
            "max_valid_speed_mps": args.max_valid_speed_mps,
            "startup_minthrottle_pct": args.startup_minthrottle_pct,
        },
    }
    hall_gate_info = load_hall_gate_status(args.hall_report_json)
    report["preflight"]["hall_gate"] = hall_gate_info
    if args.require_hall_ok:
        hall_ok = hall_gate_info.get("hall_diagnostic_ok") is True
        if not hall_ok:
            report["finished_at_utc"] = now_utc_iso()
            report["error"] = (
                "Preflight fail: Hall diagnostic gate failed "
                f"(require_hall_ok, cause={hall_gate_info.get('primary_cause')}, report={args.hall_report_json})"
            )
            ensure_parent(args.out_json)
            with open(args.out_json, "w", encoding="utf-8") as fh:
                json.dump(report, fh, ensure_ascii=False, indent=2)
            failure_diag = build_failure_diagnostics(report, rows)
            with open(args.out_failure_json, "w", encoding="utf-8") as fh:
                json.dump(failure_diag, fh, ensure_ascii=False, indent=2)
            write_markdown(os.path.splitext(args.out_json)[0] + ".md", report)
            print(f"Autotune error: {report['error']}")
            print(f"Partial report: {args.out_json}")
            return 4

    search_segments = []
    for target in (0.80, 1.40, 1.94, 2.80, 3.60, 4.17):
        search_segments.append((f"up_{target:.2f}", target, 5.0))
        search_segments.append((f"down_{target:.2f}", 0.00, 3.0))
    search_segments.extend([
        ("disturb_hi", 4.17, 4.0),
        ("disturb_mid", 1.94, 4.0),
        ("disturb_hi2", 4.17, 4.0),
        ("hold_1.94", 1.94, 20.0),
    ])

    validation_segments = []
    for target in (0.80, 1.40, 1.94, 2.80, 3.60, 4.17):
        validation_segments.append((f"up_{target:.2f}", target, 8.0))
        validation_segments.append((f"down_{target:.2f}", 0.00, 4.0))
    validation_segments.extend([
        ("disturb_hi", 4.17, 6.0),
        ("disturb_mid", 1.94, 6.0),
        ("disturb_hi2", 4.17, 6.0),
        ("hold_1.94", 1.94, 60.0),
    ])

    # Phase A uses an independent Telnet client; run it before opening the
    # persistent autotune session because firmware accepts only one client.
    log_progress("Fase A: midiendo movimiento mínimo (find_min_throttle_hil)")
    report["phase_a"] = run_find_min_throttle(args.host, args.port, args.timeout)
    phase_a_report = report["phase_a"].get("report", {})
    startup_min_throttle_pct = 35.0
    phase_a_min_pid_f = 35.0
    if isinstance(phase_a_report, dict):
        min_pid = phase_a_report.get("min_effective_pidOut_pct")
        if isinstance(min_pid, (int, float)):
            min_pid_f = float(min_pid)
            phase_a_min_pid_f = min_pid_f
            startup_min_throttle_pct = clamp_value(max(min_pid_f, min_pid_f * 1.10), 25.0, 70.0)
        log_progress(
            "Fase A resultado: movement_detected="
            f"{phase_a_report.get('movement_detected')} "
            f"peak={phase_a_report.get('max_speed_drive_mps')}m/s"
        )
    if args.startup_minthrottle_pct > 0.0:
        startup_min_throttle_pct = clamp_value(args.startup_minthrottle_pct, 5.0, 90.0)
    report["phase_a"]["startup_minthrottle_pct"] = round(startup_min_throttle_pct, 2)
    phase_a_report = report["phase_a"].get("report", {})
    if isinstance(phase_a_report, dict) and phase_a_report.get("movement_detected") is False:
        report["finished_at_utc"] = now_utc_iso()
        report["error"] = (
            "Abortado: fase A detectó NO_MOTION (ruedas no se mueven). "
            "No se ejecutan barridos inútiles."
        )
        ensure_parent(args.out_json)
        with open(args.out_json, "w", encoding="utf-8") as fh:
            json.dump(report, fh, ensure_ascii=False, indent=2)
        failure_diag = build_failure_diagnostics(report, rows)
        with open(args.out_failure_json, "w", encoding="utf-8") as fh:
            json.dump(failure_diag, fh, ensure_ascii=False, indent=2)
        write_markdown(os.path.splitext(args.out_json)[0] + ".md", report)
        log_progress("Abortado temprano por NO_MOTION en fase A")
        return 3

    tn = TelnetRunner(args.host, args.port, args.timeout)
    original_spid: Dict[str, Any] = {}

    try:
        banner = tn.connect()
        log_progress("Telnet conectado, iniciando preflight")
        pre_events: List[Tuple[float, str]] = []
        for cmd in ("net.status", "comms.status", "spid.status", "speed.status"):
            pre_events.extend(tn.send(cmd, wait_s=0.40))

        comms_line = choose_last(pre_events, "[PI][STATUS]")
        spid_line = choose_last(pre_events, "[SPID] state{")
        speed_line = choose_last(pre_events, "[SPD][STATUS]")

        comms = parse_comms_status(comms_line) if comms_line else {}
        spid = parse_spid_status(spid_line) if spid_line else {}
        speed = parse_speed_status(speed_line) if speed_line else {}
        report["preflight"] = {
            "banner_tail": [x[1] for x in banner[-3:]],
            "comms": comms,
            "spid": spid,
            "speed": speed,
        }
        original_spid = dict(spid)

        if comms.get("piFresh", False):
            raise RuntimeError("Preflight fail: Pi fresca (RC/Telnet no serán fuente principal)")
        if comms.get("estop") is True or (comms.get("brake") or 0) > 0:
            raise RuntimeError("Preflight fail: ESTOP o brake activo")
        if not spid.get("init", False):
            raise RuntimeError("Preflight fail: speed PID no inicializado")
        if speed.get("driver") != "READY":
            raise RuntimeError("Preflight fail: Hall driver no READY")

        log_progress("Preflight OK: arrancando fases B/C/D")

        for cmd in (
            "spid.reset",
            "spid.max 4.17",
            "spid.ramp 2.0",
            "spid.set 10 2 0",
        f"spid.minthrottle {startup_min_throttle_pct:.2f}",
        "spid.thslewup 30",
            "spid.thslewdown 45",
            "spid.minth.spd 0.35",
            "spid.launchwin 1200",
            "spid.iunwind 0.35",
            "spid.dfilter 3.0",
            "speed.reset",
            f"spid.stream on {args.sample_ms}",
            "drive.log on",
            "drive.log pid on 100",
        ):
            tn.send(cmd, wait_s=0.25)

        startup_slew = clamp_value(max(45.0, phase_a_min_pid_f * 1.4), 45.0, 85.0)
        startup_slew_fast = clamp_value(max(55.0, phase_a_min_pid_f * 1.7), 55.0, 95.0)
        structure_candidates = [
            # Candidatos front-loaded para superar el umbral de arranque Hall.
            Structure(startup_slew_fast, clamp_value(startup_slew_fast + 15.0, 65.0, 100.0), 0.45, 2600, 0.40, 2.8),
            Structure(startup_slew, clamp_value(startup_slew + 12.0, 55.0, 95.0), 0.40, 2200, 0.35, 3.0),
            Structure(35.0, 55.0, 0.35, 1800, 0.30, 3.5),
            Structure(30.0, 45.0, 0.35, 1200, 0.35, 3.0),
            Structure(24.0, 36.0, 0.35, 1300, 0.45, 3.0),
        ]

        # Phase B: structure sweep with baseline tunings
        phase_b_results: List[RunResult] = []
        consecutive_no_motion_b = 0
        for i, struct_cfg in enumerate(structure_candidates):
            log_progress(f"Fase B candidato {i+1}/{len(structure_candidates)}")
            cand = Candidate(
                phase="B",
                label=f"B_struct_{i+1}",
                tunings=Tunings(10.0, 2.0, 0.0),
                structure=struct_cfg,
            )
            res = run_profile(
                tn,
                cand,
                "search",
                search_segments,
                rows,
                progress_interval_s=args.progress_interval_s,
                movement_threshold_mps=args.movement_threshold_mps,
                no_motion_grace_s=args.no_motion_grace_s,
                no_motion_min_target_mps=args.no_motion_min_target_mps,
                no_motion_min_throttle_pct=args.no_motion_min_throttle_pct,
                max_valid_speed_mps=args.max_valid_speed_mps,
            )
            phase_b_results.append(res)
            report["phase_b"].append(
                {
                    "candidate": asdict(cand),
                    "metrics": asdict(res.metrics),
                    "abort_reason": res.abort_reason,
                    "abort_class": res.abort_class,
                    "abort_detail": res.abort_detail,
                }
            )
            if res.abort_class in report["discard_counters"]:
                report["discard_counters"][res.abort_class] += 1
            if res.abort_class is not None:
                report["discard_counters"]["candidates_aborted"] += 1
            if res.abort_class in ("NO_MOTION_TRUE", "NO_MOTION_FAILSAFE", "NO_MOTION_INHIBIT"):
                consecutive_no_motion_b += 1
                log_progress(
                    f"Fase B {cand.label} abortado por {res.abort_class} "
                    f"({consecutive_no_motion_b}/{args.max_consecutive_no_motion})"
                )
                if consecutive_no_motion_b >= args.max_consecutive_no_motion:
                    log_progress("Fase B detenida por NO_MOTION consecutivo")
                    report["discard_counters"]["phase_stop_no_motion"] += 1
                    break
            else:
                consecutive_no_motion_b = 0

        if phase_b_results and all(
            r.abort_class in ("NO_MOTION_TRUE", "NO_MOTION_FAILSAFE", "NO_MOTION_INHIBIT")
            for r in phase_b_results
        ):
            raise RuntimeError("Fase B abortada: todos los candidatos quedaron en NO_MOTION")

        baseline = phase_b_results[0].metrics
        valid_b = [r for r in phase_b_results if r.abort_reason is None]
        if not valid_b:
            valid_b = list(phase_b_results)
        if not valid_b:
            raise RuntimeError("Fase B sin candidatos válidos")
        best_structure = min(valid_b, key=lambda r: r.metrics.score).candidate.structure

        # Phase C: gain sweep with selected structure
        gain_candidates = [
            Tunings(6.0, 0.8, 0.0),
            Tunings(8.0, 1.0, 0.0),
            Tunings(10.0, 1.2, 0.0),
            Tunings(12.0, 1.4, 0.0),
            Tunings(10.0, 1.6, 0.02),
            Tunings(12.0, 1.6, 0.02),
            Tunings(14.0, 1.8, 0.02),
            Tunings(10.0, 2.0, 0.05),
            Tunings(12.0, 2.0, 0.05),
        ]

        phase_c_results: List[RunResult] = []
        consecutive_no_motion_c = 0
        for i, gains in enumerate(gain_candidates):
            log_progress(f"Fase C candidato {i+1}/{len(gain_candidates)}")
            cand = Candidate(
                phase="C",
                label=f"C_gain_{i+1}",
                tunings=gains,
                structure=best_structure,
            )
            res = run_profile(
                tn,
                cand,
                "search",
                search_segments,
                rows,
                progress_interval_s=args.progress_interval_s,
                movement_threshold_mps=args.movement_threshold_mps,
                no_motion_grace_s=args.no_motion_grace_s,
                no_motion_min_target_mps=args.no_motion_min_target_mps,
                no_motion_min_throttle_pct=args.no_motion_min_throttle_pct,
                max_valid_speed_mps=args.max_valid_speed_mps,
            )
            phase_c_results.append(res)
            report["phase_c"].append(
                {
                    "candidate": asdict(cand),
                    "metrics": asdict(res.metrics),
                    "abort_reason": res.abort_reason,
                    "abort_class": res.abort_class,
                    "abort_detail": res.abort_detail,
                }
            )
            if res.abort_class in report["discard_counters"]:
                report["discard_counters"][res.abort_class] += 1
            if res.abort_class is not None:
                report["discard_counters"]["candidates_aborted"] += 1
            if res.abort_class in ("NO_MOTION_TRUE", "NO_MOTION_FAILSAFE", "NO_MOTION_INHIBIT"):
                consecutive_no_motion_c += 1
                log_progress(
                    f"Fase C {cand.label} abortado por {res.abort_class} "
                    f"({consecutive_no_motion_c}/{args.max_consecutive_no_motion})"
                )
                if consecutive_no_motion_c >= args.max_consecutive_no_motion:
                    log_progress("Fase C detenida por NO_MOTION consecutivo")
                    report["discard_counters"]["phase_stop_no_motion"] += 1
                    break
            else:
                consecutive_no_motion_c = 0

        if phase_c_results and all(
            r.abort_class in ("NO_MOTION_TRUE", "NO_MOTION_FAILSAFE", "NO_MOTION_INHIBIT")
            for r in phase_c_results
        ):
            raise RuntimeError("Fase C abortada: todos los candidatos quedaron en NO_MOTION")

        valid_c = [r for r in phase_c_results if r.abort_reason is None]
        if not valid_c:
            valid_c = list(phase_c_results)
        if not valid_c:
            raise RuntimeError("Fase C sin candidatos válidos")
        selected = min(valid_c, key=lambda r: r.metrics.score).candidate
        report["selected"] = asdict(selected)

        # Phase D: validation x2 (full profile)
        d_runs: List[RunResult] = []
        for run_idx in range(2):
            log_progress(f"Fase D validación {run_idx+1}/2")
            val_cand = Candidate(
                phase="D",
                label=f"D_val_{run_idx+1}",
                tunings=selected.tunings,
                structure=selected.structure,
            )
            res = run_profile(
                tn,
                val_cand,
                "validation",
                validation_segments,
                rows,
                progress_interval_s=args.progress_interval_s,
                movement_threshold_mps=args.movement_threshold_mps,
                no_motion_grace_s=args.no_motion_grace_s,
                no_motion_min_target_mps=args.no_motion_min_target_mps,
                no_motion_min_throttle_pct=args.no_motion_min_throttle_pct,
                max_valid_speed_mps=args.max_valid_speed_mps,
            )
            d_runs.append(res)
            report["phase_d"].append(
                {
                    "candidate": asdict(val_cand),
                    "metrics": asdict(res.metrics),
                    "abort_reason": res.abort_reason,
                    "abort_class": res.abort_class,
                    "abort_detail": res.abort_detail,
                }
            )
            if res.abort_class in report["discard_counters"]:
                report["discard_counters"][res.abort_class] += 1
            if res.abort_class is not None:
                report["discard_counters"]["candidates_aborted"] += 1

        if any(r.abort_reason for r in d_runs):
            report["acceptance"] = {"accepted": False, "reason": "Abort en validación"}
        else:
            acc = acceptance_check(d_runs[0].metrics, d_runs[1].metrics, baseline)
            report["acceptance"] = acc

        recommendation = [
            f"spid.set {selected.tunings.kp:.3f} {selected.tunings.ki:.3f} {selected.tunings.kd:.3f}",
            f"spid.thslewup {selected.structure.thslewup:.3f}",
            f"spid.thslewdown {selected.structure.thslewdown:.3f}",
            f"spid.minth.spd {selected.structure.minth_spd:.3f}",
            f"spid.launchwin {selected.structure.launchwin_ms}",
            f"spid.iunwind {selected.structure.iunwind:.3f}",
            f"spid.dfilter {selected.structure.dfilter_hz:.3f}",
        ]
        report["recommended_commands"] = recommendation

        # Baseline vs candidate comparison artifact
        candidate_avg = RunMetrics(
            score=safe_mean([r.metrics.score for r in d_runs]),
            overshoot_max_pct=max(r.metrics.overshoot_max_pct for r in d_runs),
            settling_time_5pct_max_s=max(
                [x for x in [r.metrics.settling_time_5pct_max_s for r in d_runs] if x is not None],
                default=None,
            ),
            steady_state_rms_error_mps=max(r.metrics.steady_state_rms_error_mps for r in d_runs),
            speed_ripple_p95_mps=max(r.metrics.speed_ripple_p95_mps for r in d_runs),
            throttle_jerk_p95_pct_per_100ms=max(r.metrics.throttle_jerk_p95_pct_per_100ms for r in d_runs),
            throttle_burst_count_per_min=max(r.metrics.throttle_burst_count_per_min for r in d_runs),
            brake_delta_p95_pct=max(r.metrics.brake_delta_p95_pct for r in d_runs),
            overspeed_transition_rate_per_min=safe_mean([r.metrics.overspeed_transition_rate_per_min for r in d_runs]),
            failsafe_events=sum(r.metrics.failsafe_events for r in d_runs),
            inhibit_toggle_rate_per_min=safe_mean([r.metrics.inhibit_toggle_rate_per_min for r in d_runs]),
        )
        compare = {
            "baseline": asdict(baseline),
            "candidate": asdict(candidate_avg),
            "delta": {
                "overspeed_transition_rate_per_min": candidate_avg.overspeed_transition_rate_per_min - baseline.overspeed_transition_rate_per_min,
                "speed_ripple_p95_mps": candidate_avg.speed_ripple_p95_mps - baseline.speed_ripple_p95_mps,
                "throttle_jerk_p95_pct_per_100ms": candidate_avg.throttle_jerk_p95_pct_per_100ms - baseline.throttle_jerk_p95_pct_per_100ms,
                "throttle_burst_count_per_min": candidate_avg.throttle_burst_count_per_min - baseline.throttle_burst_count_per_min,
                "brake_delta_p95_pct": candidate_avg.brake_delta_p95_pct - baseline.brake_delta_p95_pct,
            },
        }

        report["finished_at_utc"] = now_utc_iso()
        ensure_parent(args.out_json)
        with open(args.out_json, "w", encoding="utf-8") as fh:
            json.dump(report, fh, ensure_ascii=False, indent=2)
        with open(args.out_compare, "w", encoding="utf-8") as fh:
            json.dump(compare, fh, ensure_ascii=False, indent=2)
        failure_diag = build_failure_diagnostics(report, rows)
        with open(args.out_failure_json, "w", encoding="utf-8") as fh:
            json.dump(failure_diag, fh, ensure_ascii=False, indent=2)
        write_csv(args.out_csv, rows)
        write_markdown(os.path.splitext(args.out_json)[0] + ".md", report)

        print("=== AUTOTUNE RESULT ===")
        print(f"Accepted: {report['acceptance'].get('accepted')}")
        print(f"Report JSON: {args.out_json}")
        print(f"Compare JSON: {args.out_compare}")
        print(f"Failure diagnostics JSON: {args.out_failure_json}")
        print(f"Timeseries CSV: {args.out_csv}")

        return 0 if report["acceptance"].get("accepted", False) else 2

    except Exception as exc:
        report["finished_at_utc"] = now_utc_iso()
        report["error"] = str(exc)
        ensure_parent(args.out_json)
        with open(args.out_json, "w", encoding="utf-8") as fh:
            json.dump(report, fh, ensure_ascii=False, indent=2)
        failure_diag = build_failure_diagnostics(report, rows)
        with open(args.out_failure_json, "w", encoding="utf-8") as fh:
            json.dump(failure_diag, fh, ensure_ascii=False, indent=2)
        write_markdown(os.path.splitext(args.out_json)[0] + ".md", report)
        if rows:
            write_csv(args.out_csv, rows)
        print(f"Autotune error: {exc}")
        print(f"Partial report: {args.out_json}")
        return 1

    finally:
        try:
            tn.send("spid.target 0.000", wait_s=0.1)
            tn.send("spid.target off", wait_s=0.1)
            if original_spid:
                if all(k in original_spid for k in ("kp", "ki", "kd")):
                    tn.send(
                        f"spid.set {original_spid['kp']:.6f} {original_spid['ki']:.6f} {original_spid['kd']:.6f}",
                        wait_s=0.2,
                    )
                if "max" in original_spid:
                    tn.send(f"spid.max {original_spid['max']:.3f}", wait_s=0.15)
                if "ramp" in original_spid:
                    tn.send(f"spid.ramp {original_spid['ramp']:.3f}", wait_s=0.15)
                if "tmin" in original_spid:
                    tn.send(f"spid.minthrottle {original_spid['tmin']:.3f}", wait_s=0.15)
                if "thsup" in original_spid:
                    tn.send(f"spid.thslewup {original_spid['thsup']:.3f}", wait_s=0.15)
                if "thsdown" in original_spid:
                    tn.send(f"spid.thslewdown {original_spid['thsdown']:.3f}", wait_s=0.15)
                if "minspd" in original_spid:
                    tn.send(f"spid.minth.spd {original_spid['minspd']:.3f}", wait_s=0.15)
                if "lwin" in original_spid:
                    tn.send(f"spid.launchwin {int(round(original_spid['lwin']))}", wait_s=0.15)
                if "iunw" in original_spid:
                    tn.send(f"spid.iunwind {original_spid['iunw']:.3f}", wait_s=0.15)
                if "dfhz" in original_spid:
                    tn.send(f"spid.dfilter {original_spid['dfhz']:.3f}", wait_s=0.15)
                if "cap" in original_spid:
                    tn.send(f"spid.brakecap {original_spid['cap']:.3f}", wait_s=0.15)
                if "hys" in original_spid:
                    tn.send(f"spid.hys {original_spid['hys']:.3f}", wait_s=0.15)
            tn.send("spid.stream off", wait_s=0.1)
            tn.send("drive.log pid off", wait_s=0.1)
            tn.send("drive.log off", wait_s=0.1)
        except Exception:
            pass
        tn.close()


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
