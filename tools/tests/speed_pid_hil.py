#!/usr/bin/env python3
"""HIL test runner for ESP32 speed PID with RC fallback focus.

This tool drives a Telnet session against the ESP32 firmware, collects
`[DRIVE]` and `[SPID]` logs, and evaluates a test checklist (T0..T6).

Default mode is interactive because several checks require manual RC actions.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
import telnetlib
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional, Tuple


RE_DRIVE_TOKENS = re.compile(r"([A-Za-z]+)=([^\s]+)")
RE_SPID_STATE = re.compile(
    r"state\{init=(?P<init>[YN]) en=(?P<en>[YN]) fb=(?P<fb>[YN]) "
    r"failsafe=(?P<failsafe>[YN]) overspeed=(?P<overspeed>[YN]) mode=(?P<mode>[A-Z]+)\}"
)
RE_COMMS_LAST_FRAME = re.compile(r"lastFrame=(NONE|\d+ms)")


@dataclass
class TestCaseResult:
    case_id: str
    title: str
    status: str = "skip"  # pass|fail|skip
    summary: str = ""
    checks: List[Dict[str, Any]] = field(default_factory=list)
    evidence: List[str] = field(default_factory=list)


@dataclass
class Report:
    started_at_utc: str
    finished_at_utc: str = ""
    host: str = ""
    port: int = 23
    sample_ms: int = 200
    mode: str = "interactive"
    preflight: Dict[str, Any] = field(default_factory=dict)
    cases: List[TestCaseResult] = field(default_factory=list)
    diagnostics: List[str] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "started_at_utc": self.started_at_utc,
            "finished_at_utc": self.finished_at_utc,
            "host": self.host,
            "port": self.port,
            "sample_ms": self.sample_ms,
            "mode": self.mode,
            "preflight": self.preflight,
            "cases": [
                {
                    "id": c.case_id,
                    "title": c.title,
                    "status": c.status,
                    "summary": c.summary,
                    "checks": c.checks,
                    "evidence": c.evidence,
                }
                for c in self.cases
            ],
            "diagnostics": self.diagnostics,
        }


class TelnetRunner:
    def __init__(self, host: str, port: int, timeout: float):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.tn: Optional[telnetlib.Telnet] = None

    def connect(self) -> List[str]:
        self.tn = telnetlib.Telnet(self.host, self.port, timeout=self.timeout)
        time.sleep(0.6)
        return self.read_lines()

    def close(self) -> None:
        if self.tn is None:
            return
        try:
            self.tn.write(b"exit\n")
            time.sleep(0.1)
        except Exception:
            pass
        try:
            self.tn.close()
        except Exception:
            pass
        self.tn = None

    def send(self, cmd: str, wait_s: float = 0.45) -> List[str]:
        if self.tn is None:
            raise RuntimeError("Telnet is not connected")
        self.tn.write((cmd + "\n").encode("utf-8"))
        time.sleep(wait_s)
        return self.read_lines()

    def read_lines(self) -> List[str]:
        if self.tn is None:
            return []
        try:
            payload = self.tn.read_very_eager()
        except EOFError:
            return []
        if not payload:
            return []
        decoded = payload.decode("utf-8", errors="ignore")
        return [ln.strip() for ln in decoded.splitlines() if ln.strip()]

    def collect(self, duration_s: float, poll_s: float = 0.1) -> List[str]:
        lines: List[str] = []
        end = time.time() + duration_s
        while time.time() < end:
            lines.extend(self.read_lines())
            time.sleep(poll_s)
        lines.extend(self.read_lines())
        return lines


def to_float(value: str) -> Optional[float]:
    cleaned = value.strip()
    for suffix in ("m/s2", "m/s", "%", "deg", "ms"):
        if cleaned.endswith(suffix):
            cleaned = cleaned[: -len(suffix)]
            break
    cleaned = cleaned.strip().rstrip(",")
    if not cleaned:
        return None
    try:
        return float(cleaned)
    except ValueError:
        return None


def to_int(value: str) -> Optional[int]:
    cleaned = value.strip().rstrip(",")
    if "/" in cleaned:
        cleaned = cleaned.split("/", 1)[0]
    try:
        return int(cleaned)
    except ValueError:
        f = to_float(cleaned)
        if f is None:
            return None
        return int(round(f))


def parse_bool_yn(value: Optional[str]) -> Optional[bool]:
    if value is None:
        return None
    if value == "Y":
        return True
    if value == "N":
        return False
    return None


def parse_drive_line(line: str) -> Dict[str, Any]:
    data: Dict[str, Any] = {"raw_line": line}
    for key, val in RE_DRIVE_TOKENS.findall(line):
        data[key] = val

    parsed: Dict[str, Any] = {"raw_line": line}
    parsed["src"] = data.get("src")
    parsed["mode"] = data.get("mode")
    parsed["fb"] = parse_bool_yn(data.get("fb"))
    parsed["fs"] = parse_bool_yn(data.get("fs"))
    parsed["ovs"] = parse_bool_yn(data.get("ovs"))

    int_fields = ["cmd", "duty", "brake", "ageMs"]
    for fld in int_fields:
        if fld in data:
            parsed[fld] = to_int(data[fld])

    float_fields = [
        "targetRaw",
        "target",
        "speed",
        "pidOut",
        "autoBrake",
        "err",
        "over",
        "targetRawMps",
        "targetMps",
        "speedMps",
        "pidOutPct",
        "autoBrakeRawPct",
        "autoBrakeFiltPct",
        "errMps",
        "throttleRawPct",
        "throttleFiltPct",
    ]
    for fld in float_fields:
        if fld in data:
            parsed[fld] = to_float(data[fld])

    # Normalize legacy and PIDTRACE key names.
    if "targetRaw" not in parsed and "targetRawMps" in parsed:
        parsed["targetRaw"] = parsed["targetRawMps"]
    if "target" not in parsed and "targetMps" in parsed:
        parsed["target"] = parsed["targetMps"]
    if "speed" not in parsed and "speedMps" in parsed:
        parsed["speed"] = parsed["speedMps"]
    if "pidOut" not in parsed and "pidOutPct" in parsed:
        parsed["pidOut"] = parsed["pidOutPct"]
    if "autoBrake" not in parsed and "autoBrakeFiltPct" in parsed:
        parsed["autoBrake"] = parsed["autoBrakeFiltPct"]
    if "err" not in parsed and "errMps" in parsed:
        parsed["err"] = parsed["errMps"]

    parsed["launchAssistActive"] = parse_bool_yn(data.get("launchAssistActive"))
    parsed["throttleSaturated"] = parse_bool_yn(data.get("throttleSaturated"))
    parsed["integratorClamped"] = parse_bool_yn(data.get("integratorClamped"))

    if "raw" in data:
        parsed["raw"] = None if data["raw"] == "STALE" else to_int(data["raw"])
        parsed["raw_state"] = data["raw"]

    return parsed


def parse_spid_line(line: str) -> Dict[str, Any]:
    parsed: Dict[str, Any] = {"raw_line": line}
    m = RE_SPID_STATE.search(line)
    if m:
        parsed["init"] = parse_bool_yn(m.group("init"))
        parsed["en"] = parse_bool_yn(m.group("en"))
        parsed["fb"] = parse_bool_yn(m.group("fb"))
        parsed["failsafe"] = parse_bool_yn(m.group("failsafe"))
        parsed["overspeed"] = parse_bool_yn(m.group("overspeed"))
        parsed["mode"] = m.group("mode")

    for key in ["targetRaw", "target", "speed", "err", "overErr", "throttle", "brake"]:
        mm = re.search(r"\b" + re.escape(key) + r"=([^\s]+)", line)
        if mm:
            parsed[key] = to_float(mm.group(1))

    mm_max = re.search(r"\bmax=([^\s]+)", line)
    if mm_max:
        parsed["max"] = to_float(mm_max.group(1))

    return parsed


def parse_comms_line(line: str) -> Dict[str, Any]:
    parsed: Dict[str, Any] = {"raw_line": line}
    m = RE_COMMS_LAST_FRAME.search(line)
    if m:
        lf = m.group(1)
        parsed["lastFrame"] = lf
        if lf == "NONE":
            parsed["lastFrameMs"] = None
            parsed["piFresh"] = False
        else:
            ms = int(lf[:-2])
            parsed["lastFrameMs"] = ms
            parsed["piFresh"] = ms <= 120

    for flag in ("estop", "drive"):
        mm = re.search(r"\b" + flag + r"=([YN])", line)
        if mm:
            parsed[flag] = parse_bool_yn(mm.group(1))

    mm_brake = re.search(r"\bbrake=(\d+)", line)
    if mm_brake:
        parsed["brake"] = int(mm_brake.group(1))

    return parsed


def parse_speed_status(line: str) -> Dict[str, Any]:
    parsed: Dict[str, Any] = {"raw_line": line}
    mm = re.search(r"\bdriver=([A-Z_]+)", line)
    if mm:
        parsed["driver"] = mm.group(1)
    mm_speed = re.search(r"\bspeed=([0-9.]+)km/h", line)
    if mm_speed:
        parsed["speedKmh"] = float(mm_speed.group(1))
    return parsed


def now_utc_iso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def choose_last(lines: List[str], prefix: str) -> Optional[str]:
    for line in reversed(lines):
        if line.startswith(prefix):
            return line
    return None


def collect_drive(lines: List[str]) -> List[Dict[str, Any]]:
    return [parse_drive_line(ln) for ln in lines if ln.startswith("[DRIVE]")]


def collect_spid(lines: List[str]) -> List[Dict[str, Any]]:
    return [parse_spid_line(ln) for ln in lines if ln.startswith("[SPID]")]


def summarize_status(cases: List[TestCaseResult]) -> Dict[str, int]:
    out = {"pass": 0, "fail": 0, "skip": 0}
    for c in cases:
        out[c.status] = out.get(c.status, 0) + 1
    return out


def prompt_step(text: str, allow_skip: bool = True) -> str:
    hint = "[Enter=ok"
    if allow_skip:
        hint += ", s=skip"
    hint += ", q=abort]"
    while True:
        ans = input(f"\n{text}\n{hint}: ").strip().lower()
        if ans == "q":
            return "abort"
        if allow_skip and ans == "s":
            return "skip"
        if ans == "":
            return "ok"
        print("Entrada inválida. Usa Enter, s o q.")


def approx_equal(actual: Optional[float], expected: float, tol: float) -> bool:
    if actual is None:
        return False
    return abs(actual - expected) <= tol


def build_diagnostics(
    baseline_comms: Optional[Dict[str, Any]],
    all_drive: List[Dict[str, Any]],
    cases: List[TestCaseResult],
) -> List[str]:
    diagnostics: List[str] = []

    any_case_fail = any(c.status == "fail" for c in cases)

    pi_fresh_now = bool(baseline_comms and baseline_comms.get("piFresh"))
    if pi_fresh_now:
        diagnostics.append(
            "`comms.status` muestra Pi fresca (<=120 ms): por diseño actual RC-PID no tomará control mientras Pi esté activa."
        )

    stale_count = sum(1 for d in all_drive if d.get("raw_state") == "STALE")
    if stale_count > 0:
        diagnostics.append(
            "Se observaron muestras `raw=STALE` en `[DRIVE]`: posible problema de frescura RC (`rcFresh`) o pérdida de señal RMT."
        )

    rc_fb_failsafe = [d for d in all_drive if d.get("src") == "RC" and d.get("fb") is False and d.get("fs") is True]
    if rc_fb_failsafe:
        diagnostics.append(
            "Se detectó `src=RC` con `fb=N fs=Y`: el PID está entrando en fail-safe por feedback Hall no válido o sin transiciones frescas."
        )

    rc_pidout_positive = [d for d in all_drive if d.get("src") == "RC" and (d.get("pidOut") or 0.0) > 0.5]
    if rc_pidout_positive:
        # Detect likely duty stall: repeated same duty with positive pidOut
        by_duty = {}
        for d in rc_pidout_positive:
            duty = d.get("duty")
            by_duty[duty] = by_duty.get(duty, 0) + 1
        max_repeats = max(by_duty.values()) if by_duty else 0
        if max_repeats >= 3:
            diagnostics.append(
                "`src=RC` con `pidOut>0` y duty repetido sugiere inhibición/umbral PWM o saturación en la ruta de acelerador."
            )

    brake_active_many = [d for d in all_drive if (d.get("brake") or 0) > 0]
    if len(brake_active_many) >= 3:
        diagnostics.append(
            "`brake>0` sostenido en `[DRIVE]`: el freno (manual RC, Pi o auto-overspeed) puede estar inhibiendo throttle."
        )

    if not diagnostics:
        if any_case_fail:
            diagnostics.append("Hay fallas en casos de test pero no se detectó una causa dominante automática; revisar evidencia por caso.")
        else:
            diagnostics.append("No se detectaron condiciones anómalas dominantes en el árbol de diagnóstico.")

    return diagnostics


def ensure_parent(path: str) -> None:
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)


def write_markdown(report: Report, path: str) -> None:
    status = summarize_status(report.cases)
    lines: List[str] = []
    lines.append("# Speed PID HIL Test Report")
    lines.append("")
    lines.append(f"- Start (UTC): {report.started_at_utc}")
    lines.append(f"- End (UTC): {report.finished_at_utc}")
    lines.append(f"- Target: `{report.host}:{report.port}`")
    lines.append(f"- Mode: `{report.mode}`")
    lines.append(f"- Sample: `{report.sample_ms} ms`")
    lines.append("")
    lines.append("## Summary")
    lines.append("")
    lines.append(f"- PASS: {status.get('pass', 0)}")
    lines.append(f"- FAIL: {status.get('fail', 0)}")
    lines.append(f"- SKIP: {status.get('skip', 0)}")
    lines.append("")
    lines.append("## Cases")
    lines.append("")
    for case in report.cases:
        lines.append(f"### {case.case_id} - {case.title}")
        lines.append("")
        lines.append(f"- Status: **{case.status.upper()}**")
        lines.append(f"- Summary: {case.summary}")
        if case.checks:
            lines.append("- Checks:")
            for chk in case.checks:
                lines.append(f"  - {chk.get('name', 'check')}: {chk.get('status', '-')}")
        if case.evidence:
            lines.append("- Evidence:")
            for ev in case.evidence[:8]:
                lines.append(f"  - `{ev}`")
        lines.append("")

    lines.append("## Diagnostics")
    lines.append("")
    for d in report.diagnostics:
        lines.append(f"- {d}")

    ensure_parent(path)
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")


def run(args: argparse.Namespace) -> int:
    report = Report(
        started_at_utc=now_utc_iso(),
        host=args.host,
        port=args.port,
        sample_ms=args.sample_ms,
        mode=args.mode,
    )

    all_captured_lines: List[str] = []

    tn = TelnetRunner(args.host, args.port, args.timeout)
    try:
        banner_lines = tn.connect()
        all_captured_lines.extend(banner_lines)

        # Preflight snapshot
        preflight_lines: List[str] = []
        for cmd in ["net.status", "comms.status", "spid.status", "speed.status", "drive.log"]:
            preflight_lines.extend(tn.send(cmd))

        all_captured_lines.extend(preflight_lines)
        comms_line = choose_last(preflight_lines, "[PI][STATUS]")
        spid_line = choose_last(preflight_lines, "[SPID]")
        speed_line = choose_last(preflight_lines, "[SPD][STATUS]")

        baseline_comms = parse_comms_line(comms_line) if comms_line else None
        baseline_spid = parse_spid_line(spid_line) if spid_line else None
        baseline_speed = parse_speed_status(speed_line) if speed_line else None

        report.preflight = {
            "banner": banner_lines[-3:],
            "comms": baseline_comms,
            "spid": baseline_spid,
            "speed": baseline_speed,
        }

        # Normalization commands
        norm_lines: List[str] = []
        for cmd in [
            "spid.reset",
            "spid.max 4.17",
            "spid.ramp 2.0",
            "spid.set 10 2 0",
            "drive.log on",
            f"spid.stream on {args.sample_ms}",
        ]:
            norm_lines.extend(tn.send(cmd, wait_s=0.6))
        all_captured_lines.extend(norm_lines)

        # T0
        t0 = TestCaseResult("T0", "Baseline de control")
        max_ok = bool(baseline_spid and baseline_spid.get("init") is True and approx_equal(baseline_spid.get("max"), 4.17, 0.05))
        hall_ready = bool(baseline_speed and baseline_speed.get("driver") == "READY")
        t0.checks.append({"name": "SPID initialized + max=4.17", "status": "pass" if max_ok else "fail"})
        t0.checks.append({"name": "Hall driver READY", "status": "pass" if hall_ready else "fail"})
        if max_ok and hall_ready:
            t0.status = "pass"
            t0.summary = "Estado base correcto para iniciar pruebas de PID de velocidad."
        else:
            t0.status = "fail"
            t0.summary = "Baseline incompleto: revisar inicialización SPID o backend Hall."
        for ln in [comms_line, spid_line, speed_line]:
            if ln:
                t0.evidence.append(ln)
        report.cases.append(t0)

        interactive = args.mode == "interactive"
        quick_auto = args.mode == "quick"

        # Probe support for telnet speed-target override
        target_probe_lines = tn.send("spid.target", wait_s=0.3)
        all_captured_lines.extend(target_probe_lines)
        has_target_cmd = any(ln.startswith("[SPID][TARGET]") for ln in target_probe_lines)

        # T1
        t1 = TestCaseResult("T1", "Arbitraje PI/RC")
        if interactive:
            ans = prompt_step(
                "T1-A: Asegura Pi sin frames frescos (detener envío de Raspberry). Luego mueve RC throttle ~50 y manten 3s.",
                allow_skip=False,
            )
            if ans == "abort":
                raise KeyboardInterrupt
            captured = tn.collect(3.0)
            all_captured_lines.extend(captured)
            drive_entries = collect_drive(captured)
            saw_src_rc = any(d.get("src") == "RC" for d in drive_entries)
            current_comms_line = choose_last(captured + tn.send("comms.status"), "[PI][STATUS]")
            current_comms = parse_comms_line(current_comms_line) if current_comms_line else None
            t1.checks.append({"name": "Pi stale and RC selects src=RC", "status": "pass" if saw_src_rc else "fail"})

            # Optional PI sub-checks
            ans_pi = prompt_step(
                "T1-B (opcional): Si podés, activa Pi fresca con DRIVE_EN=1 y manten RC moviéndose 2s. Enter para probar, s para omitir.",
                allow_skip=True,
            )
            if ans_pi == "abort":
                raise KeyboardInterrupt
            if ans_pi == "ok":
                cap_pi = tn.collect(2.5)
                all_captured_lines.extend(cap_pi)
                drive_pi = collect_drive(cap_pi)
                saw_src_pi = any(d.get("src") == "PI" for d in drive_pi)
                t1.checks.append({"name": "Pi fresh DRIVE_EN=1 selects src=PI", "status": "pass" if saw_src_pi else "fail"})
                t1.evidence.extend([d["raw_line"] for d in drive_pi[-3:]])
            else:
                t1.checks.append({"name": "Pi fresh DRIVE_EN=1 selects src=PI", "status": "skip"})

            ans_pi0 = prompt_step(
                "T1-C (opcional): Si podés, deja Pi fresca con DRIVE_EN=0 (sin freno) y RC moviéndose 2s. Enter para probar, s para omitir.",
                allow_skip=True,
            )
            if ans_pi0 == "abort":
                raise KeyboardInterrupt
            if ans_pi0 == "ok":
                cap_pi0 = tn.collect(2.5)
                all_captured_lines.extend(cap_pi0)
                drive_pi0 = collect_drive(cap_pi0)
                saw_rc_in_pi0 = any(d.get("src") == "RC" for d in drive_pi0)
                t1.checks.append({
                    "name": "Pi fresh DRIVE_EN=0 does not fall back to RC",
                    "status": "pass" if not saw_rc_in_pi0 else "fail",
                })
                t1.evidence.extend([d["raw_line"] for d in drive_pi0[-3:]])
            else:
                t1.checks.append({"name": "Pi fresh DRIVE_EN=0 does not fall back to RC", "status": "skip"})

            if current_comms_line:
                t1.evidence.append(current_comms_line)
            t1.evidence.extend([d["raw_line"] for d in drive_entries[-3:]])

            if any(chk["status"] == "fail" for chk in t1.checks):
                t1.status = "fail"
                t1.summary = "Arbitraje PI/RC no coincide con la regla esperada en al menos una verificación."
            elif all(chk["status"] == "skip" for chk in t1.checks):
                t1.status = "skip"
                t1.summary = "Caso omitido por falta de condiciones de prueba."
            else:
                t1.status = "pass"
                t1.summary = "Arbitraje principal validado (RC en Pi stale) y subcasos opcionales evaluados/omitidos."
        elif quick_auto and has_target_cmd:
            tn.send("spid.target 1.00", wait_s=0.5)
            cap_t1 = tn.collect(2.2)
            all_captured_lines.extend(cap_t1)
            tn.send("spid.target off", wait_s=0.4)
            drive_t1 = collect_drive(cap_t1)
            saw_src_tel = any(d.get("src") == "TEL" for d in drive_t1)
            t1.checks.append({"name": "spid.target forces src=TEL", "status": "pass" if saw_src_tel else "fail"})
            t1.evidence.extend([d["raw_line"] for d in drive_t1[-5:]])
            t1.status = "pass" if saw_src_tel else "fail"
            t1.summary = (
                "Override por Telnet toma control del speed PID."
                if saw_src_tel
                else "No se observó src=TEL tras inyectar setpoint por Telnet."
            )
        elif quick_auto and not has_target_cmd:
            t1.status = "skip"
            t1.summary = "Firmware sin `spid.target`; no se puede automatizar este caso."
        else:
            t1.status = "skip"
            t1.summary = "Modo no interactivo: caso de arbitraje dinámico omitido."
        report.cases.append(t1)

        # T2
        t2 = TestCaseResult("T2", "Mapping lineal RC -> m/s")
        mapping_points = [(0, 0.00), (50, 2.085), (100, 4.17)]
        if interactive:
            for rc, expected in mapping_points:
                ans = prompt_step(
                    f"T2: Coloca RC throttle en {rc}% y manten 2.5s para medir targetRaw.",
                    allow_skip=False,
                )
                if ans == "abort":
                    raise KeyboardInterrupt
                cap = tn.collect(2.5)
                all_captured_lines.extend(cap)
                drives = [d for d in collect_drive(cap) if d.get("src") == "RC"]
                latest = drives[-1] if drives else None
                actual = latest.get("targetRaw") if latest else None
                ok = approx_equal(actual, expected, 0.10)
                t2.checks.append(
                    {
                        "name": f"RC {rc}% -> targetRaw {expected:.3f} m/s (+/-0.10)",
                        "status": "pass" if ok else "fail",
                        "actual": actual,
                    }
                )
                if latest:
                    t2.evidence.append(latest["raw_line"])

            if any(chk["status"] == "fail" for chk in t2.checks):
                t2.status = "fail"
                t2.summary = "Al menos un punto del mapping lineal quedó fuera de tolerancia."
            else:
                t2.status = "pass"
                t2.summary = "Mapping RC lineal a m/s validado en 0/50/100%."
        elif quick_auto and has_target_cmd:
            target_points = [(0.00, 0.00), (2.09, 2.09), (4.17, 4.17)]
            for injected, expected in target_points:
                tn.send(f"spid.target {injected:.2f}", wait_s=0.5)
                cap = tn.collect(2.0)
                all_captured_lines.extend(cap)
                status_lines = tn.send("spid.status", wait_s=0.4)
                all_captured_lines.extend(status_lines)
                spids = collect_spid(status_lines + cap)
                latest = spids[-1] if spids else None
                actual = latest.get("targetRaw") if latest else None
                ok = approx_equal(actual, expected, 0.10)
                t2.checks.append(
                    {
                        "name": f"TEL {injected:.2f} m/s -> targetRaw {expected:.2f} (+/-0.10)",
                        "status": "pass" if ok else "fail",
                        "actual": actual,
                    }
                )
                if latest:
                    t2.evidence.append(latest["raw_line"])
            tn.send("spid.target off", wait_s=0.4)

            if any(chk["status"] == "fail" for chk in t2.checks):
                t2.status = "fail"
                t2.summary = "Al menos un setpoint inyectado por Telnet no se reflejó correctamente en targetRaw."
            else:
                t2.status = "pass"
                t2.summary = "Setpoints en m/s por Telnet reflejados correctamente en `targetRaw`."
        elif quick_auto and not has_target_cmd:
            t2.status = "skip"
            t2.summary = "Firmware sin `spid.target`; no se puede automatizar mapping por Telnet."
        else:
            t2.status = "skip"
            t2.summary = "Modo no interactivo: mapping dinámico omitido."
        report.cases.append(t2)

        # T3
        t3 = TestCaseResult("T3", "Freno manual RC negativo")
        if interactive:
            ans = prompt_step(
                "T3: Coloca RC throttle por debajo del umbral de freno (ej. -30% o menos) durante 2.5s.",
                allow_skip=False,
            )
            if ans == "abort":
                raise KeyboardInterrupt
            cap = tn.collect(2.5)
            all_captured_lines.extend(cap)
            drives = collect_drive(cap)
            brake_hits = [d for d in drives if (d.get("brake") or 0) >= 100]
            throttle_inhibited = [d for d in drives if (d.get("cmd") == 0 or (d.get("pidOut") or 0.0) <= 0.1)]
            ok_brake = len(brake_hits) > 0
            ok_inhibit = len(throttle_inhibited) > 0
            t3.checks.append({"name": "Brake >=100% with RC negative", "status": "pass" if ok_brake else "fail"})
            t3.checks.append({"name": "Throttle inhibited while braking", "status": "pass" if ok_inhibit else "fail"})
            t3.evidence.extend([d["raw_line"] for d in drives[-4:]])

            if ok_brake and ok_inhibit:
                t3.status = "pass"
                t3.summary = "Freno manual RC domina e inhibe aceleración como se espera."
            else:
                t3.status = "fail"
                t3.summary = "Freno manual RC no mostró dominancia/inhibición esperada."
        else:
            t3.status = "skip"
            t3.summary = "Requiere acción física/RC negativo; no automatizado en modo quick."
        report.cases.append(t3)

        # T4
        t4 = TestCaseResult("T4", "Fail-safe Hall")
        if interactive:
            ans = prompt_step(
                "T4: Ajusta RC throttle a 5-10% y manten rueda sin transiciones Hall (si es posible) por 3s.",
                allow_skip=True,
            )
            if ans == "abort":
                raise KeyboardInterrupt
            if ans == "ok":
                cap = tn.collect(3.2)
                all_captured_lines.extend(cap)
                drives = collect_drive(cap)
                spids = collect_spid(cap)

                saw_fs_drive = any((d.get("fb") is False and d.get("fs") is True) for d in drives)
                saw_fs_spid = any((s.get("fb") is False and s.get("failsafe") is True and s.get("mode") == "FAILSAFE") for s in spids)
                saw_zero_out = any(((d.get("pidOut") or 0.0) <= 0.1) for d in drives if d.get("fs") is True)

                t4.checks.append({"name": "Failsafe asserted (drive/spid)", "status": "pass" if (saw_fs_drive or saw_fs_spid) else "fail"})
                t4.checks.append({"name": "Throttle goes to zero in failsafe", "status": "pass" if saw_zero_out else "fail"})
                t4.evidence.extend([ln for ln in cap if ln.startswith("[DRIVE]") or ln.startswith("[SPID]")][-8:])

                if any(chk["status"] == "fail" for chk in t4.checks):
                    t4.status = "fail"
                    t4.summary = "No se evidenció fail-safe Hall completo bajo la condición ejecutada."
                else:
                    t4.status = "pass"
                    t4.summary = "Fail-safe Hall verificado con salida segura."
            else:
                t4.status = "skip"
                t4.summary = "Caso omitido: no se pudo forzar condición de feedback Hall inválido."
        elif quick_auto and has_target_cmd:
            # Best-effort automated attempt: request modest speed while wheels should remain mostly static.
            tn.send("spid.target 1.00", wait_s=0.5)
            cap = tn.collect(3.0)
            all_captured_lines.extend(cap)
            tn.send("spid.target off", wait_s=0.4)
            drives = collect_drive(cap)
            spids = collect_spid(cap)

            saw_fs_drive = any((d.get("fb") is False and d.get("fs") is True) for d in drives)
            saw_fs_spid = any((s.get("fb") is False and s.get("failsafe") is True and s.get("mode") == "FAILSAFE") for s in spids)
            t4.evidence.extend([ln for ln in cap if ln.startswith("[DRIVE]") or ln.startswith("[SPID]")][-8:])
            if saw_fs_drive or saw_fs_spid:
                t4.status = "pass"
                t4.summary = "Fail-safe observado automáticamente bajo setpoint inyectado."
                t4.checks.append({"name": "Failsafe asserted", "status": "pass"})
            else:
                t4.status = "skip"
                t4.summary = "No se pudo forzar fail-safe automáticamente sin manipular feedback Hall."
                t4.checks.append({"name": "Failsafe asserted", "status": "skip"})
        else:
            t4.status = "skip"
            t4.summary = "Modo no interactivo: caso de fail-safe omitido."
        report.cases.append(t4)

        # T5
        t5 = TestCaseResult("T5", "Overspeed")
        if interactive:
            ans = prompt_step(
                "T5-A: Lleva RC throttle a 100% durante 2.5s para spin-up. Luego Enter.",
                allow_skip=False,
            )
            if ans == "abort":
                raise KeyboardInterrupt
            warm = tn.collect(2.5)
            all_captured_lines.extend(warm)

            ans2 = prompt_step(
                "T5-B: Baja RC rápidamente a 0% y manten 3s para observar overspeed + autoBrake.",
                allow_skip=False,
            )
            if ans2 == "abort":
                raise KeyboardInterrupt
            cap = tn.collect(3.0)
            all_captured_lines.extend(cap)
            drives = collect_drive(cap)

            saw_ovs_mode = any(d.get("mode") == "OVERSPEED" for d in drives)
            saw_auto_brake = any((d.get("autoBrake") or 0.0) > 0.1 for d in drives)
            no_pos_pid = all((d.get("pidOut") or 0.0) <= 0.5 for d in drives if d.get("mode") == "OVERSPEED")

            t5.checks.append({"name": "Mode enters OVERSPEED", "status": "pass" if saw_ovs_mode else "fail"})
            t5.checks.append({"name": "AutoBrake > 0 in overspeed", "status": "pass" if saw_auto_brake else "fail"})
            t5.checks.append({"name": "No positive throttle in overspeed", "status": "pass" if no_pos_pid else "fail"})
            t5.evidence.extend([d["raw_line"] for d in drives[-8:]])

            if any(chk["status"] == "fail" for chk in t5.checks):
                t5.status = "fail"
                t5.summary = "Comportamiento overspeed incompleto o inconsistente."
            else:
                t5.status = "pass"
                t5.summary = "Overspeed con freno automático proporcional validado."
        elif quick_auto and has_target_cmd:
            tn.send("spid.target 4.17", wait_s=0.5)
            warm = tn.collect(2.8)
            all_captured_lines.extend(warm)

            warm_drives = collect_drive(warm)
            warm_speed_seen = any((d.get("speed") or 0.0) > 0.20 for d in warm_drives if d.get("src") == "TEL")

            tn.send("spid.target 0.00", wait_s=0.5)
            cap = tn.collect(3.0)
            all_captured_lines.extend(cap)
            tn.send("spid.target off", wait_s=0.4)
            drives = collect_drive(cap)

            saw_ovs_mode = any(d.get("mode") == "OVERSPEED" for d in drives)
            saw_auto_brake = any((d.get("autoBrake") or 0.0) > 0.1 for d in drives)
            no_pos_pid = all((d.get("pidOut") or 0.0) <= 0.5 for d in drives if d.get("mode") == "OVERSPEED")

            if not warm_speed_seen:
                t5.status = "skip"
                t5.summary = "No hubo velocidad suficiente para provocar overspeed en prueba automática."
                t5.checks.append({"name": "Warmup speed > 0.20 m/s", "status": "skip"})
            else:
                t5.checks.append({"name": "Mode enters OVERSPEED", "status": "pass" if saw_ovs_mode else "fail"})
                t5.checks.append({"name": "AutoBrake > 0 in overspeed", "status": "pass" if saw_auto_brake else "fail"})
                t5.checks.append({"name": "No positive throttle in overspeed", "status": "pass" if no_pos_pid else "fail"})
                if any(chk["status"] == "fail" for chk in t5.checks):
                    t5.status = "fail"
                    t5.summary = "Overspeed no se comportó como esperado en ejecución automática."
                else:
                    t5.status = "pass"
                    t5.summary = "Overspeed validado automáticamente con setpoint inyectado."
            t5.evidence.extend([d["raw_line"] for d in drives[-8:]])
        else:
            t5.status = "skip"
            t5.summary = "Modo no interactivo: caso de overspeed omitido."
        report.cases.append(t5)

        # T6
        t6 = TestCaseResult("T6", "Rendimiento del lazo")
        perf_lines = [ln for ln in all_captured_lines if ln.startswith("[DRIVE] dt=")]
        recurrent = len(perf_lines) > 2
        t6.checks.append({
            "name": "No recurrent [DRIVE] dt/runtime warnings",
            "status": "pass" if not recurrent else "fail",
            "count": len(perf_lines),
        })
        t6.evidence.extend(perf_lines[-8:])
        if recurrent:
            t6.status = "fail"
            t6.summary = f"Se detectaron warnings de performance recurrentes ({len(perf_lines)})."
        else:
            t6.status = "pass"
            t6.summary = f"Sin warnings recurrentes de dt/runtime (count={len(perf_lines)})."
        report.cases.append(t6)

        all_drive = collect_drive(all_captured_lines)
        report.diagnostics = build_diagnostics(baseline_comms, all_drive, report.cases)

        report.finished_at_utc = now_utc_iso()

        out_json = args.out
        out_md = os.path.splitext(out_json)[0] + ".md"
        ensure_parent(out_json)
        with open(out_json, "w", encoding="utf-8") as fh:
            json.dump(report.to_dict(), fh, ensure_ascii=False, indent=2)
        write_markdown(report, out_md)

        summary = summarize_status(report.cases)
        print("\n=== Speed PID HIL Test Summary ===")
        print(f"PASS={summary.get('pass', 0)} FAIL={summary.get('fail', 0)} SKIP={summary.get('skip', 0)}")
        print(f"JSON: {out_json}")
        print(f"MD:   {out_md}")

        return 1 if summary.get("fail", 0) > 0 else 0

    except KeyboardInterrupt:
        report.finished_at_utc = now_utc_iso()
        report.diagnostics.append("Ejecución abortada por usuario.")
        out_json = args.out
        out_md = os.path.splitext(out_json)[0] + ".md"
        ensure_parent(out_json)
        with open(out_json, "w", encoding="utf-8") as fh:
            json.dump(report.to_dict(), fh, ensure_ascii=False, indent=2)
        write_markdown(report, out_md)
        print("\nEjecución abortada por usuario.")
        print(f"Reporte parcial: {out_json}")
        return 130
    finally:
        try:
            tn.send("spid.target off", wait_s=0.2)
            tn.send("spid.stream off", wait_s=0.2)
            tn.send("drive.log off", wait_s=0.2)
        except Exception:
            pass
        tn.close()


def parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="HIL tests for ESP32 speed PID (RC focus)")
    parser.add_argument("--host", default="esp32-salus.local", help="Telnet host")
    parser.add_argument("--port", type=int, default=23, help="Telnet port")
    parser.add_argument("--timeout", type=float, default=5.0, help="Telnet connection timeout in seconds")
    parser.add_argument("--sample-ms", type=int, default=200, help="SPID stream period in milliseconds")
    parser.add_argument(
        "--out",
        default="artifacts/speed_pid_test_report.json",
        help="Output JSON report path",
    )
    parser.add_argument(
        "--mode",
        choices=["interactive", "quick"],
        default="interactive",
        help="interactive: guided manual RC steps; quick: baseline + passive checks",
    )
    return parser.parse_args(argv)


def main(argv: List[str]) -> int:
    args = parse_args(argv)
    return run(args)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
