#!/usr/bin/env python3
"""HIL validation for signed speed targets (TEL simulation of PI reverse requests)."""

from __future__ import annotations

import argparse
import json
import os
import re
import telnetlib
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Dict, List, Optional, Tuple

RE_SPLIT = re.compile(r"(?<!\n)(?<!\])(?=\[[A-Z])")
RE_DIR = re.compile(r"^\[DRIVE\]\[DIR\]\s+(?P<dir>FWD|REV)\s+switching=(?P<switch>[YN])")
RE_SPEED = re.compile(
    r"^\[SPD\]\[STATUS\].*?dir=(?P<dir>FWD|REV|UNK).*?speed=(?P<kmh>[-+]?[0-9.]+)km/h\s+(?P<mps>[-+]?[0-9.]+)m/s"
)


@dataclass
class CaseResult:
    case_id: str
    status: str
    summary: str
    expected_dir: str
    final_dir: Optional[str]
    saw_switching: bool
    speed_sign_ok: Optional[bool]
    motion_observed: bool
    max_abs_speed_mps: float
    evidence: List[str]


def now_iso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


class TelnetClient:
    def __init__(self, host: str, port: int, timeout: float):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.tn: Optional[telnetlib.Telnet] = None

    def connect(self) -> List[str]:
        self.tn = telnetlib.Telnet(self.host, self.port, timeout=self.timeout)
        time.sleep(0.7)
        return self.read_lines()

    def read_lines(self) -> List[str]:
        if self.tn is None:
            return []
        try:
            data = self.tn.read_very_eager()
        except EOFError:
            return []
        if not data:
            return []
        text = RE_SPLIT.sub("\n", data.decode("utf-8", errors="ignore"))
        return [ln.strip() for ln in text.splitlines() if ln.strip()]

    def send(self, cmd: str, wait_s: float = 0.35) -> List[str]:
        if self.tn is None:
            raise RuntimeError("telnet not connected")
        self.tn.write((cmd + "\n").encode("utf-8"))
        time.sleep(wait_s)
        return self.read_lines()

    def collect(self, duration_s: float, poll_s: float = 0.08) -> List[str]:
        out: List[str] = []
        end = time.monotonic() + duration_s
        while time.monotonic() < end:
            out.extend(self.read_lines())
            time.sleep(poll_s)
        out.extend(self.read_lines())
        return out

    def close(self) -> None:
        if self.tn is None:
            return
        try:
            self.tn.write(b"spid.target off\n")
            time.sleep(0.1)
            self.tn.write(b"exit\n")
            time.sleep(0.1)
        except Exception:
            pass
        try:
            self.tn.close()
        except Exception:
            pass
        self.tn = None


def parse_last_dir(lines: List[str]) -> Optional[str]:
    for line in reversed(lines):
        m = RE_DIR.search(line)
        if m:
            return m.group("dir")
    return None


def parse_speed_samples(lines: List[str]) -> List[float]:
    out: List[float] = []
    for line in lines:
        m = RE_SPEED.search(line)
        if m:
            out.append(float(m.group("mps")))
    return out


def run_signed_target_case(
    client: TelnetClient,
    case_id: str,
    target_mps: float,
    expected_dir: str,
    expect_switching: bool,
    duration_s: float,
    motion_threshold_mps: float,
    require_motion: bool,
) -> Tuple[CaseResult, List[str]]:
    captured: List[str] = []
    captured.extend(client.send(f"spid.target {target_mps:.2f}", wait_s=0.45))
    for _ in range(4):
        captured.extend(client.send("drive.dir", wait_s=0.12))

    start = time.monotonic()
    next_probe = start + 0.5
    while time.monotonic() - start < duration_s:
        captured.extend(client.collect(0.18, poll_s=0.05))
        if time.monotonic() >= next_probe:
            captured.extend(client.send("drive.dir", wait_s=0.10))
            captured.extend(client.send("speed.status", wait_s=0.10))
            next_probe = time.monotonic() + 0.5

    final_dir = parse_last_dir(captured)
    saw_switching = any("[DRIVE][DIR]" in ln and "switching=Y" in ln for ln in captured)
    speed_samples = parse_speed_samples(captured)
    max_abs_speed = max((abs(v) for v in speed_samples), default=0.0)
    motion_observed = max_abs_speed >= motion_threshold_mps

    speed_sign_ok: Optional[bool] = None
    target_positive = target_mps > 0.05
    target_negative = target_mps < -0.05
    target_zero = not target_positive and not target_negative
    if motion_observed:
        if target_positive:
            speed_sign_ok = any(v >= motion_threshold_mps for v in speed_samples)
        elif target_negative:
            speed_sign_ok = any(v <= -motion_threshold_mps for v in speed_samples)

    dir_ok = final_dir == expected_dir
    switching_ok = (not expect_switching) or saw_switching

    if target_zero:
        sign_ok = True
        sign_note = "signo N/A (target=0)"
    elif speed_sign_ok is None:
        sign_ok = not require_motion
        sign_note = "signo no observado (sin movimiento suficiente)"
    else:
        sign_ok = speed_sign_ok
        sign_note = "signo OK" if speed_sign_ok else "signo incorrecto"

    passed = dir_ok and switching_ok and sign_ok
    status = "PASS" if passed else "FAIL"

    summary = (
        f"target={target_mps:.2f} dir={final_dir or 'N/A'} "
        f"switch={('Y' if saw_switching else 'N')} motionMax={max_abs_speed:.2f}m/s {sign_note}"
    )

    evidence = [ln for ln in captured if ln.startswith("[SPID][TARGET]") or ln.startswith("[DRIVE][DIR]") or ln.startswith("[SPD][STATUS]")]
    return (
        CaseResult(
            case_id=case_id,
            status=status,
            summary=summary,
            expected_dir=expected_dir,
            final_dir=final_dir,
            saw_switching=saw_switching,
            speed_sign_ok=speed_sign_ok,
            motion_observed=motion_observed,
            max_abs_speed_mps=round(max_abs_speed, 3),
            evidence=evidence[:20],
        ),
        captured,
    )


def write_markdown(path: str, report: Dict[str, object]) -> None:
    lines: List[str] = []
    lines.append("# Reverse Signed Target HIL")
    lines.append("")
    lines.append(f"- Start (UTC): {report['started_at_utc']}")
    lines.append(f"- End (UTC): {report['finished_at_utc']}")
    lines.append(f"- Target: `{report['host']}:{report['port']}`")
    lines.append(f"- PASS: {report['summary']['pass']}")
    lines.append(f"- FAIL: {report['summary']['fail']}")
    lines.append("")
    lines.append("## Cases")
    lines.append("")
    for case in report["cases"]:
        lines.append(f"### {case['case_id']}")
        lines.append(f"- Status: **{case['status']}**")
        lines.append(f"- Summary: {case['summary']}")
        lines.append(f"- Expected dir: `{case['expected_dir']}`")
        lines.append(f"- Final dir: `{case['final_dir']}`")
        lines.append(f"- Motion observed: `{case['motion_observed']}`")
        lines.append("- Evidence:")
        for ln in case.get("evidence", [])[:8]:
            lines.append(f"  - `{ln}`")
        lines.append("")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Validate signed speed target reverse behavior over Telnet")
    p.add_argument("--host", default="esp32-salus.local")
    p.add_argument("--port", type=int, default=23)
    p.add_argument("--timeout", type=float, default=8.0)
    p.add_argument("--target-mps", type=float, default=1.00)
    p.add_argument("--case-seconds", type=float, default=3.0)
    p.add_argument("--motion-threshold", type=float, default=0.10)
    p.add_argument("--require-motion", action="store_true", help="Fail if speed sign cannot be observed")
    p.add_argument("--out-json", default="artifacts/reverse_signed_telnet_hil_report.json")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    os.makedirs(os.path.dirname(args.out_json), exist_ok=True)
    out_md = os.path.splitext(args.out_json)[0] + ".md"

    report: Dict[str, object] = {
        "started_at_utc": now_iso(),
        "host": args.host,
        "port": args.port,
        "params": {
            "target_mps": args.target_mps,
            "case_seconds": args.case_seconds,
            "motion_threshold_mps": args.motion_threshold,
            "require_motion": args.require_motion,
        },
        "preflight": {},
        "cases": [],
    }

    client = TelnetClient(args.host, args.port, args.timeout)
    all_lines: List[str] = []

    try:
        all_lines.extend(client.connect())
        preflight_lines: List[str] = []
        for cmd in ("comms.status", "spid.status", "speed.status", "drive.dir"):
            preflight_lines.extend(client.send(cmd, wait_s=0.35))
        report["preflight"] = {"lines": preflight_lines[-12:]}
        all_lines.extend(preflight_lines)

        for cmd in ("spid.target off", "spid.stream off", "speed.stream off", "drive.log pid off", "drive.log off"):
            all_lines.extend(client.send(cmd, wait_s=0.25))

        case_a, lines_a = run_signed_target_case(
            client,
            "A_pos_target",
            abs(args.target_mps),
            expected_dir="FWD",
            expect_switching=False,
            duration_s=args.case_seconds,
            motion_threshold_mps=args.motion_threshold,
            require_motion=args.require_motion,
        )
        all_lines.extend(lines_a)

        case_b, lines_b = run_signed_target_case(
            client,
            "B_neg_target",
            -abs(args.target_mps),
            expected_dir="REV",
            expect_switching=True,
            duration_s=args.case_seconds,
            motion_threshold_mps=args.motion_threshold,
            require_motion=args.require_motion,
        )
        all_lines.extend(lines_b)

        case_c, lines_c = run_signed_target_case(
            client,
            "C_zero_target",
            0.0,
            expected_dir="FWD",
            expect_switching=False,
            duration_s=max(2.0, args.case_seconds * 0.7),
            motion_threshold_mps=args.motion_threshold,
            require_motion=False,
        )
        all_lines.extend(lines_c)

        off_lines = client.send("spid.target off", wait_s=0.35)
        off_lines.extend(client.send("drive.dir", wait_s=0.20))
        all_lines.extend(off_lines)
        off_ok = any(ln.startswith("[SPID][TARGET] OFF") for ln in off_lines)
        case_d = CaseResult(
            case_id="D_target_off",
            status="PASS" if off_ok else "FAIL",
            summary="spid.target off desactiva override" if off_ok else "spid.target off no confirmado",
            expected_dir="FWD",
            final_dir=parse_last_dir(off_lines),
            saw_switching=any("switching=Y" in ln for ln in off_lines if "[DRIVE][DIR]" in ln),
            speed_sign_ok=None,
            motion_observed=False,
            max_abs_speed_mps=0.0,
            evidence=[ln for ln in off_lines if ln.startswith("[SPID][TARGET]") or ln.startswith("[DRIVE][DIR]")][:10],
        )

        cases = [case_a, case_b, case_c, case_d]
        pass_count = sum(1 for c in cases if c.status == "PASS")
        fail_count = len(cases) - pass_count

        report["cases"] = [c.__dict__ for c in cases]
        report["summary"] = {"pass": pass_count, "fail": fail_count}
        report["captured_tail"] = all_lines[-80:]

    except Exception as exc:
        report["error"] = str(exc)
        report.setdefault("summary", {"pass": 0, "fail": 1})
    finally:
        client.close()

    report["finished_at_utc"] = now_iso()

    with open(args.out_json, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
    write_markdown(out_md, report)

    print(f"Report JSON: {args.out_json}")
    print(f"Report MD:   {out_md}")
    if report.get("error"):
        print(f"Error: {report['error']}")
        return 1
    return 0 if report.get("summary", {}).get("fail", 1) == 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
