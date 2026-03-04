#!/usr/bin/env python3
"""
Regression smoke for Telnet parser behavior (safe commands only).

Focus:
- Banner + base status commands
- Stream toggles (on/off [ms]) without throttle/actuation
- Unknown command handling
- Single-client replacement behavior
"""

from __future__ import annotations

import argparse
import json
import re
import telnetlib
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import List, Tuple


@dataclass
class CheckResult:
    name: str
    passed: bool
    detail: str
    evidence: List[str]


def now_utc_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def recv_lines(tn: telnetlib.Telnet, delay_s: float = 0.35) -> List[str]:
    time.sleep(delay_s)
    raw = tn.read_very_eager().decode("utf-8", errors="ignore")
    return [ln.strip() for ln in raw.splitlines() if ln.strip()]


def connect_telnet(host: str, port: int, timeout_s: float) -> Tuple[telnetlib.Telnet, List[str]]:
    tn = telnetlib.Telnet(host, port, timeout=timeout_s)
    banner = recv_lines(tn, 0.8)
    return tn, banner


def send_cmd(tn: telnetlib.Telnet, command: str, delay_s: float = 0.45) -> List[str]:
    tn.write((command + "\n").encode("utf-8"))
    return recv_lines(tn, delay_s)


def any_match(lines: List[str], pattern: str) -> bool:
    regex = re.compile(pattern)
    return any(regex.search(ln) for ln in lines)


def to_markdown(target: str, started: str, ended: str, results: List[CheckResult]) -> str:
    passed = sum(1 for r in results if r.passed)
    failed = len(results) - passed
    lines = [
        "# Telnet Command Regression",
        "",
        f"- Start (UTC): {started}",
        f"- End (UTC): {ended}",
        f"- Target: `{target}`",
        f"- PASS: {passed}",
        f"- FAIL: {failed}",
        "",
        "## Cases",
        "",
    ]
    for result in results:
        lines.append(f"### {result.name}")
        lines.append(f"- Status: **{'PASS' if result.passed else 'FAIL'}**")
        lines.append(f"- Detail: {result.detail}")
        if result.evidence:
            lines.append("- Evidence:")
            for ln in result.evidence[:8]:
                lines.append(f"  - `{ln}`")
        lines.append("")
    return "\n".join(lines)


def run(args: argparse.Namespace) -> int:
    started = now_utc_iso()
    target = f"{args.host}:{args.port}"
    results: List[CheckResult] = []

    def add(name: str, passed: bool, detail: str, evidence: List[str]) -> None:
        results.append(CheckResult(name=name, passed=passed, detail=detail, evidence=evidence))

    # T1: banner + base statuses
    tn = None
    try:
        tn, banner = connect_telnet(args.host, args.port, args.timeout)
        base_cmds = [
            ("net.status", r"^\[NET\]"),
            ("steer.status", r"^\[STEER\] estado="),
            ("pid.status", r"^\[PID\] kp="),
            ("spid.status", r"^\[SPID\] state\{"),
            ("comms.status", r"^\[PI\]\[STATUS\]"),
            ("speed.status", r"^\[SPD\]\[STATUS\]"),
            ("sys.rt", r"^\[SYS\]\[RT\]"),
            ("sys.stack", r"^\[SYS\]\[STACK\]"),
        ]
        evidence = banner[:4]
        ok_banner = any("Servidor Telnet ESP32" in ln for ln in banner)
        ok_cmds = True
        for cmd, pattern in base_cmds:
            out = send_cmd(tn, cmd)
            evidence.extend(out[:2])
            if not any_match(out, pattern):
                ok_cmds = False
        add(
            "T1_banner_and_status",
            ok_banner and ok_cmds,
            "Banner y comandos de estado base",
            evidence,
        )
    except Exception as exc:  # pragma: no cover - runtime env errors
        add("T1_banner_and_status", False, f"Exception: {exc}", [])
    finally:
        if tn is not None:
            tn.close()

    # T2: safe stream toggles
    tn = None
    try:
        tn, _ = connect_telnet(args.host, args.port, args.timeout)
        checks = [
            ("sys.jitter on 500", r"^\[SYS\]\[JITTER\] ON periodo=500ms"),
            ("sys.jitter off", r"^\[SYS\]\[JITTER\] OFF$"),
            ("pid.stream on 200", r"^\[PID\]\[STREAM\] ON periodo=200ms"),
            ("pid.stream off", r"^\[PID\]\[STREAM\] OFF$"),
            ("spid.stream on 200", r"^\[SPID\]\[STREAM\] ON periodo=200ms"),
            ("spid.stream off", r"^\[SPID\]\[STREAM\] OFF$"),
            ("speed.stream on 200", r"^\[SPD\]\[STREAM\] ON periodo=200ms"),
            ("speed.stream off", r"^\[SPD\]\[STREAM\] OFF$"),
            ("drive.rc.stream on 200", r"^\[DRIVE\]\[RC\]\[STREAM\] ON periodo=200ms"),
            ("drive.rc.stream off", r"^\[DRIVE\]\[RC\]\[STREAM\] OFF$"),
        ]
        evidence: List[str] = []
        ok = True
        for cmd, pattern in checks:
            out = send_cmd(tn, cmd)
            evidence.extend(out[:2])
            if not any_match(out, pattern):
                ok = False
        add("T2_stream_toggles", ok, "Streams seguros on/off [ms]", evidence)
    except Exception as exc:  # pragma: no cover
        add("T2_stream_toggles", False, f"Exception: {exc}", [])
    finally:
        if tn is not None:
            tn.close()

    # T3: unknown command
    tn = None
    try:
        tn, _ = connect_telnet(args.host, args.port, args.timeout)
        out = send_cmd(tn, "no.existe.cmd")
        ok = any("[STEER] Comando desconocido: no.existe.cmd" in ln for ln in out)
        add("T3_unknown_command", ok, "Manejo de comando invalido", out[:6])
    except Exception as exc:  # pragma: no cover
        add("T3_unknown_command", False, f"Exception: {exc}", [])
    finally:
        if tn is not None:
            tn.close()

    # T4: single-client replacement
    c1 = None
    c2 = None
    try:
        c1, _ = connect_telnet(args.host, args.port, args.timeout)
        _ = send_cmd(c1, "net.status", 0.3)
        c2, banner2 = connect_telnet(args.host, args.port, args.timeout)
        tail_c1 = recv_lines(c1, 0.4)
        msg_ok = any("nuevo cliente conectado" in ln.lower() for ln in tail_c1)
        out_c2 = send_cmd(c2, "net.status", 0.3)
        c2_ok = any_match(out_c2, r"^\[NET\]")
        add(
            "T4_single_client_replacement",
            msg_ok and c2_ok,
            "Sesion nueva reemplaza a la anterior",
            banner2[:3] + tail_c1[:3] + out_c2[:3],
        )
    except Exception as exc:  # pragma: no cover
        add("T4_single_client_replacement", False, f"Exception: {exc}", [])
    finally:
        if c1 is not None:
            c1.close()
        if c2 is not None:
            c2.close()

    ended = now_utc_iso()
    passed = sum(1 for r in results if r.passed)
    failed = len(results) - passed

    payload = {
        "start_utc": started,
        "end_utc": ended,
        "target": target,
        "summary": {"pass": passed, "fail": failed},
        "results": [
            {
                "name": r.name,
                "status": "PASS" if r.passed else "FAIL",
                "detail": r.detail,
                "evidence": r.evidence,
            }
            for r in results
        ],
    }

    out_json = Path(args.out_json)
    out_md = Path(args.out_md)
    out_json.parent.mkdir(parents=True, exist_ok=True)
    out_md.parent.mkdir(parents=True, exist_ok=True)
    out_json.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    out_md.write_text(to_markdown(target, started, ended, results) + "\n", encoding="utf-8")

    print(f"Target: {target}")
    print(f"PASS={passed} FAIL={failed}")
    print(f"JSON: {out_json}")
    print(f"MD:   {out_md}")

    return 0 if failed == 0 else 1


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Regression smoke for Telnet command parser")
    parser.add_argument("--host", default="esp32-salus.local", help="Telnet host")
    parser.add_argument("--port", type=int, default=23, help="Telnet port")
    parser.add_argument("--timeout", type=float, default=8.0, help="Telnet timeout seconds")
    parser.add_argument(
        "--out-json",
        default="artifacts/telnet_command_regression.json",
        help="Output JSON report",
    )
    parser.add_argument(
        "--out-md",
        default="artifacts/telnet_command_regression.md",
        help="Output Markdown report",
    )
    return parser


if __name__ == "__main__":
    raise SystemExit(run(build_arg_parser().parse_args()))
