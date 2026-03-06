#!/usr/bin/env python3
"""Autotune helper for reverse PID behavior using Telnet commands.

Tuning knobs:
- spid.maxrev (reverse clamp speed in m/s)
- spid.awx (reverse anti-windup unwind scale)
"""

from __future__ import annotations

import argparse
import json
import re
import statistics
import telnetlib
import time
from dataclasses import dataclass, asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import List, Tuple


RE_SPEED = re.compile(r"\[SPD\]\[STATUS\].*?dir=(FWD|REV|UNK).*?speed=[^ ]+\s+([+-]?\d+(?:\.\d+)?)m/s")
RE_SPID = re.compile(
    r"\[SPID\].*?targetRaw=([+-]?\d+(?:\.\d+)?)m/s.*?target=([+-]?\d+(?:\.\d+)?)m/s.*?speedRaw=([+-]?\d+(?:\.\d+)?)m/s"
)
RE_REV_STATUS = re.compile(r"\[SPID\]\[REV\]\s+max=([+-]?\d+(?:\.\d+)?)m/s\s+awx=([+-]?\d+(?:\.\d+)?)")


@dataclass
class CaseResult:
    target_mps: float
    samples: int
    target_raw_mean_mps: float
    speed_abs_mean_mps: float
    speed_abs_peak_mps: float
    abs_error_mps: float
    sign_ok: bool


@dataclass
class ConfigResult:
    maxrev_mps: float
    awx: float
    avg_abs_error_mps: float
    sign_fail_count: int
    score: float
    cases: List[CaseResult]


class TelnetClient:
    def __init__(self, host: str, port: int, timeout_s: float = 8.0) -> None:
        self.host = host
        self.port = port
        self.timeout_s = timeout_s
        self.tn: telnetlib.Telnet | None = None

    def __enter__(self) -> "TelnetClient":
        self.tn = telnetlib.Telnet(self.host, self.port, timeout=self.timeout_s)
        time.sleep(0.8)
        self.tn.read_very_eager()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self.tn is not None:
            try:
                self.send("spid.target 0.00", wait_s=0.2)
                self.send("spid.target off", wait_s=0.2)
                self.send("drive.log pid off", wait_s=0.2)
                self.send("drive.log off", wait_s=0.2)
            except Exception:
                pass
            self.tn.close()
            self.tn = None

    def send(self, cmd: str, wait_s: float = 0.22) -> List[str]:
        assert self.tn is not None
        self.tn.write((cmd + "\n").encode("utf-8"))
        time.sleep(wait_s)
        out = self.tn.read_very_eager().decode(errors="ignore")
        return [ln.strip() for ln in out.splitlines() if ln.strip()]


def parse_rev_status(lines: List[str]) -> Tuple[float, float]:
    for ln in reversed(lines):
        m = RE_REV_STATUS.search(ln)
        if m:
            return float(m.group(1)), float(m.group(2))
    return 0.0, 0.0


def run_case(client: TelnetClient, target_mps: float, settle_s: float, sample_s: float) -> CaseResult:
    client.send(f"spid.target {target_mps:.2f}", wait_s=0.30)
    t_start = time.monotonic()

    speeds: List[float] = []
    targets_raw: List[float] = []

    while (time.monotonic() - t_start) < (settle_s + sample_s):
        elapsed = time.monotonic() - t_start
        sample_phase = elapsed >= settle_s

        for ln in client.send("speed.status", wait_s=0.08):
            m = RE_SPEED.search(ln)
            if m and sample_phase:
                speeds.append(float(m.group(2)))

        for ln in client.send("spid.status", wait_s=0.08):
            m = RE_SPID.search(ln)
            if m and sample_phase:
                targets_raw.append(float(m.group(1)))

    if not speeds or not targets_raw:
        return CaseResult(
            target_mps=target_mps,
            samples=0,
            target_raw_mean_mps=0.0,
            speed_abs_mean_mps=0.0,
            speed_abs_peak_mps=0.0,
            abs_error_mps=999.0,
            sign_ok=False,
        )

    target_raw_mean = statistics.mean(targets_raw)
    speed_abs = [abs(v) for v in speeds]
    speed_abs_mean = statistics.mean(speed_abs)
    speed_abs_peak = max(speed_abs)
    sign_ok = all(v <= 0.05 for v in speeds)
    abs_error = abs(target_raw_mean - speed_abs_mean)

    return CaseResult(
        target_mps=target_mps,
        samples=len(speeds),
        target_raw_mean_mps=target_raw_mean,
        speed_abs_mean_mps=speed_abs_mean,
        speed_abs_peak_mps=speed_abs_peak,
        abs_error_mps=abs_error,
        sign_ok=sign_ok,
    )


def evaluate_config(client: TelnetClient,
                    maxrev_mps: float,
                    awx: float,
                    targets_mps: List[float],
                    settle_s: float,
                    sample_s: float) -> ConfigResult:
    client.send(f"spid.maxrev {maxrev_mps:.2f}", wait_s=0.25)
    client.send(f"spid.awx {awx:.2f}", wait_s=0.25)
    status_lines = client.send("spid.maxrev", wait_s=0.15)
    applied_maxrev, applied_awx = parse_rev_status(status_lines)

    cases: List[CaseResult] = []
    for target in targets_mps:
        cases.append(run_case(client, target, settle_s, sample_s))

    avg_error = statistics.mean(c.abs_error_mps for c in cases) if cases else 999.0
    sign_fails = sum(0 if c.sign_ok else 1 for c in cases)
    score = avg_error + (2.0 * sign_fails)

    return ConfigResult(
        maxrev_mps=applied_maxrev if applied_maxrev > 0.0 else maxrev_mps,
        awx=applied_awx if applied_awx > 0.0 else awx,
        avg_abs_error_mps=avg_error,
        sign_fail_count=sign_fails,
        score=score,
        cases=cases,
    )


def write_reports(report: dict, out_json: Path, out_md: Path) -> None:
    out_json.parent.mkdir(parents=True, exist_ok=True)
    out_json.write_text(json.dumps(report, indent=2), encoding="utf-8")

    lines: List[str] = []
    lines.append("# Reverse PID Autotune HIL")
    lines.append("")
    lines.append(f"- Start (UTC): {report['started_at_utc']}")
    lines.append(f"- End (UTC): {report['finished_at_utc']}")
    lines.append(f"- Target: `{report['host']}:{report['port']}`")
    lines.append(f"- Best: `maxrev={report['best']['maxrev_mps']:.2f} awx={report['best']['awx']:.2f}`")
    lines.append("")
    lines.append("| maxrev | awx | avg abs err m/s | sign fails | score |")
    lines.append("|---:|---:|---:|---:|---:|")
    for cfg in report["results"]:
        lines.append(
            "| {maxrev_mps:.2f} | {awx:.2f} | {avg_abs_error_mps:.3f} | {sign_fail_count} | {score:.3f} |".format(
                **cfg
            )
        )
    lines.append("")
    lines.append("## Best Config Cases")
    lines.append("")
    lines.append("| target m/s | targetRaw mean | speed abs mean | speed abs peak | abs err | sign ok |")
    lines.append("|---:|---:|---:|---:|---:|:---:|")
    for case in report["best"]["cases"]:
        lines.append(
            "| {target_mps:.2f} | {target_raw_mean_mps:.3f} | {speed_abs_mean_mps:.3f} | "
            "{speed_abs_peak_mps:.3f} | {abs_error_mps:.3f} | {ok} |".format(
                ok="Y" if case["sign_ok"] else "N", **case
            )
        )
    out_md.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_float_list(csv_text: str) -> List[float]:
    out: List[float] = []
    for part in csv_text.split(","):
        part = part.strip()
        if not part:
            continue
        out.append(float(part))
    return out


def main() -> int:
    p = argparse.ArgumentParser(description="Autotune reverse PID knobs over Telnet")
    p.add_argument("--host", default="esp32-salus.local")
    p.add_argument("--port", type=int, default=23)
    p.add_argument("--maxrev-list", default="1.20,1.30,1.35")
    p.add_argument("--awx-list", default="1.50,2.50,3.50")
    p.add_argument("--targets", default="-1.00,-2.00")
    p.add_argument("--settle-s", type=float, default=1.5)
    p.add_argument("--sample-s", type=float, default=3.0)
    p.add_argument("--apply-best", action="store_true", default=True)
    p.add_argument("--out-json", default="artifacts/reverse_pid_autotune_hil.json")
    p.add_argument("--out-md", default="artifacts/reverse_pid_autotune_hil.md")
    args = p.parse_args()

    maxrev_candidates = parse_float_list(args.maxrev_list)
    awx_candidates = parse_float_list(args.awx_list)
    targets = parse_float_list(args.targets)

    report = {
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "host": args.host,
        "port": args.port,
        "params": {
            "maxrev_candidates": maxrev_candidates,
            "awx_candidates": awx_candidates,
            "targets_mps": targets,
            "settle_s": args.settle_s,
            "sample_s": args.sample_s,
        },
        "results": [],
    }

    with TelnetClient(args.host, args.port) as client:
        for cmd in ("spid.target off", "spid.stream off", "speed.stream off", "drive.log pid off", "drive.log off"):
            client.send(cmd, wait_s=0.2)

        best: ConfigResult | None = None
        for maxrev in maxrev_candidates:
            for awx in awx_candidates:
                cfg = evaluate_config(client, maxrev, awx, targets, args.settle_s, args.sample_s)
                report["results"].append(
                    {
                        "maxrev_mps": round(cfg.maxrev_mps, 3),
                        "awx": round(cfg.awx, 3),
                        "avg_abs_error_mps": round(cfg.avg_abs_error_mps, 4),
                        "sign_fail_count": cfg.sign_fail_count,
                        "score": round(cfg.score, 4),
                        "cases": [asdict(c) for c in cfg.cases],
                    }
                )
                if best is None or cfg.score < best.score:
                    best = cfg

        if best is None:
            raise RuntimeError("no_results")

        if args.apply_best:
            client.send(f"spid.maxrev {best.maxrev_mps:.2f}", wait_s=0.25)
            client.send(f"spid.awx {best.awx:.2f}", wait_s=0.25)

        report["best"] = {
            "maxrev_mps": round(best.maxrev_mps, 3),
            "awx": round(best.awx, 3),
            "avg_abs_error_mps": round(best.avg_abs_error_mps, 4),
            "sign_fail_count": best.sign_fail_count,
            "score": round(best.score, 4),
            "cases": [asdict(c) for c in best.cases],
            "applied": bool(args.apply_best),
        }

        client.send("spid.target 0.00", wait_s=0.2)
        client.send("spid.target off", wait_s=0.2)

    report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
    out_json = Path(args.out_json)
    out_md = Path(args.out_md)
    write_reports(report, out_json, out_md)
    print(f"Report JSON: {out_json}")
    print(f"Report MD:   {out_md}")
    print(
        "Best config: maxrev={:.2f} awx={:.2f} score={:.3f}".format(
            report["best"]["maxrev_mps"], report["best"]["awx"], report["best"]["score"]
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
