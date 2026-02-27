#!/usr/bin/env python3
"""System RT HIL runner for ESP32_SALUS using `sys.*` Telnet diagnostics.

Runs two stages:
1) Nominal (expected low logging load)
2) Stress (PID trace enabled)

Collects `sys.jitter` stream plus point-in-time `sys.rt` and `sys.stack` snapshots.
Outputs JSON + Markdown evidence artifacts.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import statistics
import sys
import telnetlib
import time
from datetime import datetime, timezone
from typing import Any, Callable, Dict, List, Optional, Tuple


RE_SPLIT_MARKER = re.compile(r"(?<!\n)(?<!\])(?=\[[A-Z])")
RE_KV = re.compile(r"([A-Za-z][A-Za-z0-9_]*)=([^\s]+)")


def now_iso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def ensure_parent(path: str) -> None:
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)


def to_int(value: str) -> Optional[int]:
    text = value.strip().rstrip(",")
    if text.endswith("ms"):
        text = text[:-2]
    if text.endswith("us"):
        text = text[:-2]
    try:
        return int(text)
    except ValueError:
        try:
            return int(float(text))
        except ValueError:
            return None


def percentile(values: List[float], p: float) -> float:
    if not values:
        return 0.0
    if p <= 0:
        return min(values)
    if p >= 100:
        return max(values)
    vals = sorted(values)
    idx = (len(vals) - 1) * (p / 100.0)
    lo = int(idx)
    hi = min(lo + 1, len(vals) - 1)
    frac = idx - lo
    return vals[lo] + (vals[hi] - vals[lo]) * frac


class TelnetRunner:
    def __init__(self, host: str, port: int, timeout: float):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.tn: Optional[telnetlib.Telnet] = None

    def connect(self) -> List[str]:
        self.tn = telnetlib.Telnet(self.host, self.port, timeout=self.timeout)
        time.sleep(0.8)
        return self.read_lines()

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

    def read_lines(self) -> List[str]:
        if self.tn is None:
            return []
        try:
            payload = self.tn.read_very_eager()
        except EOFError:
            return []
        if not payload:
            return []
        decoded = RE_SPLIT_MARKER.sub("\n", payload.decode("utf-8", errors="ignore"))
        return [line.strip() for line in decoded.splitlines() if line.strip()]

    def send(self, command: str, wait_s: float = 0.35) -> List[str]:
        if self.tn is None:
            raise RuntimeError("Telnet not connected")
        self.tn.write((command + "\n").encode("utf-8"))
        time.sleep(wait_s)
        return self.read_lines()

    def heartbeat(self) -> None:
        if self.tn is None:
            raise RuntimeError("Telnet not connected")
        self.tn.write(b"\n")

    def collect(self,
                duration_s: float,
                poll_s: float = 0.10,
                keepalive_s: float = 30.0,
                max_silence_s: float = 20.0,
                on_poll: Optional[Callable[[List[str], float], None]] = None) -> List[str]:
        lines: List[str] = []
        end = time.monotonic() + duration_s
        now = time.monotonic()
        next_keepalive = now + keepalive_s if keepalive_s > 0 else 0.0
        last_data = now
        while time.monotonic() < end:
            now = time.monotonic()
            chunk = self.read_lines()
            if chunk:
                lines.extend(chunk)
                last_data = now
            if on_poll is not None:
                on_poll(chunk, now)
            if max_silence_s > 0 and (now - last_data) >= max_silence_s:
                raise RuntimeError(f"no_telnet_data_for_{max_silence_s:.1f}s")
            if keepalive_s > 0 and now >= next_keepalive:
                self.heartbeat()
                next_keepalive = now + keepalive_s
            time.sleep(poll_s)
        tail = self.read_lines()
        if tail:
            lines.extend(tail)
        if on_poll is not None:
            on_poll(tail, time.monotonic())
        return lines


def parse_sys_jitter_line(line: str) -> Optional[Dict[str, Any]]:
    if not line.startswith("[SYS][JITTER]"):
        return None
    tokens = RE_KV.findall(line)
    out: Dict[str, Any] = {"raw_line": line, "tasks": {}}
    for key, value in tokens:
        if key in ("tsMs", "worstMaxUs"):
            out[key] = to_int(value)
            continue
        if key == "tasks":
            out["tasks_count"] = to_int(value)
            continue
        if "/" in value:
            left, right = value.split("/", 1)
            current = to_int(left)
            maxv = to_int(right)
            out["tasks"][key] = {
                "current_us": current if current is not None else 0,
                "max_us": maxv if maxv is not None else 0,
            }
    return out


def summarize_jitter(entries: List[Dict[str, Any]]) -> Dict[str, Any]:
    summary: Dict[str, Any] = {
        "samples": len(entries),
        "worst_max_us": 0,
        "tasks": {},
    }
    if not entries:
        return summary

    worst_values = [e.get("worstMaxUs", 0) or 0 for e in entries]
    summary["worst_max_us"] = int(max(worst_values))

    per_task_current: Dict[str, List[float]] = {}
    per_task_max: Dict[str, List[float]] = {}
    for entry in entries:
        for task, payload in entry.get("tasks", {}).items():
            per_task_current.setdefault(task, []).append(float(payload.get("current_us", 0)))
            per_task_max.setdefault(task, []).append(float(payload.get("max_us", 0)))

    for task in sorted(per_task_current.keys()):
        current = per_task_current[task]
        maxes = per_task_max.get(task, [])
        summary["tasks"][task] = {
            "current_p95_us": round(percentile(current, 95), 2),
            "current_p99_us": round(percentile(current, 99), 2),
            "current_mean_us": round(statistics.mean(current), 2) if current else 0.0,
            "max_observed_us": int(max(maxes)) if maxes else 0,
        }

    return summary


class StageHealthMonitor:
    def __init__(self,
                 stage_name: str,
                 jitter_period_ms: int,
                 progress_period_s: float,
                 max_jitter_gap_s: float,
                 progress_log_path: str):
        self.stage_name = stage_name
        self.jitter_period_s = max(float(jitter_period_ms) / 1000.0, 0.001)
        self.progress_period_s = max(progress_period_s, 1.0)
        self.max_jitter_gap_s = max(max_jitter_gap_s, self.jitter_period_s * 2.0)
        self.progress_log_path = progress_log_path

        now = time.monotonic()
        self.started_mono = now
        self.last_progress_mono = now
        self.last_jitter_mono = now
        self.last_jitter_ts_ms = 0
        self.jitter_samples = 0

    def _emit_progress(self, now_mono: float, force: bool = False) -> None:
        if not force and (now_mono - self.last_progress_mono) < self.progress_period_s:
            return
        elapsed_s = now_mono - self.started_mono
        expected_samples = int(elapsed_s / self.jitter_period_s) if self.jitter_period_s > 0 else 0
        coverage = (100.0 * self.jitter_samples / expected_samples) if expected_samples > 0 else 100.0
        jitter_gap_s = now_mono - self.last_jitter_mono
        status = "OK" if jitter_gap_s <= self.max_jitter_gap_s else "WARN"
        msg = (
            f"[{self.stage_name}][PROGRESS] "
            f"elapsed={elapsed_s:.1f}s "
            f"samples={self.jitter_samples} "
            f"expected~={expected_samples} "
            f"coverage~={coverage:.1f}% "
            f"jitterGap={jitter_gap_s:.1f}s "
            f"lastTsMs={self.last_jitter_ts_ms} "
            f"status={status}"
        )
        print(msg, flush=True)
        if self.progress_log_path:
            ensure_parent(self.progress_log_path)
            with open(self.progress_log_path, "a", encoding="utf-8") as fh:
                fh.write(msg + "\n")
        self.last_progress_mono = now_mono

    def poll(self, lines: List[str], now_mono: float) -> None:
        for line in lines:
            parsed = parse_sys_jitter_line(line)
            if parsed is None:
                continue
            self.jitter_samples += 1
            self.last_jitter_mono = now_mono
            ts_ms = parsed.get("tsMs")
            if isinstance(ts_ms, int):
                self.last_jitter_ts_ms = ts_ms
        self._emit_progress(now_mono)
        if (now_mono - self.last_jitter_mono) > self.max_jitter_gap_s:
            raise RuntimeError(
                f"jitter_stream_gap_{(now_mono - self.last_jitter_mono):.1f}s"
                f"_exceeds_{self.max_jitter_gap_s:.1f}s"
            )

    def finish(self) -> None:
        self._emit_progress(time.monotonic(), force=True)


def collect_stage(runner: TelnetRunner,
                  name: str,
                  duration_s: float,
                  jitter_period_ms: int,
                  keepalive_s: float,
                  max_silence_s: float,
                  progress_period_s: float,
                  max_jitter_gap_s: float,
                  progress_log_path: str,
                  min_stage_coverage_pct: float,
                  raw_records: List[Dict[str, str]]) -> Dict[str, Any]:
    stage: Dict[str, Any] = {"name": name}
    monitor = StageHealthMonitor(
        stage_name=name,
        jitter_period_ms=jitter_period_ms,
        progress_period_s=progress_period_s,
        max_jitter_gap_s=max_jitter_gap_s,
        progress_log_path=progress_log_path,
    )
    runner.send(f"sys.jitter on {jitter_period_ms}")
    lines = runner.collect(
        duration_s,
        keepalive_s=keepalive_s,
        max_silence_s=max_silence_s,
        on_poll=monitor.poll,
    )
    monitor.finish()
    runner.send("sys.jitter off")

    for line in lines:
        raw_records.append({"stage": name, "line": line})

    jitter_entries = [p for p in (parse_sys_jitter_line(line) for line in lines) if p is not None]
    stage["jitter"] = summarize_jitter(jitter_entries)
    stage["jitter_lines"] = len(jitter_entries)
    expected_samples = int((duration_s * 1000.0) // float(jitter_period_ms)) if jitter_period_ms > 0 else 0
    coverage_pct = (100.0 * len(jitter_entries) / expected_samples) if expected_samples > 0 else 100.0
    stage["expected_jitter_samples"] = expected_samples
    stage["coverage_pct"] = round(coverage_pct, 2)
    stage["coverage_ok"] = coverage_pct >= min_stage_coverage_pct
    stage["sys_rt_snapshot"] = runner.send("sys.rt")
    stage["sys_stack_snapshot"] = runner.send("sys.stack")
    stage["snapshot_ok"] = bool(stage["sys_rt_snapshot"]) and bool(stage["sys_stack_snapshot"])
    return stage


def validate_pi_uart_rx_thresholds(stage: Dict[str, Any],
                                   p95_max_us: float,
                                   p99_max_us: float) -> Tuple[bool, str]:
    jitter = stage.get("jitter", {})
    tasks = jitter.get("tasks", {})
    metrics = tasks.get("PiUartRx")
    if not isinstance(metrics, dict):
        return False, "PiUartRx metrics missing"
    p95 = float(metrics.get("current_p95_us", 0.0))
    p99 = float(metrics.get("current_p99_us", 0.0))
    if p95 <= p95_max_us and p99 <= p99_max_us:
        return True, f"PiUartRx p95={p95:.2f}us p99={p99:.2f}us"
    return False, (
        f"PiUartRx p95={p95:.2f}us (max {p95_max_us:.2f}) "
        f"p99={p99:.2f}us (max {p99_max_us:.2f})"
    )


def require_sys_reset(runner: TelnetRunner, stage_name: str, raw_records: List[Dict[str, str]]) -> None:
    lines = runner.send("sys.reset keep")
    for line in lines:
        raw_records.append({"stage": f"{stage_name}_setup", "line": line})
    if not any(line.startswith("[SYS][RESET] OK") for line in lines):
        raise RuntimeError(f"{stage_name}_sys_reset_failed")


def write_markdown(path: str, report: Dict[str, Any]) -> None:
    lines: List[str] = []
    lines.append("# System RT HIL Report")
    lines.append("")
    lines.append(f"- Start (UTC): {report['started_at_utc']}")
    lines.append(f"- End (UTC): {report['finished_at_utc']}")
    lines.append(f"- Target: `{report['host']}:{report['port']}`")
    lines.append(f"- Nominal duration: `{report['config']['nominal_duration_s']} s`")
    lines.append(f"- Stress duration: `{report['config']['stress_duration_s']} s`")
    lines.append(
        f"- PiUartRx threshold: "
        f"`p95 <= {report['config']['pi_rx_p95_max_us']} us`, "
        f"`p99 <= {report['config']['pi_rx_p99_max_us']} us`"
    )
    lines.append("")
    if report.get("error"):
        lines.append(f"- Error: `{report['error']}`")
        lines.append("")

    for stage in report.get("stages", []):
        lines.append(f"## {stage.get('name')}")
        jitter = stage.get("jitter", {})
        lines.append(f"- jitter samples: `{jitter.get('samples', 0)}`")
        lines.append(f"- expected samples: `{stage.get('expected_jitter_samples', 0)}`")
        lines.append(f"- coverage: `{stage.get('coverage_pct', 0.0)}%`")
        lines.append(f"- worst max jitter us: `{jitter.get('worst_max_us', 0)}`")
        lines.append(f"- coverage ok: `{'yes' if stage.get('coverage_ok', False) else 'no'}`")
        lines.append(f"- snapshot ok: `{'yes' if stage.get('snapshot_ok', False) else 'no'}`")
        if "pi_uart_rx_threshold_ok" in stage:
            lines.append(f"- PiUartRx threshold ok: `{'yes' if stage.get('pi_uart_rx_threshold_ok', False) else 'no'}`")
            lines.append(f"- PiUartRx threshold detail: `{stage.get('pi_uart_rx_threshold_detail', '')}`")
        lines.append("")
        if jitter.get("tasks"):
            lines.append("| Task | p95 us | p99 us | mean us | max observed us |")
            lines.append("|---|---:|---:|---:|---:|")
            for task, metrics in sorted(jitter["tasks"].items()):
                lines.append(
                    "| %s | %.2f | %.2f | %.2f | %d |"
                    % (
                        task,
                        metrics.get("current_p95_us", 0.0),
                        metrics.get("current_p99_us", 0.0),
                        metrics.get("current_mean_us", 0.0),
                        metrics.get("max_observed_us", 0),
                    )
                )
            lines.append("")

    ensure_parent(path)
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")


def parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="HIL runner for sys.rt/sys.stack/sys.jitter")
    parser.add_argument("--host", default="esp32-salus.local")
    parser.add_argument("--port", type=int, default=23)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--nominal-duration-s", type=float, default=120.0)
    parser.add_argument("--stress-duration-s", type=float, default=120.0)
    parser.add_argument("--jitter-period-ms", type=int, default=500)
    parser.add_argument("--stress-pidtrace-ms", type=int, default=100)
    parser.add_argument("--keepalive-s", type=float, default=30.0)
    parser.add_argument("--max-silence-s", type=float, default=20.0)
    parser.add_argument("--progress-period-s", type=float, default=15.0)
    parser.add_argument("--max-jitter-gap-s", type=float, default=8.0)
    parser.add_argument("--out-progress", default="")
    parser.add_argument("--min-stage-coverage-pct", type=float, default=90.0)
    parser.add_argument("--pi-rx-p95-max-us", type=float, default=800.0)
    parser.add_argument("--pi-rx-p99-max-us", type=float, default=2000.0)
    parser.add_argument("--interactive-prompts", action="store_true")
    parser.add_argument("--out-json", default="artifacts/system_rt_hil_report.json")
    parser.add_argument("--out-raw", default="artifacts/system_rt_hil_raw.log")
    return parser.parse_args(argv)


def maybe_prompt(enabled: bool, text: str) -> None:
    if not enabled:
        return
    print(text, flush=True)
    try:
        input("Presiona ENTER para continuar... ")
    except EOFError:
        pass


def run(args: argparse.Namespace) -> int:
    started = now_iso()
    report: Dict[str, Any] = {
        "started_at_utc": started,
        "finished_at_utc": "",
        "host": args.host,
        "port": args.port,
        "config": {
            "nominal_duration_s": args.nominal_duration_s,
            "stress_duration_s": args.stress_duration_s,
            "jitter_period_ms": args.jitter_period_ms,
            "stress_pidtrace_ms": args.stress_pidtrace_ms,
            "keepalive_s": args.keepalive_s,
            "max_silence_s": args.max_silence_s,
            "progress_period_s": args.progress_period_s,
            "max_jitter_gap_s": args.max_jitter_gap_s,
            "out_progress": args.out_progress,
            "min_stage_coverage_pct": args.min_stage_coverage_pct,
            "pi_rx_p95_max_us": args.pi_rx_p95_max_us,
            "pi_rx_p99_max_us": args.pi_rx_p99_max_us,
            "interactive_prompts": args.interactive_prompts,
        },
        "preflight": {},
        "stages": [],
        "validation_errors": [],
        "error": None,
    }
    raw_records: List[Dict[str, str]] = []
    runner = TelnetRunner(args.host, args.port, args.timeout)

    try:
        banner = runner.connect()
        report["preflight"]["banner_lines"] = banner
        for line in banner:
            raw_records.append({"stage": "banner", "line": line})

        # Baseline setup and snapshots.
        for cmd in ("drive.log off", "drive.log pid off", "sys.jitter off", "net.status", "comms.status",
                    "speed.status", "spid.status", "sys.rt", "sys.stack"):
            lines = runner.send(cmd)
            for line in lines:
                raw_records.append({"stage": "preflight", "line": line})

        maybe_prompt(
            args.interactive_prompts,
            "Etapa nominal: deja el sistema en condiciones normales de operación (sin carga de logging).",
        )
        require_sys_reset(runner, "TC-RT-01_nominal", raw_records)
        nominal = collect_stage(
            runner,
            "TC-RT-01_nominal",
            args.nominal_duration_s,
            args.jitter_period_ms,
            args.keepalive_s,
            args.max_silence_s,
            args.progress_period_s,
            args.max_jitter_gap_s,
            args.out_progress,
            args.min_stage_coverage_pct,
            raw_records,
        )
        report["stages"].append(nominal)
        nominal_thr_ok, nominal_thr_detail = validate_pi_uart_rx_thresholds(
            nominal, args.pi_rx_p95_max_us, args.pi_rx_p99_max_us
        )
        nominal["pi_uart_rx_threshold_ok"] = nominal_thr_ok
        nominal["pi_uart_rx_threshold_detail"] = nominal_thr_detail
        if not nominal.get("coverage_ok", False):
            report["validation_errors"].append(
                "TC-RT-01_nominal low coverage "
                f"{nominal.get('coverage_pct', 0.0)}% < {args.min_stage_coverage_pct}%"
            )
        if not nominal.get("snapshot_ok", False):
            report["validation_errors"].append("TC-RT-01_nominal missing sys.rt/sys.stack snapshot")
        if not nominal_thr_ok:
            report["validation_errors"].append(f"TC-RT-01_nominal {nominal_thr_detail}")

        maybe_prompt(
            args.interactive_prompts,
            "Etapa estrés: ejecuta maniobras representativas mientras se activa trace PID.",
        )
        require_sys_reset(runner, "TC-RT-02_stress_logging", raw_records)
        runner.send(f"drive.log pid on {args.stress_pidtrace_ms}")
        stress = collect_stage(
            runner,
            "TC-RT-02_stress_logging",
            args.stress_duration_s,
            args.jitter_period_ms,
            args.keepalive_s,
            args.max_silence_s,
            args.progress_period_s,
            args.max_jitter_gap_s,
            args.out_progress,
            args.min_stage_coverage_pct,
            raw_records,
        )
        report["stages"].append(stress)
        stress_thr_ok, stress_thr_detail = validate_pi_uart_rx_thresholds(
            stress, args.pi_rx_p95_max_us, args.pi_rx_p99_max_us
        )
        stress["pi_uart_rx_threshold_ok"] = stress_thr_ok
        stress["pi_uart_rx_threshold_detail"] = stress_thr_detail
        if not stress.get("coverage_ok", False):
            report["validation_errors"].append(
                "TC-RT-02_stress_logging low coverage "
                f"{stress.get('coverage_pct', 0.0)}% < {args.min_stage_coverage_pct}%"
            )
        if not stress.get("snapshot_ok", False):
            report["validation_errors"].append("TC-RT-02_stress_logging missing sys.rt/sys.stack snapshot")
        if not stress_thr_ok:
            report["validation_errors"].append(f"TC-RT-02_stress_logging {stress_thr_detail}")
        runner.send("drive.log pid off")
        runner.send("drive.log off")

    except KeyboardInterrupt:
        report["error"] = "interrupted_by_user"
    except Exception as exc:
        report["error"] = str(exc)
    finally:
        runner.close()
        report["finished_at_utc"] = now_iso()

    if report["error"] is None and report["validation_errors"]:
        report["error"] = "; ".join(report["validation_errors"])

    out_json = args.out_json
    out_md = os.path.splitext(out_json)[0] + ".md"
    ensure_parent(out_json)
    with open(out_json, "w", encoding="utf-8") as fh:
        json.dump(report, fh, ensure_ascii=False, indent=2)
    write_markdown(out_md, report)

    ensure_parent(args.out_raw)
    with open(args.out_raw, "w", encoding="utf-8") as fh:
        for item in raw_records:
            fh.write(f"[{item['stage']}] {item['line']}\n")

    print("System RT HIL report:", out_json)
    print("System RT HIL markdown:", out_md)
    print("System RT raw log:", args.out_raw)

    if report["error"]:
        print("Execution error:", report["error"])
        return 2
    return 0


def main(argv: List[str]) -> int:
    args = parse_args(argv)
    return run(args)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
