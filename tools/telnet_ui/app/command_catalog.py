from typing import Any, Dict, List

from .models import CommandArgDefinition, CommandDefinition


def _arg(name: str,
         arg_type: str,
         label: str,
         required: bool = True,
         min_value: float = None,
         max_value: float = None,
         step: float = None,
         default: Any = None) -> CommandArgDefinition:
    return CommandArgDefinition(
        name=name,
        type=arg_type,
        label=label,
        required=required,
        min=min_value,
        max=max_value,
        step=step,
        default=default,
    )


_COMMANDS: List[CommandDefinition] = [
    CommandDefinition(
        id="net.status",
        label="Estado de red",
        group="net",
        template="net.status",
    ),
    CommandDefinition(
        id="steer.status",
        label="Estado dirección",
        group="steer",
        template="steer.status",
    ),
    CommandDefinition(
        id="steer.help",
        label="Ayuda dirección",
        group="steer",
        template="steer.help",
    ),
    CommandDefinition(
        id="steer.calibrate",
        label="Calibrar dirección",
        group="steer",
        template="steer.calibrate",
        requires_confirmation=True,
        confirm_message="Iniciará la calibración mecánica de dirección.",
        poll_after=["steer.status"],
    ),
    CommandDefinition(
        id="steer.offset",
        label="Aplicar offset dirección",
        group="steer",
        template="steer.offset {deg}",
        args=[_arg("deg", "float", "Offset (deg)", min_value=-180.0, max_value=180.0, step=0.1)],
        poll_after=["steer.status"],
    ),
    CommandDefinition(
        id="steer.reset",
        label="Reset calibración",
        group="steer",
        template="steer.reset",
        requires_confirmation=True,
        confirm_message="Borrará calibración de dirección en NVS.",
        poll_after=["steer.status"],
    ),
    CommandDefinition(
        id="steer.clear",
        label="Clear calibración",
        group="steer",
        template="steer.clear",
        requires_confirmation=True,
        confirm_message="Borrará calibración de dirección en NVS.",
        poll_after=["steer.status"],
    ),
    CommandDefinition(
        id="pid.status",
        label="Estado PID dirección",
        group="pid",
        template="pid.status",
    ),
    CommandDefinition(
        id="pid.help",
        label="Ayuda PID dirección",
        group="pid",
        template="pid.help",
    ),
    CommandDefinition(
        id="pid.set",
        label="PID set (kp,ki,kd)",
        group="pid",
        template="pid.set {kp} {ki} {kd}",
        args=[
            _arg("kp", "float", "Kp", step=0.001),
            _arg("ki", "float", "Ki", step=0.001),
            _arg("kd", "float", "Kd", step=0.001),
        ],
        poll_after=["pid.status"],
    ),
    CommandDefinition(
        id="pid.kp",
        label="PID Kp",
        group="pid",
        template="pid.kp {value}",
        args=[_arg("value", "float", "Kp", step=0.001)],
        poll_after=["pid.status"],
    ),
    CommandDefinition(
        id="pid.ki",
        label="PID Ki",
        group="pid",
        template="pid.ki {value}",
        args=[_arg("value", "float", "Ki", step=0.001)],
        poll_after=["pid.status"],
    ),
    CommandDefinition(
        id="pid.kd",
        label="PID Kd",
        group="pid",
        template="pid.kd {value}",
        args=[_arg("value", "float", "Kd", step=0.001)],
        poll_after=["pid.status"],
    ),
    CommandDefinition(
        id="pid.deadband",
        label="PID deadband (%)",
        group="pid",
        template="pid.deadband {value}",
        args=[_arg("value", "float", "Deadband (%)", min_value=0.0, max_value=100.0, step=0.01)],
        poll_after=["pid.status"],
    ),
    CommandDefinition(
        id="pid.minactive",
        label="PID min active (%)",
        group="pid",
        template="pid.minactive {value}",
        args=[_arg("value", "float", "Min active (%)", min_value=0.0, max_value=100.0, step=0.01)],
        poll_after=["pid.status"],
    ),
    CommandDefinition(
        id="spid.status",
        label="Estado PID velocidad",
        group="spid",
        template="spid.status",
    ),
    CommandDefinition(
        id="spid.help",
        label="Ayuda PID velocidad",
        group="spid",
        template="spid.help",
    ),
    CommandDefinition(
        id="spid.set",
        label="SPID set (kp,ki,kd)",
        group="spid",
        template="spid.set {kp} {ki} {kd}",
        args=[
            _arg("kp", "float", "Kp", step=0.001),
            _arg("ki", "float", "Ki", step=0.001),
            _arg("kd", "float", "Kd", step=0.001),
        ],
        poll_after=["spid.status"],
    ),
    CommandDefinition(
        id="spid.kp",
        label="SPID Kp",
        group="spid",
        template="spid.kp {value}",
        args=[_arg("value", "float", "Kp", step=0.001)],
        poll_after=["spid.status"],
    ),
    CommandDefinition(
        id="spid.ki",
        label="SPID Ki",
        group="spid",
        template="spid.ki {value}",
        args=[_arg("value", "float", "Ki", step=0.001)],
        poll_after=["spid.status"],
    ),
    CommandDefinition(
        id="spid.kd",
        label="SPID Kd",
        group="spid",
        template="spid.kd {value}",
        args=[_arg("value", "float", "Kd", step=0.001)],
        poll_after=["spid.status"],
    ),
    CommandDefinition(
        id="spid.ramp",
        label="SPID Ramp (m/s²)",
        group="spid",
        template="spid.ramp {value}",
        args=[_arg("value", "float", "Ramp (m/s²)", min_value=0.01, max_value=50.0, step=0.01)],
        poll_after=["spid.status"],
    ),
    CommandDefinition(
        id="spid.max",
        label="SPID Max (m/s)",
        group="spid",
        template="spid.max {value}",
        args=[_arg("value", "float", "Max (m/s)", min_value=0.1, max_value=4.17, step=0.01)],
        poll_after=["spid.status"],
    ),
    CommandDefinition(
        id="spid.brakecap",
        label="SPID BrakeCap (%)",
        group="spid",
        template="spid.brakecap {value}",
        args=[_arg("value", "float", "Brake cap (%)", min_value=0.0, max_value=100.0, step=0.1)],
        poll_after=["spid.status"],
    ),
    CommandDefinition(
        id="spid.hys",
        label="SPID Hysteresis (m/s)",
        group="spid",
        template="spid.hys {value}",
        args=[_arg("value", "float", "Hysteresis (m/s)", min_value=0.0, max_value=5.0, step=0.01)],
        poll_after=["spid.status"],
    ),
    CommandDefinition(
        id="spid.save",
        label="SPID save NVS",
        group="spid",
        template="spid.save",
        requires_confirmation=True,
        confirm_message="Guardará los parámetros SPID en NVS.",
        poll_after=["spid.status"],
    ),
    CommandDefinition(
        id="spid.reset",
        label="SPID reset defaults",
        group="spid",
        template="spid.reset",
        requires_confirmation=True,
        confirm_message="Restaurará los parámetros SPID por defecto.",
        poll_after=["spid.status"],
    ),
    CommandDefinition(
        id="spid.stream.on",
        label="SPID stream ON",
        group="spid",
        args=[_arg("period_ms", "int", "Periodo (ms)", required=False, min_value=50, max_value=5000, step=10, default=200)],
        poll_after=["spid.status"],
    ),
    CommandDefinition(
        id="spid.stream.off",
        label="SPID stream OFF",
        group="spid",
        template="spid.stream off",
        poll_after=["spid.status"],
    ),
    CommandDefinition(
        id="speed.status",
        label="Estado velocidad Hall",
        group="speed",
        template="speed.status",
    ),
    CommandDefinition(
        id="speed.reset",
        label="Reset contadores Hall",
        group="speed",
        template="speed.reset",
        requires_confirmation=True,
        confirm_message="Reiniciará contadores Hall.",
        poll_after=["speed.status"],
    ),
    CommandDefinition(
        id="speed.uart",
        label="Speed UART",
        group="speed",
        template="speed.uart",
    ),
    CommandDefinition(
        id="speed.stream.on",
        label="Speed stream ON",
        group="speed",
        args=[_arg("period_ms", "int", "Periodo (ms)", required=False, min_value=20, max_value=5000, step=10, default=200)],
        poll_after=["speed.status"],
    ),
    CommandDefinition(
        id="speed.stream.off",
        label="Speed stream OFF",
        group="speed",
        template="speed.stream off",
        poll_after=["speed.status"],
    ),
    CommandDefinition(
        id="comms.status",
        label="Estado COMMS",
        group="comms",
        template="comms.status",
    ),
    CommandDefinition(
        id="comms.reset",
        label="Reset contadores COMMS",
        group="comms",
        template="comms.reset",
        requires_confirmation=True,
        confirm_message="Reiniciará contadores RX de PI COMMS.",
        poll_after=["comms.status"],
    ),
    CommandDefinition(
        id="sys.rt",
        label="RT snapshot",
        group="sys",
        template="sys.rt",
    ),
    CommandDefinition(
        id="sys.stack",
        label="Stack snapshot",
        group="sys",
        template="sys.stack",
    ),
    CommandDefinition(
        id="sys.jitter.on",
        label="Jitter stream ON",
        group="sys",
        args=[_arg("period_ms", "int", "Periodo (ms)", required=False, min_value=50, max_value=5000, step=10, default=500)],
        poll_after=["sys.rt"],
    ),
    CommandDefinition(
        id="sys.jitter.off",
        label="Jitter stream OFF",
        group="sys",
        template="sys.jitter off",
        poll_after=["sys.rt"],
    ),
    CommandDefinition(
        id="sys.reset",
        label="Reset métricas SYS",
        group="sys",
        template="sys.reset keep",
        requires_confirmation=True,
        confirm_message="Reiniciará los acumulados de diagnóstico RT/stack.",
        poll_after=["sys.rt", "sys.stack"],
    ),
    CommandDefinition(
        id="drive.log",
        label="Estado drive.log",
        group="drive",
        template="drive.log",
    ),
    CommandDefinition(
        id="drive.log.on",
        label="drive.log ON",
        group="drive",
        template="drive.log on",
        poll_after=["drive.log"],
    ),
    CommandDefinition(
        id="drive.log.off",
        label="drive.log OFF",
        group="drive",
        template="drive.log off",
        poll_after=["drive.log"],
    ),
]


_COMMANDS_BY_ID: Dict[str, CommandDefinition] = {cmd.id: cmd for cmd in _COMMANDS}


def list_commands() -> List[Dict[str, Any]]:
    return [cmd.model_dump() if hasattr(cmd, "model_dump") else cmd.dict() for cmd in _COMMANDS]


def get_command(command_id: str) -> CommandDefinition:
    if command_id not in _COMMANDS_BY_ID:
        raise KeyError("Unknown command_id: %s" % command_id)
    return _COMMANDS_BY_ID[command_id]


def _require_arg(args: Dict[str, Any], name: str) -> Any:
    if name not in args or args[name] in (None, ""):
        raise ValueError("Missing argument: %s" % name)
    return args[name]


def _fmt_number(value: Any) -> str:
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        text = ("%.6f" % value).rstrip("0").rstrip(".")
        return text if text else "0"
    text = str(value).strip()
    if not text:
        raise ValueError("Numeric argument is empty")
    return text


def render_command(command_id: str, args: Dict[str, Any]) -> str:
    cmd = get_command(command_id)

    if command_id == "speed.stream.on":
        period = args.get("period_ms")
        if period in (None, ""):
            return "speed.stream on"
        return "speed.stream on %s" % _fmt_number(period)

    if command_id == "spid.stream.on":
        period = args.get("period_ms")
        if period in (None, ""):
            return "spid.stream on"
        return "spid.stream on %s" % _fmt_number(period)

    if command_id == "sys.jitter.on":
        period = args.get("period_ms")
        if period in (None, ""):
            return "sys.jitter on"
        return "sys.jitter on %s" % _fmt_number(period)

    if not cmd.template:
        raise ValueError("Command has no template: %s" % command_id)

    if not cmd.args:
        return cmd.template

    rendered_args: Dict[str, str] = {}
    for arg in cmd.args:
        if arg.required:
            rendered_args[arg.name] = _fmt_number(_require_arg(args, arg.name))
        else:
            if arg.name in args and args[arg.name] not in (None, ""):
                rendered_args[arg.name] = _fmt_number(args[arg.name])

    try:
        return cmd.template.format(**rendered_args)
    except KeyError as exc:
        raise ValueError("Missing argument for template: %s" % exc)


def bootstrap_commands() -> List[str]:
    return [
        "net.status",
        "steer.status",
        "pid.status",
        "spid.status",
        "comms.status",
        "speed.status",
        "sys.rt",
        "drive.log",
    ]


def polling_commands() -> List[str]:
    return ["comms.status", "speed.status", "spid.status"]
