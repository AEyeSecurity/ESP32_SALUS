import pytest

from tools.telnet_ui.app.command_catalog import get_command, render_command


def test_render_command_with_arg():
    rendered = render_command("steer.offset", {"deg": -2.5})
    assert rendered == "steer.offset -2.5"


def test_sensitive_command_requires_confirmation_flag():
    cmd = get_command("spid.reset")
    assert cmd.requires_confirmation is True


def test_render_command_missing_required_arg_raises():
    with pytest.raises(ValueError):
        render_command("pid.kp", {})


def test_render_optional_stream_period():
    assert render_command("speed.stream.on", {}) == "speed.stream on"
    assert render_command("speed.stream.on", {"period_ms": 500}) == "speed.stream on 500"
