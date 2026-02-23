from tools.telnet_ui.app.parser import ParsedEvent
from tools.telnet_ui.app.state_store import TelnetUiStateStore


def _event(line, tags, section, fields=None):
    return ParsedEvent(line=line, tags=tags, section=section, fields=fields or {}, message="")


def test_recent_logs_ring_buffer_limit():
    store = TelnetUiStateStore(recent_logs_limit=2)

    store.apply_event(_event("[NET] mode=STA", ["NET"], "net", {"mode": "STA"}))
    store.apply_event(_event("[SPD][STATUS] speed=1", ["SPD", "STATUS"], "speed", {"speed": "1"}))
    store.apply_event(_event("[PID] kp=1", ["PID"], "pid", {"kp": "1"}))

    snapshot = store.snapshot()
    assert len(snapshot["recent_logs"]) == 2
    assert snapshot["recent_logs"][0]["line"] == "[SPD][STATUS] speed=1"


def test_stream_patch_updates_speed_stream_state():
    store = TelnetUiStateStore()
    patch = store.apply_event(
        _event(
            "[SPD][STREAM] ON periodo=500ms",
            ["SPD", "STREAM"],
            "streams",
            {"stream": "speed", "state": "ON", "periodo": "500ms"},
        )
    )

    assert patch["streams"]["speed"]["state"] == "ON"
    assert patch["streams"]["speed"]["period_ms"] == 500
