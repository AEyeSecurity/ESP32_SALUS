from tools.telnet_ui.app.parser import TelnetLineParser


def test_parse_net_status_line():
    parser = TelnetLineParser()
    event = parser.parse_line("[NET] mode=STA ssid=esp32 ip=192.168.1.10 hostname=esp32-salus telnetPort=23")

    assert event.section == "net"
    assert event.fields["mode"] == "STA"
    assert event.fields["ip"] == "192.168.1.10"


def test_parse_spid_stream_line():
    parser = TelnetLineParser()
    event = parser.parse_line("[SPID][STREAM] ON periodo=200ms")

    assert event.section == "streams"
    assert event.fields["stream"] == "spid"
    assert event.fields["state"] == "ON"
    assert event.fields["periodo"] == "200ms"


def test_parse_unknown_line():
    parser = TelnetLineParser()
    event = parser.parse_line("linea libre sin prefijo")

    assert event.section == "unknown"
    assert event.tags == []
