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


def test_parse_sys_line():
    parser = TelnetLineParser()
    event = parser.parse_line("[SYS][JITTER] tsMs=123 worstMaxUs=50 tasks=2 OTA=5/20 PID=8/30")

    assert event.section == "system"
    assert event.fields["tsMs"] == "123"
    assert event.fields["worstMaxUs"] == "50"


def test_parse_spid_status_nested_groups_and_duplicate_keys():
    parser = TelnetLineParser()
    line = (
        "[SPID] state{init=Y en=Y fb=Y failsafe=N overspeed=N mode=NORMAL} "
        "targetRaw=1.00m/s tune{kp=1.0 ki=2.0 kd=0.0} "
        "cfg{max=4.17m/s ramp=2.0m/s2 ff{en=Y b0=0.0 bmax=55.0 du=35.0 dd=45.0 min=0.10m/s} "
        "flgr=1200ms iunw=0.35 dfhz=3.0 cap=30.0% hys=0.3m/s} "
        "sat=12.3 sat=Y"
    )

    event = parser.parse_line(line)

    assert event.section == "spid"
    assert event.fields["state.mode"] == "NORMAL"
    assert event.fields["tune.kp"] == "1.0"
    assert event.fields["cfg.max"] == "4.17m/s"
    assert event.fields["cfg.ff.en"] == "Y"
    assert event.fields["cfg.ff.bmax"] == "55.0"
    assert event.fields["cfg.hys"] == "0.3m/s"
    assert event.fields["sat"] == "12.3"
    assert event.fields["sat__dup2"] == "Y"
