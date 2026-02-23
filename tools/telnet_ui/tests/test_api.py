from fastapi.testclient import TestClient

from tools.telnet_ui.app.main import create_app


class FakeTelnetClient:
    def __init__(self):
        self._on_line = None
        self._on_connection = None
        self._on_error = None
        self._connected = False
        self._host = None
        self._port = None
        self.sent_commands = []

    def set_callbacks(self, on_line, on_connection, on_error):
        self._on_line = on_line
        self._on_connection = on_connection
        self._on_error = on_error

    @property
    def is_connected(self):
        return self._connected

    @property
    def host(self):
        return self._host

    @property
    def port(self):
        return self._port

    async def connect(self, host, port):
        self._connected = True
        self._host = host
        self._port = port
        if self._on_connection:
            await self._on_connection(True, host, port, "connected")

    async def disconnect(self, reason="manual_disconnect"):
        self._connected = False
        if self._on_connection:
            await self._on_connection(False, self._host, self._port, reason)

    async def send_command(self, command):
        if not self._connected:
            raise RuntimeError("not connected")
        self.sent_commands.append(command)



def _new_client():
    fake = FakeTelnetClient()
    app = create_app(telnet_client=fake)
    client = TestClient(app)
    return client, fake


def test_connect_bootstrap_and_disconnect():
    client, fake = _new_client()

    response = client.post("/api/connect", json={"host": "esp32-salus.local", "port": 23})
    assert response.status_code == 200
    assert response.json()["connected"] is True

    assert "net.status" in fake.sent_commands
    assert "spid.status" in fake.sent_commands

    response = client.post("/api/disconnect")
    assert response.status_code == 200
    assert response.json()["disconnected"] is True


def test_command_confirmation_required():
    client, _ = _new_client()
    client.post("/api/connect", json={"host": "esp32-salus.local", "port": 23})

    response = client.post(
        "/api/command",
        json={"command_id": "spid.reset", "args": {}, "confirmed": False},
    )
    assert response.status_code == 400
    assert response.json()["detail"] == "confirmation_required"


def test_raw_command_endpoint():
    client, fake = _new_client()
    client.post("/api/connect", json={"host": "esp32-salus.local", "port": 23})

    response = client.post("/api/raw-command", json={"command": "speed.status"})
    assert response.status_code == 200
    assert fake.sent_commands[-1] == "speed.status"


def test_websocket_receives_connection_event():
    client, _ = _new_client()

    with client.websocket_connect("/ws") as ws:
        first = ws.receive_json()
        second = ws.receive_json()
        assert first["type"] in ("connection", "state_patch")
        assert second["type"] in ("connection", "state_patch")

        client.post("/api/connect", json={"host": "esp32-salus.local", "port": 23})

        received_connected = False
        for _ in range(6):
            event = ws.receive_json()
            if event["type"] == "connection" and event["data"].get("connected") is True:
                received_connected = True
                break

        assert received_connected is True
