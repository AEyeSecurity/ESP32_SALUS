import asyncio
from pathlib import Path
from typing import Any, Dict, Optional, Set

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from .command_catalog import bootstrap_commands, get_command, list_commands, render_command
from .models import CommandExecutionResult, CommandRequest, ConnectRequest, DisconnectResponse, RawCommandRequest
from .parser import TelnetLineParser
from .state_store import TelnetUiStateStore
from .telnet_client import AsyncTelnetClient


class WebSocketHub:
    def __init__(self) -> None:
        self._clients: Set[WebSocket] = set()
        self._lock = asyncio.Lock()

    async def connect(self, websocket: WebSocket) -> None:
        await websocket.accept()
        async with self._lock:
            self._clients.add(websocket)

    async def disconnect(self, websocket: WebSocket) -> None:
        async with self._lock:
            if websocket in self._clients:
                self._clients.remove(websocket)

    async def broadcast(self, event_type: str, data: Dict[str, Any]) -> None:
        payload = {"type": event_type, "data": data}
        stale: Set[WebSocket] = set()
        async with self._lock:
            clients = list(self._clients)
        for client in clients:
            try:
                await client.send_json(payload)
            except Exception:
                stale.add(client)
        if stale:
            async with self._lock:
                for client in stale:
                    self._clients.discard(client)


class TelnetUiService:
    def __init__(self, telnet_client: Optional[AsyncTelnetClient] = None) -> None:
        self.state_store = TelnetUiStateStore(recent_logs_limit=500)
        self.parser = TelnetLineParser()
        self.ws_hub = WebSocketHub()
        self.telnet_client = telnet_client

    async def init_telnet_client(self) -> None:
        async def on_line(line: str) -> None:
            event = self.parser.parse_line(line)
            patch = self.state_store.apply_event(event)
            await self.ws_hub.broadcast("raw_line", {"line": line})
            await self.ws_hub.broadcast(
                "parsed_event",
                {
                    "line": event.line,
                    "tags": event.tags,
                    "section": event.section,
                    "fields": event.fields,
                    "message": event.message,
                },
            )
            await self.ws_hub.broadcast("state_patch", patch)

        async def on_connection(connected: bool,
                                host: Optional[str],
                                port: Optional[int],
                                reason: str) -> None:
            patch = self.state_store.set_connection(connected, host, port, reason)
            await self.ws_hub.broadcast("connection", patch["connection"])
            await self.ws_hub.broadcast("state_patch", patch)

        async def on_error(message: str) -> None:
            await self.ws_hub.broadcast("error", {"message": message})

        if self.telnet_client is None:
            self.telnet_client = AsyncTelnetClient(
                on_line=on_line,
                on_connection=on_connection,
                on_error=on_error,
                keepalive_interval_sec=120,
                polling_interval_sec=2,
            )
            return

        if hasattr(self.telnet_client, "set_callbacks"):
            self.telnet_client.set_callbacks(on_line, on_connection, on_error)


def _model_dump(model: Any) -> Dict[str, Any]:
    if hasattr(model, "model_dump"):
        return model.model_dump()
    return model.dict()


def create_app(telnet_client: Optional[AsyncTelnetClient] = None) -> FastAPI:
    service = TelnetUiService(telnet_client=telnet_client)
    app = FastAPI(title="ESP32 Salus Telnet UI", version="0.1.0")

    static_dir = Path(__file__).resolve().parent.parent / "static"
    app.mount("/static", StaticFiles(directory=str(static_dir)), name="static")

    @app.on_event("startup")
    async def _startup() -> None:
        await service.init_telnet_client()

    @app.on_event("shutdown")
    async def _shutdown() -> None:
        if service.telnet_client and service.telnet_client.is_connected:
            await service.telnet_client.disconnect("shutdown")

    @app.get("/")
    async def index() -> FileResponse:
        return FileResponse(str(static_dir / "index.html"))

    @app.post("/api/connect")
    async def api_connect(payload: ConnectRequest) -> Dict[str, Any]:
        if service.telnet_client is None:
            await service.init_telnet_client()

        assert service.telnet_client is not None

        try:
            await service.telnet_client.connect(payload.host, payload.port)
            for command in bootstrap_commands():
                await service.telnet_client.send_command(command)
        except Exception as exc:
            raise HTTPException(status_code=502, detail="Connection failed: %s" % exc)

        if service.telnet_client.is_connected:
            patch = service.state_store.set_connection(True, payload.host, payload.port, "connected")
            await service.ws_hub.broadcast("connection", patch["connection"])
            await service.ws_hub.broadcast("state_patch", patch)

        return service.state_store.snapshot()["connection"]

    @app.post("/api/disconnect")
    async def api_disconnect() -> Dict[str, Any]:
        if service.telnet_client and service.telnet_client.is_connected:
            await service.telnet_client.disconnect("manual_disconnect")
        patch = service.state_store.set_connection(False, reason="manual_disconnect")
        await service.ws_hub.broadcast("connection", patch["connection"])
        await service.ws_hub.broadcast("state_patch", patch)
        response = DisconnectResponse(disconnected=True)
        return _model_dump(response)

    @app.get("/api/status")
    async def api_status() -> Dict[str, Any]:
        return service.state_store.snapshot()

    @app.get("/api/commands")
    async def api_commands() -> Dict[str, Any]:
        return {"commands": list_commands()}

    @app.post("/api/command")
    async def api_command(payload: CommandRequest) -> Dict[str, Any]:
        if service.telnet_client is None or not service.telnet_client.is_connected:
            raise HTTPException(status_code=409, detail="Not connected")

        try:
            command_def = get_command(payload.command_id)
        except KeyError as exc:
            raise HTTPException(status_code=404, detail=str(exc))

        if command_def.requires_confirmation and not payload.confirmed:
            raise HTTPException(status_code=400, detail="confirmation_required")

        try:
            rendered_command = render_command(payload.command_id, payload.args)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc))

        try:
            await service.telnet_client.send_command(rendered_command)
            for follow_up in command_def.poll_after:
                await service.telnet_client.send_command(follow_up)
        except Exception as exc:
            raise HTTPException(status_code=500, detail="Command send failed: %s" % exc)

        result = CommandExecutionResult(
            command_id=payload.command_id,
            rendered_command=rendered_command,
            queued=True,
        )
        result_dict = _model_dump(result)
        await service.ws_hub.broadcast("command_result", result_dict)
        return result_dict

    @app.post("/api/raw-command")
    async def api_raw_command(payload: RawCommandRequest) -> Dict[str, Any]:
        if service.telnet_client is None or not service.telnet_client.is_connected:
            raise HTTPException(status_code=409, detail="Not connected")

        command = payload.command.strip()
        if not command:
            raise HTTPException(status_code=400, detail="Empty command")
        try:
            await service.telnet_client.send_command(command)
        except Exception as exc:
            raise HTTPException(status_code=500, detail="Command send failed: %s" % exc)

        result = {"command": command, "queued": True}
        await service.ws_hub.broadcast("command_result", result)
        return result

    @app.websocket("/ws")
    async def ws_endpoint(websocket: WebSocket) -> None:
        await service.ws_hub.connect(websocket)
        snapshot = service.state_store.snapshot()
        await websocket.send_json({"type": "connection", "data": snapshot["connection"]})
        await websocket.send_json({"type": "state_patch", "data": snapshot})
        try:
            while True:
                await websocket.receive_text()
        except WebSocketDisconnect:
            pass
        finally:
            await service.ws_hub.disconnect(websocket)

    app.state.telnet_ui_service = service
    return app


app = create_app()
