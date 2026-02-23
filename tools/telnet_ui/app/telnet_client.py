import asyncio
from typing import Awaitable, Callable, List, Optional


LineCallback = Callable[[str], Awaitable[None]]
ConnectionCallback = Callable[[bool, Optional[str], Optional[int], str], Awaitable[None]]
ErrorCallback = Callable[[str], Awaitable[None]]


class AsyncTelnetClient:
    def __init__(self,
                 on_line: LineCallback,
                 on_connection: ConnectionCallback,
                 on_error: ErrorCallback,
                 keepalive_interval_sec: int = 120,
                 polling_interval_sec: int = 2) -> None:
        self._on_line = on_line
        self._on_connection = on_connection
        self._on_error = on_error

        self._keepalive_interval_sec = keepalive_interval_sec
        self._polling_interval_sec = polling_interval_sec

        self._host: Optional[str] = None
        self._port: Optional[int] = None
        self._reader: Optional[asyncio.StreamReader] = None
        self._writer: Optional[asyncio.StreamWriter] = None

        self._send_queue: "asyncio.Queue[str]" = asyncio.Queue()

        self._reader_task: Optional[asyncio.Task] = None
        self._writer_task: Optional[asyncio.Task] = None
        self._keepalive_task: Optional[asyncio.Task] = None
        self._polling_task: Optional[asyncio.Task] = None

        self._lock = asyncio.Lock()

    def set_callbacks(self, on_line: LineCallback, on_connection: ConnectionCallback, on_error: ErrorCallback) -> None:
        self._on_line = on_line
        self._on_connection = on_connection
        self._on_error = on_error

    @property
    def host(self) -> Optional[str]:
        return self._host

    @property
    def port(self) -> Optional[int]:
        return self._port

    @property
    def is_connected(self) -> bool:
        return self._writer is not None and (not self._writer.is_closing())

    async def connect(self, host: str, port: int) -> None:
        async with self._lock:
            await self._disconnect_locked("reconnecting")

            try:
                reader, writer = await asyncio.wait_for(asyncio.open_connection(host, port), timeout=8)
            except Exception as exc:
                await self._on_connection(False, host, port, "connect_error")
                await self._on_error("No se pudo conectar a %s:%s (%s)" % (host, port, exc))
                raise

            self._host = host
            self._port = port
            self._reader = reader
            self._writer = writer

            self._reader_task = asyncio.create_task(self._reader_loop(), name="telnet-reader")
            self._writer_task = asyncio.create_task(self._writer_loop(), name="telnet-writer")
            self._keepalive_task = asyncio.create_task(self._keepalive_loop(), name="telnet-keepalive")
            self._polling_task = asyncio.create_task(self._polling_loop(), name="telnet-polling")

            await self._on_connection(True, host, port, "connected")

    async def disconnect(self, reason: str = "manual_disconnect") -> None:
        async with self._lock:
            await self._disconnect_locked(reason)

    async def send_command(self, command: str) -> None:
        cmd = command.strip()
        if not cmd:
            raise ValueError("Command is empty")
        if not self.is_connected:
            raise RuntimeError("Telnet client is not connected")
        await self._send_queue.put(cmd)

    async def _disconnect_locked(self, reason: str) -> None:
        if self._writer and not self._writer.is_closing():
            # Best-effort: stop streams before closing the session.
            try:
                self._writer.write(b"speed.stream off\n")
                self._writer.write(b"spid.stream off\n")
                await self._writer.drain()
            except Exception:
                pass

        for task in (self._reader_task, self._writer_task, self._keepalive_task, self._polling_task):
            if task is not None:
                task.cancel()

        self._reader_task = None
        self._writer_task = None
        self._keepalive_task = None
        self._polling_task = None

        if self._writer is not None:
            try:
                self._writer.close()
                await self._writer.wait_closed()
            except Exception:
                pass

        self._reader = None
        self._writer = None

        host = self._host
        port = self._port
        self._host = None
        self._port = None

        while not self._send_queue.empty():
            try:
                self._send_queue.get_nowait()
            except Exception:
                break

        await self._on_connection(False, host, port, reason)

    async def _reader_loop(self) -> None:
        try:
            while self._reader is not None:
                raw = await self._reader.readline()
                if not raw:
                    break
                line = raw.decode("utf-8", errors="ignore").strip()
                if line:
                    await self._on_line(line)
        except asyncio.CancelledError:
            return
        except Exception as exc:
            await self._on_error("Error leyendo Telnet: %s" % exc)
        finally:
            if self.is_connected:
                await self.disconnect("remote_disconnect")

    async def _writer_loop(self) -> None:
        try:
            while self._writer is not None:
                cmd = await self._send_queue.get()
                if self._writer is None or self._writer.is_closing():
                    break
                self._writer.write((cmd + "\n").encode("utf-8"))
                await self._writer.drain()
        except asyncio.CancelledError:
            return
        except Exception as exc:
            await self._on_error("Error enviando comando Telnet: %s" % exc)
            if self.is_connected:
                await self.disconnect("writer_error")

    async def _keepalive_loop(self) -> None:
        try:
            while self.is_connected:
                await asyncio.sleep(self._keepalive_interval_sec)
                if self.is_connected:
                    await self._send_queue.put("net.status")
        except asyncio.CancelledError:
            return

    async def _polling_loop(self) -> None:
        poll_commands: List[str] = ["comms.status", "speed.status", "spid.status"]
        try:
            while self.is_connected:
                await asyncio.sleep(self._polling_interval_sec)
                if not self.is_connected:
                    break
                for cmd in poll_commands:
                    await self._send_queue.put(cmd)
        except asyncio.CancelledError:
            return
