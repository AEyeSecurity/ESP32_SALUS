# Telnet UI (`tools/telnet_ui`)

Interfaz web local para operar comandos Telnet del firmware ESP32 SALUS.

## Requisitos

- Python 3.8+
- ESP32 accesible por red en puerto Telnet `23`

## Ejecutar

Desde la raíz del repo:

```bash
python3 -m venv .venv-telnet-ui
source .venv-telnet-ui/bin/activate
pip install -r tools/telnet_ui/requirements.txt
uvicorn tools.telnet_ui.app.main:app --reload --host 0.0.0.0 --port 8000
```

Abrir: `http://localhost:8000`

Opción rápida (doble click):

```bash
python3 run_telnet_ui.py
```

El launcher:

- crea `.venv-telnet-ui` si no existe,
- se relanza automáticamente usando ese entorno,
- instala dependencias (`requirements.txt`) si faltan,
- y arranca la UI en `http://localhost:8000`.

Si tu gestor de archivos lo permite, marca `run_telnet_ui.py` como ejecutable y ábrelo con doble click.

## Flujo de uso

1. Configurar `host` y `puerto` en la barra superior (`esp32-salus.local:23` o `192.168.4.1:23`).
2. Clic en **Conectar**.
3. Usar paneles por dominio (`steer`, `pid`, `spid`, `speed`, `comms`, `sys`, `drive`, `net`).
4. Usar **Consola Raw** para comandos libres.

## API

- `POST /api/connect` body `{ "host": "esp32-salus.local", "port": 23 }`
- `POST /api/disconnect`
- `GET /api/status`
- `GET /api/commands`
- `POST /api/command` body `{ "command_id": "spid.status", "args": {}, "confirmed": false }`
- `POST /api/raw-command` body `{ "command": "speed.status" }`
- `WS /ws` eventos `connection`, `state_patch`, `raw_line`, `parsed_event`, `command_result`, `error`

## Pruebas

```bash
PYTHONPATH=. pytest -q tools/telnet_ui/tests
```
