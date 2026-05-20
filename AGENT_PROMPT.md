# Prompt inicial para agentes IA

Estas trabajando en el proyecto `ESP32_SALUS`, ubicado en:

`/home/leosole/Desktop/AEye/ESP32_SALUS`

Es firmware ESP32 con PlatformIO y Arduino framework. El foco principal del proyecto es control de direccion, acelerador, frenos, velocidad Hall, comunicacion UART con Raspberry Pi y debug/subida de codigo por WiFi.

Antes de hacer cambios, revisa estos archivos:

- `platformio.ini`
- `README.md`
- `OTA_TELNET.md`
- `src/main.cpp`
- `src/ota_telnet.cpp`

## Conexion a Raspberry Pi

Para conectarte a la Raspberry:

```bash
ssh salus
```

Documentacion/codigo en la Raspberry:

- `/home/salus/codigo/RAPY_ESP32_COMMS`: cliente UART actual alineado con el firmware ESP32, protocolo v2.
- `/home/salus/codigo/RASPY_SALUS`: legacy para comunicacion UART. Sus docs/codigo pueden hablar de protocolo v1, `460800`, frames de 6/4 bytes, `ALLOW_REVERSE`, `accel_i8` y `telemetry_u8`; no usarlo como fuente de verdad para el firmware actual.

Para comunicacion ESP32 <-> Raspberry, tomar como referencia el protocolo v2 del repo ESP32 y `RAPY_ESP32_COMMS`.

## Build y upload

Compilar firmware:

```bash
pio run -e esp32dev
```

Si `pio` no esta en PATH:

```bash
~/.platformio/penv/bin/pio run -e esp32dev
```

Subida por USB:

```bash
pio run -e esp32dev -t upload --upload-port /dev/ttyUSB0
```

## OTA

La forma preferida de subir codigo es OTA.

OTA por red STA:

```bash
pio run -e esp32dev-ota-sta -t upload
```

Destino:

- host: `esp32-salus.local`
- puerto OTA: `3232`
- password OTA: `ota1234`

Si falla la red STA, usar AP fallback. Primero conectarse al WiFi del ESP32:

```bash
nmcli --ask dev wifi connect "esp32-salus"
```

Luego subir por OTA AP:

```bash
pio run -e esp32dev-ota-ap -t upload
```

Destino AP:

- IP: `192.168.4.1`
- puerto OTA: `3232`
- password OTA: `ota1234`

## Debug por Telnet

El debug principal es por Telnet, no por UI web.

Conectar por red STA:

```bash
telnet esp32-salus.local 23
```

O con netcat:

```bash
nc esp32-salus.local 23
```

Conectar por AP fallback:

```bash
telnet 192.168.4.1 23
```

Comandos utiles:

```text
net.status
comms.status
sys.rt
sys.stack
sys.jitter on 500
sys.jitter off
pid.status
pid.stream on 200
pid.stream off
spid.status
spid.stream on 200
spid.stream off
speed.status
speed.stream on 200
speed.stream off
drive.log on
drive.log off
drive.log pid on 100
drive.log pid off
drive.rc.status
spid.target 1.0
spid.target -1.0
spid.target 0
spid.target off
exit
```

Notas importantes:

- Solo hay un cliente Telnet activo a la vez.
- Un cliente nuevo reemplaza al anterior.
- Al cerrar Telnet se limpian streams/logs ruidosos y overrides de prueba.
- `taskOtaTelnet` corre cada 20 ms y atiende OTA, Telnet y streams; no bloquearla.
- Evitar agregar logs excesivos en tareas de control. Usar los comandos Telnet existentes siempre que sea posible.

## Pruebas utiles

```bash
python3 tools/tests/telnet_command_regression.py --host esp32-salus.local
python3 tools/tests/system_rt_hil.py --host esp32-salus.local
python3 tools/tests/speed_pid_hil.py --mode interactive
```
