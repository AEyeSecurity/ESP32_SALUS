# COMMS UART ESP32 <-> Raspberry Pi (estado real del firmware)

Este documento describe lo que **hace hoy** el codigo en `src/pi_comms.cpp`,
`src/quad_functions.cpp` y `src/pid.cpp`.

> Referencia de protocolo (espejo): `ESP32_UART_PROTOCOL.md`  
> Fuente de verdad documental: `aeye-ros-workspace/src/sensores/ESP32_UART_PROTOCOL.md`.

## 1. Configuracion UART usada por el proyecto

Definida en `src/main.cpp` (`g_piCommsConfig`):

- Puerto: `UART_NUM_0`
- Pines: `TX=GPIO1`, `RX=GPIO3`
- Baudrate: `460800`
- Formato: `8N1`, sin paridad, sin flow control
- Buffer driver: RX `512`, TX `256`
- Tarea RX: periodo `1 ms`, `uart_read_bytes` con timeout `2 ms`
- Tarea TX: periodo `10 ms` (100 Hz)

## 2. Formato de tramas

### 2.1 Pi -> ESP32 (6 bytes)

- Byte 0: `0xAA`
- Byte 1: `ver_flags` (nibble alto version, nibble bajo flags)
- Byte 2: `steer_i8` (`-100..100` esperado)
- Byte 3: `accel_i8` (setpoint de velocidad normalizado)
- Byte 4: `brake_u8` (`0..100` esperado)
- Byte 5: `crc8` (CRC-8 Dallas/Maxim sobre bytes 0..4)

Flags en `ver_flags` (nibble bajo):

- Bit 0: `ESTOP`
- Bit 1: `DRIVE_EN`
- Bit 2: reservado
- Bit 3: reservado

### 2.2 ESP32 -> Pi (4 bytes)

- Byte 0: `0x55`
- Byte 1: `status_flags`
- Byte 2: `telemetry_u8` (velocidad)
- Byte 3: `crc8` (CRC-8 Dallas/Maxim sobre bytes 0..2)

Status bits:

- Bit 0: `READY`
- Bit 1: `FAULT`
- Bit 2: `OVERCURRENT`
- Bit 3: `REVERSE_REQ` (no usado en modo PID de velocidad actual)

Semantica de `telemetry_u8`:

- `0..254`: velocidad actual en `km/h`
- `255`: `N/A` (sin velocidad valida disponible)

## 3. Comportamiento RX real (`taskPiCommsRx`)

- Solo sincroniza con header `0xAA`.
- Si aparece otro `0xAA` antes de completar 6 bytes, cuenta `framesMalformed` y resincroniza.
- Si CRC falla, descarta trama y cuenta `framesCrcError`.
- Si CRC es valido, actualiza snapshot (`PiCommsRxSnapshot`) y `framesOk`.

Campos derivados que calcula RX:

- `accelEffective`:
  - `accelRaw <= 0` -> `0`
  - `1..100` -> pasa directo
  - `>100` -> clamp a `100`
- `wantsReverse/reverseRequestActive/reverseGranted` quedan en `false`

## 4. Comportamiento TX real (`taskPiCommsTx`)

- Envia cada 10 ms: `[0x55, status, telemetry, crc]`.
- `status` sale de `g_txState.statusFlags`, pero:
  - `READY` queda forzado en `piCommsSetStatusFlags`.
  - `REVERSE_REQ` se fuerza a `0` en cada envio.
- `telemetry` por defecto inicia en `255` (`N/A`) y en ese modo se calcula
  automaticamente desde el backend Hall (`GPIO26/27/14`):
  - `0..254` cuando `speedKmh` Hall está disponible.
  - `255` si el backend Hall no está listo.
- Override manual opcional: si se llama `piCommsSetTelemetry(x)` con `x!=255`,
  TX usa ese valor fijo y no la velocidad.

## 5. Como se usan los datos en control (importante)

### 5.1 Traccion/freno (`taskQuadDriveControl`)

Se considera frame de Pi "fresco" si su edad es <= `120 ms`.

- Si Pi esta fresca y `ESTOP=1`:
  - throttle inhibido (duty minimo)
  - `commandValue=0`
  - freno aplicado al `100%`
- Si Pi esta fresca y `DRIVE_EN=1`:
  - throttle usa PID de velocidad Hall con setpoint derivado de `accel_i8`:
    - `accel<=0` -> `target=0 m/s`
    - `1..100` -> `target=(accel/100)*max_speed_mps` (default `4.17`, equivalente a `15 km/h`)
  - si falla feedback Hall, el controlador entra en fail-safe: `throttle=0` y sin freno automatico adicional
- Si Pi esta fresca:
  - el freno aplicado se arbitra como:
    - `brake = max(brake_u8_pi, brake_overspeed_auto)`
    - `brake_overspeed_auto` solo aplica en modo `OVERSPEED` del speed PID
    - en `ESTOP`, freno forzado a `100%`
- Si Pi NO esta fresca:
  - el sistema cae a control RC para traccion y freno
  - en RC fresco, la traccion tambien usa speed PID:
    - `rc_throttle<=0` -> `target=0 m/s`
    - `1..100` -> `target=(rc/100)*max_speed_mps` (lineal, default `4.17`)
  - el freno RC manual (throttle negativo bajo umbral) se mezcla con overspeed: `max(brake_rc, brake_overspeed_auto)`

### 5.2 Direccion (`taskPidControl`)

- Si Pi esta fresca (<= `120 ms`), el PID usa `piSnapshot.steer`.
- Si no, vuelve a steering de RC.
- Hoy `steer` de Pi no depende de `DRIVE_EN` ni de `ESTOP`.

### 5.3 Prioridad PI vs RC durante pruebas

Si llegan frames de Pi frescos (`<=120 ms`), el firmware prioriza entrada Pi:

- Traccion: con `DRIVE_EN=1`, usa setpoint de velocidad de Pi (`accel_i8`).
- Direccion: usa `steer` de Pi aunque `DRIVE_EN=0`.
- Freno: con frame fresco, usa `max(brake_u8_pi, brake_overspeed_auto)`.

Consecuencia practica:

- Si una prueba en Raspberry transmite continuamente con `accel=0`, el vehiculo
  puede quedar "sin acelerador manual" (RC) porque entra en control Pi.
- Para pruebas con manejo manual, usar captura **solo RX** en Raspberry (no TX
  hacia ESP32).

## 6. Modo de prueba seguro (sin bloquear control manual)

1. En Raspberry, detener el servicio de bridge:
   - `sudo systemctl stop salus-ws.service`
   - verificar: `systemctl is-active salus-ws.service` -> `inactive`
2. Para evitar auto-reinicio durante pruebas:
   - `sudo systemctl mask --runtime salus-ws.service`
3. Correr prueba pasiva (solo lectura de `/dev/serial0`), sin enviar frames
   `0xAA` a la ESP32.
4. Al finalizar, restaurar servicio:
   - `sudo systemctl unmask salus-ws.service`
   - `sudo systemctl start salus-ws.service` (si corresponde)

## 7. Failsafe real vs supuesto comun

Lo que **si** ocurre al perder tramas frescas de Pi:

- Se dejan de usar comandos Pi.
- Drive/PID vuelven a RC cuando corresponde.

Lo que **no** ocurre automaticamente en `pi_comms`:

- No existe una rutina que fuerce globalmente `brake=100`, `drive_en=0` o
  `steer=0` por timeout dentro del modulo UART.

## 8. CRC usado

`crc8_maxim` en `src/pi_comms.cpp`:

- Init: `0x00`
- Polinomio: `0x31`
- Procesamiento MSB-first (Dallas/Maxim)

## 9. Observabilidad

Comandos Telnet utiles (`src/ota_telnet.cpp`):

- `comms.status`: snapshot actual (edad de frame, flags, accel/brake efectivos,
  contadores OK/CRC/malformed)
- `comms.reset`: resetea contadores de RX

---

Si cambias `PiCommsConfig`, `kPiSnapshotFreshTicks` o la logica de
`taskQuadDriveControl/taskPidControl`, actualiza este archivo.
