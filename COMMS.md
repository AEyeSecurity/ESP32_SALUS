# COMMS UART ESP32 <-> Raspberry Pi (estado real del firmware)

Este documento describe el comportamiento actual implementado en:
- `src/pi_comms.cpp`
- `src/quad_functions.cpp`
- `src/pid.cpp`

> Referencia de protocolo (espejo): `ESP32_UART_PROTOCOL.md`  
> Fuente de verdad documental: `aeye-ros-workspace/src/sensores/ESP32_UART_PROTOCOL.md`.

## 1. UART configurada

Definida en `src/main.cpp` (`g_piCommsConfig`):

- Puerto: `UART_NUM_0`
- Pines: `TX=GPIO1`, `RX=GPIO3`
- Baudrate: `115200`
- Formato: `8N1`, sin paridad, sin flow control
- Buffer driver: RX `512`, TX `256`
- RX task: periodo `2 ms`, `uart_read_bytes` timeout `0 ms` (poll no bloqueante)
- TX task: periodo `10 ms` (100 Hz)

## 2. Tramas del protocolo

### 2.1 Pi -> ESP32 (7 bytes)

- Byte 0: `0xAA`
- Byte 1: `ver_flags` (nibble alto versión, nibble bajo flags)
- Byte 2: `steer_i8` (`-100..100` esperado)
- Byte 3: `speed_cmd_lsb`
- Byte 4: `speed_cmd_msb`
- Byte 5: `brake_u8` (`0..100` esperado)
- Byte 6: `crc8` (CRC-8 Dallas/Maxim sobre bytes `0..5`)

`speed_cmd_u16` usa **little-endian** y representa `m/s x100`.

Flags en `ver_flags` (nibble bajo):

- Bit 0: `ESTOP`
- Bit 1: `DRIVE_EN`
- Bit 2: reservado
- Bit 3: reservado

Versión esperada en firmware: `2`.

### 2.2 ESP32 -> Pi (8 bytes)

- Byte 0: `0x55`
- Byte 1: `status_flags`
- Byte 2: `speed_meas_lsb`
- Byte 3: `speed_meas_msb`
- Byte 4: `steer_meas_lsb`
- Byte 5: `steer_meas_msb`
- Byte 6: `brake_applied_u8`
- Byte 7: `crc8` (CRC-8 Dallas/Maxim sobre bytes `0..6`)

Campos:

- `speed_meas_u16` (LE): `m/s x100` (Hall)
  - `0xFFFF`: N/A (backend Hall no válido)
- `steer_meas_i16` (LE): ángulo real centrado (`deg x100`)
  - `-32768`: N/A (sensor/datos de dirección inválidos)
- `brake_applied_u8`: freno realmente aplicado (`0..100`)

`status_flags`:

- Bit 0: `READY`
- Bit 1: `ESTOP_ACTIVE`
- Bit 2: `FAILSAFE_ACTIVE`
- Bit 3: `PI_FRESH`
- Bit 4-5: `CONTROL_SOURCE`
  - `00`: NONE
  - `01`: PI
  - `10`: RC
  - `11`: TEL
- Bit 6: `OVERSPEED_ACTIVE`
- Bit 7: reservado

## 3. RX real (`taskPiCommsRx`)

- Sincroniza por header `0xAA`.
- Si aparece `0xAA` antes de completar 7 bytes, cuenta `framesMalformed` y resincroniza.
- Si CRC falla, descarta trama y suma `framesCrcError`.
- Si la versión (`ver_flags >> 4`) no es `2`, descarta trama y suma `framesVersionError`.
- Si la trama es válida, actualiza snapshot RX y suma `framesOk`.

Snapshot RX expuesto (`PiCommsRxSnapshot`):

- `steer`
- `speedCmdCentiMps`
- `brake`
- `driveEnabled`
- `estop`
- contadores: `framesOk`, `framesCrcError`, `framesMalformed`, `framesVersionError`

## 4. TX real (`taskPiCommsTx`)

Cada 10 ms envía frame `[0x55 ... crc]` con:

- `speed_meas_u16`: desde `hallSpeedGetSnapshot().speedMps` (`m/s x100`)
- `steer_meas_i16`: desde `pidGetRuntimeSnapshot().measuredDeg`, centrado con `steeringCalibrationSnapshot().adjustedCenterDeg`
- `brake_applied_u8`: desde estado runtime de `taskQuadDriveControl`
- `status_flags`: desde estado runtime de `taskQuadDriveControl`

## 5. Uso en control

### 5.1 Tracción/freno (`taskQuadDriveControl`)

Un frame de Pi es fresco si edad `<= 120 ms`.

- Con Pi fresca y `ESTOP=1`: throttle inhibido + freno `100%`.
- Con Pi fresca y `DRIVE_EN=1`: setpoint PI de velocidad vía `speed_cmd_u16` (`m/s x100`) clamped a `spid.max`.
- Si falla feedback Hall en speed PID: modo failsafe (`throttle=0`).
- Freno aplicado:
  - Pi fresca: `max(brake_u8_pi, brake_overspeed_auto)`
  - Pi no fresca: `max(brake_rc_manual, brake_overspeed_auto)`

### 5.2 Dirección (`taskPidControl`)

- Pi fresca (`<=120 ms`): dirección usa `piSnapshot.steer`
- Pi no fresca: vuelve a RC

## 6. Failsafe de enlace

Al perder frescura de trama Pi, el firmware deja de usar comandos Pi y vuelve a ruta RC/local según la lógica de `taskQuadDriveControl` y `taskPidControl`.

## 7. CRC

`crc8_maxim` en `src/pi_comms.cpp`:

- Init: `0x00`
- Polinomio: `0x31`
- Procesamiento: MSB-first

## 8. Observabilidad

Telnet (`src/ota_telnet.cpp`):

- `comms.status`: snapshot RX actual (edad frame, flags, `speedCmd`, contadores)
- `comms.reset`: resetea contadores RX

---

Si cambias `PiCommsConfig`, `kPiSnapshotFreshTicks` o la lógica de
`taskQuadDriveControl/taskPidControl`, actualiza este archivo.
