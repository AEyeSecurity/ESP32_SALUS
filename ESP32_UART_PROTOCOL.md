# SALUS-UART-ESP32-RPI-V2

Documento canónico del protocolo UART entre ESP32 y Raspberry Pi para control y telemetría.

## Estado de versión

- Protocolo: `SALUS-UART-ESP32-RPI-V2`
- Fecha de actualización: `2026-02-25`
- Fuente de verdad: `/home/leo/codigo/aeye-ros-workspace/src/sensores/ESP32_UART_PROTOCOL.md`
- Espejos obligatorios:
  - `/home/leo/codigo/ESP32_SALUS/ESP32_UART_PROTOCOL.md`
  - `/home/salus/codigo/RASPY_SALUS/ESP32_UART_PROTOCOL.md`

## 1. Transporte físico

- Enlace: UART0 ESP32 <-> UART Raspberry Pi
- Pines ESP32: `TX=GPIO1`, `RX=GPIO3`
- Velocidad: `115200`
- Formato: `8N1`
- Paridad: `none`
- Flow control: `none`

## 2. Tramas

### 2.1 Pi -> ESP32 (7 bytes)

Estructura:

1. `0xAA`
2. `ver_flags`
3. `steer_i8`
4. `speed_cmd_lsb`
5. `speed_cmd_msb`
6. `brake_u8`
7. `crc8`

`ver_flags`:

- nibble alto: versión (esperada `2`)
- nibble bajo:
  - bit0: `ESTOP`
  - bit1: `DRIVE_EN`
  - bit2: reservado
  - bit3: reservado

Codificación de `speed_cmd_u16`:

- little-endian
- unidades: `m/s x100`
- ejemplo: `2.50 m/s` -> `250` -> `0x00FA` -> bytes `FA 00`

Rangos esperados:

- `steer_i8`: `-100..100`
- `brake_u8`: `0..100`

### 2.2 ESP32 -> Pi (8 bytes)

Estructura:

1. `0x55`
2. `status_flags`
3. `speed_meas_lsb`
4. `speed_meas_msb`
5. `steer_meas_lsb`
6. `steer_meas_msb`
7. `brake_applied_u8`
8. `crc8`

Codificación de campos:

- `speed_meas_u16` (LE): velocidad Hall en `m/s x100`
  - `0xFFFF`: N/A (Hall no válido)
- `steer_meas_i16` (LE): ángulo de dirección centrado en `deg x100`
  - relativo a `adjustedCenterDeg`
  - `-32768`: N/A (sensor/runtime no válido)
- `brake_applied_u8`: freno aplicado real (`0..100`)

`status_flags`:

- bit0: `READY`
- bit1: `ESTOP_ACTIVE`
- bit2: `FAILSAFE_ACTIVE`
- bit3: `PI_FRESH`
- bit4-5: `CONTROL_SOURCE`
  - `00`: NONE
  - `01`: PI
  - `10`: RC
  - `11`: TEL
- bit6: `OVERSPEED_ACTIVE`
- bit7: reservado

## 3. CRC

- Tipo: CRC-8 Dallas/Maxim
- Polinomio: `0x31`
- Init: `0x00`
- Procesamiento: MSB-first
- Cobertura:
  - Pi->ESP32: bytes `0..5`
  - ESP32->Pi: bytes `0..6`

## 4. Cadencias y frescura

- TX ESP32 (`0x55`): `100 Hz` (cada `10 ms`)
- Freshness de control Pi en firmware: `<=120 ms`

## 5. Semántica de control en firmware

- Pi fresca + `DRIVE_EN=1` -> control de velocidad por PID Hall con target `speed_cmd_u16`.
- `ESTOP=1` -> throttle inhibido y freno 100%.
- Dirección desde Pi usa `steer_i8` cuando frame Pi está fresco.
- Sin frame fresco, el firmware vuelve a ruta RC/local.

## 6. Ejemplos

### 6.1 Pi->ESP32 (v2) ejemplo válido

- `ver_flags=0x22` (`ver=2`, `DRIVE_EN=1`)
- `steer=0`
- `speed_cmd=2.50 m/s` (`0x00FA`)
- `brake=0`

Payload sin CRC:

- `AA 22 00 FA 00 00`

### 6.2 ESP32->Pi ejemplo conceptual

Payload sin CRC:

- `55 status speedL speedH steerL steerH brake`

Ejemplo:

- `status=0x19` (`READY`, `PI_FRESH`, `src=PI`)
- `speed=1.75 m/s` (`0x00AF`)
- `steer=-3.20 deg` (`-320` -> `0xFEC0`)
- `brake=12`

## 7. Reglas de sincronización documental

1. Toda modificación inicia en este archivo canónico.
2. Copia textual exacta a los espejos.
3. Actualizar fecha en cada cambio.
4. Validar `diff` entre canónico y espejos antes de commit.
