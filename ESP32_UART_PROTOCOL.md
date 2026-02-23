# SALUS-UART-ESP32-RPI-V1

Documento canónico del protocolo UART entre ESP32 y Raspberry Pi para control y telemetría de velocidad.
Este documento es la base del futuro sensor ROS2 de velocidad.

## Estado de versión

- Protocolo: `SALUS-UART-ESP32-RPI-V1`
- Fecha de actualización: `2026-02-20`
- Fuente de verdad: `/home/leo/codigo/aeye-ros-workspace/src/sensores/ESP32_UART_PROTOCOL.md`
- Espejos obligatorios:
  - `/home/leo/codigo/ESP32_SALUS/ESP32_UART_PROTOCOL.md`
  - `/home/salus/codigo/RASPY_SALUS/ESP32_UART_PROTOCOL.md`
- Repos/commits verificados al momento de redactar:
  - `aeye-ros-workspace`: `4d25a21` (`main`)
  - `ESP32_SALUS`: `bb5b868` (`main`)
  - `RASPY_SALUS`: `0dc46fb` (`main`)

## 1. Transporte físico

- Enlace: UART0 ESP32 <-> UART Raspberry Pi
- Pines ESP32: `TX=GPIO1`, `RX=GPIO3`
- Velocidad: `460800`
- Formato: `8N1`
- Paridad: `none`
- Flow control: `none`

## 2. Tramas del protocolo

### 2.1 Pi -> ESP32 (6 bytes)

Estructura:

1. `0xAA`
2. `ver_flags`
3. `steer_i8`
4. `accel_i8`
5. `brake_u8`
6. `crc8`

`ver_flags`:

- nibble alto: versión
- nibble bajo:
  - bit0: `ESTOP`
  - bit1: `DRIVE_EN`
  - bit2: reservado
  - bit3: reservado

Rangos esperados:

- `steer_i8`: `-100..100`
- `accel_i8`: objetivo de velocidad normalizado
  - `<=0` -> `0.0 m/s`
  - `1..100` -> `target_mps = accel_i8 * (max_speed_mps / 100)`
  - `>100` -> clamp a `100`
  - operación actual en este vehículo: `max_speed_mps = 4.17` (`15 km/h`)
- `brake_u8`: `0..100`

### 2.2 ESP32 -> Pi (4 bytes)

Estructura:

1. `0x55`
2. `status_flags`
3. `telemetry_u8`
4. `crc8`

`status_flags`:

- bit0: `READY`
- bit1: `FAULT`
- bit2: `OVERCURRENT`
- bit3: `REVERSE_REQ` (no usado en modo PID de velocidad actual; se mantiene en `0`)

Semántica de `telemetry_u8`:

- `0..254`: `speed_kmh`
- `255`: `N/A` (sin velocidad válida)

## 3. CRC

- Tipo: CRC-8 Dallas/Maxim
- Polinomio: `0x31`
- Init: `0x00`
- Procesamiento: MSB-first
- Cobertura:
  - Pi->ESP32: bytes `0..4`
  - ESP32->Pi: bytes `0..2`

## 4. Cadencias y frescura

- TX ESP32 (`0x55`): `100 Hz` (cada `10 ms`)
- Freshness de control de Pi usada en firmware: `<=120 ms`
- Regla de velocidad N/A en ESP32->Pi:
  - `telemetry_u8=255` si el backend Hall no está listo

## 5. Fuente de `telemetry_u8` en firmware ESP32

- Modo automático cuando `g_txState.telemetry == 255`:
  - usa `speedKmh` Hall redondeado y clamped a `0..254`
  - usa `255` (`N/A`) si el backend Hall no está listo
- Override manual opcional:
  - si `piCommsSetTelemetry(x)` con `x != 255`, se envía ese valor fijo

## 6. Ejemplos de tramas

### 6.1 Ejemplo válido Pi->ESP32

Payload sin CRC:

- `AA 12 00 14 00`

CRC calculado:

- `30`

Trama completa válida:

- `AA 12 00 14 00 30`

### 6.2 Ejemplo válido ESP32->Pi

Payload sin CRC:

- `55 01 0A`

CRC calculado:

- `16`

Trama completa válida:

- `55 01 0A 16`

### 6.3 Ejemplo inválido por CRC mismatch

- `55 01 0A 17`

Comportamiento esperado:

- se descarta la trama
- no se actualiza estado/velocidad
- se incrementa contador de error CRC

## 7. Contrato ROS2 para el nuevo sensor (objetivo)

Compatibilidad total con consumidores actuales:

- mantener `/wheel/speed_kmh` (`std_msgs/Float32`)
- mantener `/wheel/velocity` (`geometry_msgs/TwistStamped`), derivado de velocidad
- mantener `/wheel/odom` (`nav_msgs/Odometry`), integrado desde velocidad

Reglas de cálculo:

- `speed_kmh = telemetry_u8` cuando `telemetry_u8 != 255`
- `speed_mps = forward_sign * (speed_kmh / 3.6)`
- política `N/A`:
  - mientras `telemetry_u8=255`, conservar último valor válido hasta `speed_timeout_s` (default `0.5`)
  - vencido el timeout, forzar `0`

Parámetros mínimos esperados del nuevo sensor:

- `serial_port` (default `/dev/serial0`)
- `baudrate` (default `460800`)
- `speed_timeout_s` (default `0.5`)
- `forward_sign` (default `1.0`)
- `speed_topic` (default `/wheel/speed_kmh`)
- `velocity_topic` (default `/wheel/velocity`)
- `odom_topic` (default `/wheel/odom`)

## 8. Limitaciones explícitas

- `accel_i8` ya no representa throttle directo; representa objetivo de velocidad
- `telemetry_u8` no transporta throttle
- `telemetry_u8` no transporta dirección/ángulo de giro
- `/wheel/throttle` queda fuera del contrato canónico nuevo

## 9. Migración: legacy vs nuevo

`wheel_odom_uart` actual (sniffer crudo `b16/b17`) queda documentado como `DEPRECATED`.

| Aspecto | Legacy (`wheel_odom_uart` sniffer) | Nuevo (ESP32 telemetry UART) |
|---|---|---|
| Fuente UART | stream crudo de display | frame `0x55 status telemetry crc` |
| Decodificación velocidad | LUT por `b16/b17` | valor directo `telemetry_u8` |
| Integridad | sin CRC de protocolo canónico | CRC-8 obligatorio |
| Dependencia inversión | alta (backend/eléctrica) | ya resuelta en ESP32 |
| Complejidad ROS2 | alta | baja |
| Estado objetivo | DEPRECATED | recomendado |

## 10. Checklist de implementación futura (nuevo sensor ROS2)

1. Abrir UART con `460800 8N1`.
2. Implementar parser de 4 bytes con resincronización por header `0x55`.
3. Validar CRC antes de aceptar trama.
4. Mapear `telemetry_u8` a `speed_kmh` (`255 => N/A`).
5. Aplicar timeout `speed_timeout_s` para forzar `0` cuando corresponda.
6. Publicar en `/wheel/speed_kmh`, `/wheel/velocity`, `/wheel/odom`.
7. Mantener compatibilidad de tipos y nombres de tópicos existentes.
8. Exponer métricas mínimas: frames válidos, CRC errors, last frame age.

## 11. Regla de sincronización documental

1. Toda modificación inicia en este archivo canónico.
2. Copia textual exacta a los dos espejos.
3. Actualizar fecha y commits verificados en cada cambio.
4. Validar con `diff` exacto entre canónico y espejos antes de commit.
