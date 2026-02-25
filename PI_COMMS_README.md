# ESP32 ↔ Raspberry Pi UART Link

Guía operativa del enlace UART entre ESP32 y Raspberry Pi con el protocolo actual v2.

> Referencia de protocolo (espejo): `ESP32_UART_PROTOCOL.md`  
> Fuente de verdad documental: `aeye-ros-workspace/src/sensores/ESP32_UART_PROTOCOL.md`.

## 1. Hardware

| Raspberry Pi | ESP32       | Nota                     |
|--------------|-------------|--------------------------|
| GPIO14 (TX)  | GPIO3 / RX0 | UART0 recibe desde la Pi |
| GPIO15 (RX)  | GPIO1 / TX0 | UART0 transmite a la Pi  |
| GND          | GND         | Masa común obligatoria   |

- Nivel lógico: `3.3V`
- Enlace: `460800`, `8N1`, sin flow control

## 2. Tareas FreeRTOS

| Tarea      | Archivo             | Cadencia | Rol |
|------------|---------------------|----------|-----|
| `PiUartRx` | `src/pi_comms.cpp`  | ~1 kHz   | Parser `0xAA`, validación CRC + versión, snapshot RX |
| `PiUartTx` | `src/pi_comms.cpp`  | 100 Hz   | Emisión `0x55` con velocidad, ángulo y estado |

## 3. Protocolo v2

### Pi → ESP32 (7 bytes)

```text
0: 0xAA
1: ver_flags (ver=2 en nibble alto; bit0 ESTOP; bit1 DRIVE_EN)
2: steer_i8      (-100..100)
3: speed_cmd_lsb (u16 LE, m/s x100)
4: speed_cmd_msb
5: brake_u8      (0..100)
6: CRC-8 Dallas/Maxim (bytes 0..5)
```

### ESP32 → Pi (8 bytes)

```text
0: 0x55
1: status_flags
2: speed_meas_lsb   (u16 LE, m/s x100)
3: speed_meas_msb
4: steer_meas_lsb   (i16 LE, deg x100 centrado)
5: steer_meas_msb
6: brake_applied_u8 (0..100)
7: CRC-8 Dallas/Maxim (bytes 0..6)
```

Sentinels:

- `speed_meas_u16 = 0xFFFF` -> Hall N/A
- `steer_meas_i16 = -32768` -> steering N/A

`status_flags`:

- bit0 `READY`
- bit1 `ESTOP_ACTIVE`
- bit2 `FAILSAFE_ACTIVE`
- bit3 `PI_FRESH`
- bit4-5 `CONTROL_SOURCE` (`00 NONE`, `01 PI`, `10 RC`, `11 TEL`)
- bit6 `OVERSPEED_ACTIVE`

## 4. Flujo de control

### Tracción/freno (`taskQuadDriveControl`)

- Frame Pi fresco (`<=120 ms`) + `DRIVE_EN=1` -> usa `speed_cmd_u16` como target de velocidad en `m/s`.
- `ESTOP=1` -> throttle 0 + freno 100%.
- Freno aplicado final:
  - Pi fresca: `max(brake_u8_pi, brake_overspeed_auto)`
  - Pi no fresca: `max(brake_rc_manual, brake_overspeed_auto)`

### Dirección (`taskPidControl`)

- Frame Pi fresco -> usa `steer_i8` de Pi.
- Sin frescura -> vuelve a RC.

### Telemetría TX

- `speed_meas`: desde Hall (`speedMps * 100`)
- `steer_meas`: `measuredDeg - adjustedCenterDeg` en `deg x100`
- `brake_applied`: porcentaje realmente aplicado
- `status_flags`: derivados de estado runtime de drive

## 5. Debug

- `comms.status`: snapshot RX (`lastFrame`, `speedCmd`, flags, contadores `ok/crcErr/malformed/verErr`)
- `comms.reset`: resetea contadores RX
- Activar `debug::kLogPiComms` para logs `[PI][RX]` y `[PI][TX]`

## 6. Prueba rápida

1. Verificar cableado y `460800 8N1`.
2. Enviar frame Pi v2 válido (`ver=2`) y revisar `comms.status`.
3. Confirmar que `speedCmd` refleja `m/s x100`.
4. Observar telemetría UART de salida y validar:
   - velocidad en `m/s x100`
   - ángulo centrado en `deg x100`
   - flags de estado coherentes
