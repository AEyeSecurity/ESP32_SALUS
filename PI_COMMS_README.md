# ESP32 ↔ Raspberry Pi UART Link

Guía paso a paso para usar la UART (GPIO3/GPIO1) del ESP32 como enlace de control con una Raspberry Pi siguiendo el protocolo descrito en `COMMS.md`.

> Referencia de protocolo (espejo): `ESP32_UART_PROTOCOL.md`  
> Fuente de verdad documental: `aeye-ros-workspace/src/sensores/ESP32_UART_PROTOCOL.md`.

---

## 1. Hardware

| Raspberry Pi | ESP32             | Nota                         |
|--------------|------------------|------------------------------|
| GPIO14 (TX)  | GPIO3 / RX0       | UART0 recibe desde la Pi     |
| GPIO15 (RX)  | GPIO1 / TX0       | UART0 transmite a la Pi      |
| GND          | GND               | Masa común obligatoria       |

- Nivel lógico: **3.3 V**.  
- Para cables >50 cm considera transceptores RS‑485 (MAX3485/SN75176).

---

## 2. Firmware y tareas

El proyecto crea dos tareas FreeRTOS dedicadas al enlace:

| Tarea        | Archivo          | Frecuencia | Rol principal                                     |
|--------------|------------------|------------|---------------------------------------------------|
| `PiUartRx`   | `src/pi_comms.cpp` | ~1 kHz     | Deserializa frames `0xAA`, valida CRC‑8 y publica un snapshot seguro (`PiCommsRxSnapshot`). |
| `PiUartTx`   | `src/pi_comms.cpp` | 100 Hz     | Envía frames `0x55` con `status_flags` y `telemetry_u8` (velocidad km/h codificada). |

- Se configuran mediante `PiCommsConfig` en `src/main.cpp`.  
- El logging detallado se activa con `debug::kLogPiComms`.

---

## 3. Protocolo (resumen operativo)

### Pi → ESP32 (6 bytes)

```
0: 0xAA
1: ver_flags (bits 7-4 versión, bits 0-1 = ESTOP/DRIVE_EN, bit2 reservado)
2: steer_i8  (-100..100)
3: accel_i8  (objetivo de velocidad normalizado)
4: brake_u8  (0..100)
5: CRC-8 Dallas/Maxim (bytes 0-4)
```

### ESP32 → Pi (4 bytes)

```
0: 0x55
1: status_flags (READY/FAULT/OVERCURRENT/REVERSE_REQ)
2: telemetry_u8 (0..254 = km/h, 255 = N/A)
3: CRC-8 Dallas/Maxim (bytes 0-2)
```

`telemetry_u8` se genera desde backend Hall cuando
`g_txState.telemetry == 255`:

- `0..254` desde `speedKmh` Hall (redondeado y clamped);
- `255` (`N/A`) si el backend Hall no está listo.

Si `piCommsSetTelemetry(x)` recibe `x != 255`, ese valor manual tiene prioridad
sobre la codificacion de velocidad.

Implementación de CRC: `crc8_maxim` en `src/pi_comms.cpp`.

---

## 4. Flujo de control

### 4.1 Tracción y freno (frame fresco <=120 ms)

- Si `ESTOP=1`, `taskQuadDriveControl` inhibe acelerador (`cmd=0`, duty mínimo) y fuerza `brake=100%`.
- Si `ESTOP=0` y `DRIVE_EN=1`, usa `accel_i8` como setpoint de velocidad:
  - `accel_i8<=0` -> `target=0 m/s`
  - `1..100` -> `target=(accel_i8/100)*max_speed_mps` (default `4.17 m/s`, equivalente a `15 km/h`)
  - `>100` -> clamp a `100`
- El PWM de tracción lo genera el PID de velocidad (`speed_pid`) con feedback Hall.
- Si el feedback Hall no es válido durante control por velocidad, entra fail-safe conservador: `throttle=0` y sin freno automático adicional.
- Si `ESTOP=0` y `DRIVE_EN=0`, la tracción queda en RC (no en Pi).
- Cuando el frame de Pi está fresco, el freno aplicado se arbitra como:
  - `applied_brake = max(brake_u8_pi, brake_overspeed_auto)`
  - `brake_overspeed_auto` solo aparece en modo `OVERSPEED` del speed PID
  - `ESTOP` fuerza `applied_brake=100`

### 4.2 Reversa (estado actual)

- En el modo PID de velocidad actual, reversa por protocolo está deshabilitada.
- `accel_i8 < 0` se clampa a `0`.
- `REVERSE_REQ` no se activa (bit 3 en `status_flags` permanece en `0`).

### 4.3 ESTOP y timeout de enlace

- `ESTOP` aplica freno inmediato y corta tracción mientras el frame esté fresco.
- Si el frame deja de estar fresco (>120 ms), `taskQuadDriveControl` y `taskPidControl` vuelven a RC.
- En `pi_comms` no hay una rutina de timeout que fuerce globalmente `brake=100`.

### 4.4 Dirección

- `steer_i8` sí se usa: `taskPidControl` toma `piSnapshot.steer` cuando Pi está fresca.
- Si el frame envejece (>120 ms), vuelve a dirección RC.
- El uso de `steer_i8` no depende de `DRIVE_EN` ni de `ESTOP`.

---

## 5. Debug y monitoreo

| Recurso                         | Descripción                                                       |
|--------------------------------|-------------------------------------------------------------------|
| `debug::kLogPiComms` (main.cpp)| Habilita logs `[PI][RX]`/`[PI][TX]` con flags decodificados.      |
| Telnet cmd `comms.status`      | Snapshot en vivo (edad del frame, flags, accel/brake efectivos). |
| Telnet cmd `comms.reset`       | Reinicia contadores de frames OK/CRC/malformed.                  |

Recomendado: activar `debug::kLogDrive` temporalmente para ver qué comandos terminan alimentando el PWM.

---

## 6. Pasos de uso

1. **Compilar y flashear** con PlatformIO (`pio run -t upload`).  
2. **Conectar hardware** según la tabla de la sección 1, compartir GND.  
3. **Iniciar la Raspberry Pi** y ejecutar `test_comms.py` (o tu proceso) a 460 800 bps.  
4. **Verificar handshake**:  
   - En Telnet ejecutar `comms.status` → debería mostrar `driver=READY` y `ageMs` estable.  
   - Mover `accel_i8` positivo con `DRIVE_EN=1`; comprobar logs `[PI][RX]` y `[DRIVE]`.  
   - Enviar `accel_i8<0` y confirmar que se clampa a `target=0 m/s` (sin reversa por protocolo).  
5. **Probar ESTOP**: setear `ESTOP=1` y validar que `cmd=0`, `brake=100` y duty en mínimo.  
6. **Integrar en el vehículo** una vez que no existan CRC errors y los tiempos (`ageMs`) se mantengan <50 ms.

---

## 7. Pruebas sin perder control manual RC

Durante validaciones de telemetria, si la Raspberry transmite frames de control
de forma continua, la ESP32 puede priorizar Pi sobre RC y parecer que se
"bloquea" el acelerador manual.

Puntos clave:

- Si el frame Pi esta fresco (`<=120 ms`), direccion usa `steer` de Pi.
- Si ademas `DRIVE_EN=1`, traccion usa el PID de velocidad con setpoint derivado de `accel_i8`.
- Con frame fresco, freno aplica `max(brake_u8_pi, brake_overspeed_auto)`.

Procedimiento recomendado para pruebas con manejo manual:

1. Detener servicio: `sudo systemctl stop salus-ws.service`
2. Verificar: `systemctl is-active salus-ws.service` -> `inactive`
3. (Opcional) bloquear arranque durante prueba:
   - `sudo systemctl mask --runtime salus-ws.service`
4. Usar script de captura **solo RX** en Raspberry (no enviar `0xAA`).
5. Restaurar al terminar:
   - `sudo systemctl unmask salus-ws.service`
   - `sudo systemctl start salus-ws.service` (si corresponde)

---

## 8. Troubleshooting rápido

| Síntoma                                   | Causa probable / acción                                        |
|-------------------------------------------|----------------------------------------------------------------|
| `driver=NOT_READY` en `comms.status`      | La UART no inicializó; revisar `piCommsInit` y cableado.       |
| `ok=0 crcErr>0`                           | Ruido o baud incorrecto; revisar velocidad y masa compartida.  |
| `accel<0` no genera reversa               | Esperado en modo PID de velocidad actual: negativos se clamped a `0`. |
| `cmd=0 (RC)` aun con Pi en marcha         | `DRIVE_EN` en 0 o frames viejos (>120 ms); revisar envío/tiempos. |
| `ESTOP` no frena                          | Verificar que bit 0 del `ver_flags` realmente se escribe y llega (logs `[PI][RX]`). |
| Se pierde control manual al testear desde Pi | Algún proceso Pi está transmitiendo control; detener `salus-ws.service` y usar modo solo RX. |

Con esta guía deberías poder cablear, probar y depurar la comunicación sin tocar el resto del firmware.
