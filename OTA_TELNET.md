# Comandos OTA y Logs WiFi (`ESP32_SALUS`)

## Requisito rapido

Ejecutar desde la carpeta del proyecto:

```bash
cd /home/leo/codigo/ESP32_SALUS
```

Si `pio` no esta en `PATH`, usar:

```bash
~/.platformio/penv/bin/pio
```

## Upload por cable (USB)

```bash
pio run -e esp32dev -t upload --upload-port /dev/ttyUSB0
```

## Upload OTA por STA

```bash
pio run -e esp32dev-ota-sta -t upload
```

Objetivo OTA STA por defecto:

- Hostname: `esp32-salus.local`
- Puerto OTA: `3232`

## Upload OTA por AP (fallback)

Primero conectarse al AP de la ESP32:

```bash
nmcli --ask dev wifi connect "esp32-salus"
```

Luego subir por OTA AP:

```bash
pio run -e esp32dev-ota-ap -t upload
```

Objetivo OTA AP por defecto:

- IP: `192.168.4.1`
- Puerto OTA: `3232`

## Ver logs por WiFi (Telnet)

### En modo STA

```bash
telnet esp32-salus.local 23
```

Alternativa:

```bash
nc esp32-salus.local 23
```

### En modo AP

```bash
telnet 192.168.4.1 23
```

Alternativa:

```bash
nc 192.168.4.1 23
```

## Comandos utiles dentro de Telnet

```text
net.status
pid.status
spid.status
steer.status
comms.status
speed.status
speed.reset
speed.stream
speed.uart
spid.stream
drive.log
```

Notas de sesion:
- Solo se mantiene un cliente Telnet a la vez.
- Si entra un cliente nuevo, reemplaza al anterior automaticamente.
- Una sesion inactiva se cierra sola (timeout) para evitar bloqueos por clientes colgados.

## Velocidad Hall (`speed.*`)

Backend activo por ISR Hall en `GPIO26/27/14` (active-low).

- `speed.status`: muestra snapshot Hall (`km/h`, `m/s`, `rpm`, máscara Hall, edad/período y contadores).
- `speed.reset`: reinicia contadores Hall y devuelve estado actualizado.
- `speed.stream on [ms] | speed.stream off`: stream periódico de `speed.status` (rango `20..5000 ms`).
- `speed.uart`: responde `N/A source=hall` (sin backend UART de velocidad).

## PID de velocidad (`spid.*`)

- `spid.status`: muestra snapshot runtime del PID de velocidad (`target raw/ramped`, `speed`, `error`, `mode`, `overspeed`, `throttle`, `brake`, `failsafe`) y configuración (`kp/ki/kd`, `ramp`, `max`, `ilim`, `deadband`, `brakecap`, `hys`).
- `spid.set <kp> <ki> <kd>`: actualiza ganancias.
- `spid.kp <v> | spid.ki <v> | spid.kd <v>`: ajusta cada ganancia en vivo.
- `spid.ramp <mps2>`: ajusta rampa máxima de setpoint en `m/s^2`.
- `spid.max <mps>`: ajusta velocidad máxima mapeada desde `accel_i8` (default `4.17`, equivalente a `15 km/h`).
- `spid.brakecap <pct>`: tope de freno automático por overspeed (`0..100`, default `30`).
- `spid.hys <mps>`: histéresis de salida de overspeed (default `0.3`).
- `spid.save`: persiste configuración en NVS (`speed_pid`, incluyendo `brkcap` y `hys`).
- `spid.reset`: restaura defaults y persiste en NVS.
- `spid.stream on [ms] | spid.stream off`: stream periódico de `spid.status` para tuning en vivo (`50..5000 ms`).

## Logs de drive (`drive.log`)

- `drive.log`: muestra estado actual (`ON/OFF`) de logs `[DRIVE]`.
- `drive.log on`: habilita logs de `taskQuadDriveControl` en runtime.
- `drive.log off`: deshabilita logs de `taskQuadDriveControl` en runtime.
