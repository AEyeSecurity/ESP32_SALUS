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
steer.status
comms.status
speed.status
speed.reset
speed.stream
speed.uart
```

Notas de sesion:
- Solo se mantiene un cliente Telnet a la vez.
- Si entra un cliente nuevo, reemplaza al anterior automaticamente.
- Una sesion inactiva se cierra sola (timeout) para evitar bloqueos por clientes colgados.

## Velocidad Hall (`speed.*`)

Backend activo por ISR Hall en `GPIO26/27/14` (active-low).

- `speed.status`: muestra snapshot Hall (`km/h`, `rpm`, máscara Hall, edad/período y contadores).
- `speed.reset`: reinicia contadores Hall y devuelve estado actualizado.
- `speed.stream on [ms] | speed.stream off`: stream periódico de `speed.status` (rango `20..5000 ms`).
- `speed.uart`: responde `N/A source=hall` (sin backend UART de velocidad).
