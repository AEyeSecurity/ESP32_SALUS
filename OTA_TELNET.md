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
```
