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

## Medidor de velocidad UART2

- UART usada: `UART_NUM_2`
- RX: `GPIO26`
- Baudrate: `2000`
- Inversion RX por defecto al arranque: `ON`
- Modo: solo sniff/telemetria (sin reenvio de trama)

### Stream continuo por Telnet

```text
speed.stream on
speed.stream on 100
speed.stream 250
speed.stream off
```

- `speed.stream on [ms]`: activa envio continuo de `[SPD][STATUS] ...`
- Rango de periodo: `20..5000 ms` (clamp automatico)

### Ajuste UART en vivo (diagnostico)

```text
speed.uart
speed.uart 2000 off
speed.uart 2000 on
speed.uart 2083 on
```

- `speed.uart`: muestra `baud`, inversion RX y pin.
- `speed.uart <baud> [on|off]`: reconfigura UART2 RX en caliente.
- `on` = RX invertido, `off` = RX normal.
