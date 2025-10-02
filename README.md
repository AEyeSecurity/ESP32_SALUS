# ESP32 Cuatri - Comunicacion WiFi con OTA y Telnet

Firmware para ESP32 orientado a pruebas de actuadores y comunicacion remota. El dispositivo opera como punto de acceso WiFi, expone consola por UART y Telnet y mantiene soporte OTA permanente mientras se ejecutan rutinas de control en paralelo.

## Caracteristicas principales

- **WiFi AP**: punto de acceso "ESPcuatri" (IP 192.168.4.1)
- **OTA**: actualizacion de firmware sin cable via ArduinoOTA (puerto 3232)
- **Telnet**: canal de monitoreo y debugging en tiempo real (puerto 23)
- **UART**: consola serie a 115200 baudios para comandos locales
- **FreeRTOS**: cada funcion critica vive en una tarea separada para evitar bloqueos

## Arquitectura de tareas

| Tarea            | Core | Prioridad | Periodo / Ritmo            | Responsabilidad clave |
|------------------|------|-----------|----------------------------|------------------------|
| `taskOtaTelnet`  | 0    | 3         | 20 ms (vTaskDelayUntil)    | Llama `ArduinoOTA.handle()` y publica heartbeat Telnet cada 5 s |
| `taskBridgeTest` | 1    | 2         | bucle continuo cooperativo | Ejecuta rampas PWM de prueba sobre el puente H y reporta el duty |
| `taskRcMonitor`  | 1    | 1         | 100 ms                     | Lee canales RC (GPIO 14 y 16) y notifica cambios por UART/Telnet |
| `taskPid`        | 1    | 2         | 30 ms (placeholder)        | Reserva para el controlador PID periodico |
| `loop()`         | 1*   | 1 (default) | 50 ms                     | Atiende mensajes UART y libera CPU |

*La tarea `loop()` es la tarea Arduino por defecto, que corre en el mismo core que otras tareas de aplicacion.*

Las tareas se crean en `setup()` mediante `xTaskCreatePinnedToCore`, fijando afinidad y prioridad. Si una tarea no puede inicializarse, se notifica por UART y Telnet.

`taskPid` aun no implementa el control, pero ya mantiene un periodo fijo de 30 ms para insertar la logica en el futuro.

## Flujo de arranque

1. Inicializa UART, WiFi (modo AP), OTA y Telnet.
2. Configura GPIO del receptor RC y activa el driver del puente H.
3. Lanza las tareas FreeRTOS listadas arriba.

## Dependencias y compilacion

### PlatformIO

```
pio run
pio run --target upload
```

El proyecto usa PlatformIO con `framework = arduino` y depende de:

- `TelnetStream` para la sesion remota
- `IBusBM` para integracion futura con IBUS (opcional)

> Nota: este repositorio asume que `pio` esta en el PATH. Si no, instala PlatformIO Core o usa la extension de VS Code.

## Conexion rapida

1. Conectarse al WiFi "ESPcuatri" (clave `teamcit2024`).
2. Abrir una consola Telnet con `telnet 192.168.4.1 23`.
3. Usar UART a 115200 baudios, 8N1, para consola local.
4. Subir firmware OTA con `pio run --target upload` o `espota.py` apuntando a 192.168.4.1:3232.

## Extensiones sugeridas

- Reemplazar `taskBridgeTest` por la logica real de actuacion leyendo comandos de RC o PID.
- Integrar el controlador PID en `taskPid`, protegiendo recursos compartidos con colas o semaforos.
- Anadir comandos extra via Telnet para diagnostico o streaming de sensores.

---
Proyecto Team CIT 2024
