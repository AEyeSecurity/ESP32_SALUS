# ESP32 Cuatri - Comunicacion WiFi con OTA y Telnet

Firmware para ESP32 orientado a pruebas de actuadores y comunicacion remota. El dispositivo opera como punto de acceso WiFi, expone consola por UART y Telnet y mantiene soporte OTA permanente mientras se ejecutan rutinas de control en paralelo.

## Caracteristicas principales

- **WiFi AP**: punto de acceso "ESPcuatri" (IP 192.168.4.1)
- **OTA**: actualizacion de firmware sin cable via ArduinoOTA (puerto 3232)
- **Telnet**: canal de monitoreo y debugging en tiempo real (puerto 23)
- **UART**: consola serie a 115200 baudios para comandos locales
- **FreeRTOS**: cada funcion critica vive en una tarea separada para evitar bloqueos
- **Quad Logic**: capa de control que consume las entradas RC y genera PWM de acelerador, direccion y comandos de caja

## Arquitectura de tareas

| Tarea               | Core | Prioridad | Periodo / Ritmo            | Responsabilidad clave |
|---------------------|------|-----------|----------------------------|------------------------|
| `taskOtaTelnet`     | 0    | 3         | 20 ms (vTaskDelayUntil)    | Atiende OTA y heartbeat Telnet |
| `taskRcMonitor`     | 1    | 1         | 100 ms                     | Lee el FS-iA6 (GPIO 4/6/0/2) y publica snapshots en una cola |
| `taskQuadLogic`     | 1    | 2         | 30 ms                      | Consume la cola RC, aplica PWM al acelerador (GPIO17) y steering por H-bridge |
| `taskAs5600Monitor` | 1    | 1         | 30 ms                      | Valida el encoder AS5600 y reporta estado |
| `taskBridgeTest`*   | 1    | 2         | bucle continuo cooperativo | Rampa de prueba del puente H (desactivada por defecto) |
| `loop()`            | 1    | 1 (default) | 50 ms                     | Filtro de mensajes UART y watchdog de lazo principal |

\* Solo se crea si `debug::kEnableBridgeTask` es `true`.
La tarea `loop()` es la tarea Arduino por defecto.

† La tarea `loop()` es la tarea Arduino por defecto.

Las tareas se crean en `setup()` mediante `startTaskPinned`, fijando afinidad y prioridad. Si una tarea no puede inicializarse, se notifica por UART y Telnet.

## Flujo de arranque

1. Inicializa UART, WiFi (modo AP), OTA y Telnet.
2. Configura los GPIO del receptor RC y crea la cola compartida `RcInputSnapshot`.
3. Inicializa `quad_logic`: reserva el canal LEDC #4 para el acelerador (GPIO17), inicializa el puente H y abre `Serial1` para la caja.
4. Lanza las tareas FreeRTOS (`taskOtaTelnet`, `taskRcMonitor`, `taskAs5600Monitor`, `taskQuadLogic` y las opcionales de prueba).

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

## Modulo `quad_logic`

- `quadLogicCreateRcQueue()` crea una cola de longitud 1 que transporta `RcInputSnapshot` con acelerador, direccion y auxiliares ya normalizados a +/-100.
- `taskRcMonitor` escribe cada snapshot en esa cola con `quadLogicQueueRcInput()`, de modo que la capa de actuacion siempre consume el valor mas reciente sin bloquear.
- `initQuadLogic()` configura:
  - LEDC canal **4** @ 20 kHz / 10 bits y lo asocia al GPIO17 (acelerador). Este canal queda reservado exclusivamente para el throttle, mientras que el puente H continua usando los canales 0 y 1.
  - El puente H mediante `init_h_bridge()` y gestiona enable/disable automaticamente segun el comando de direccion.
  - `Serial1` (`GEARBOX_UART_BAUD`, `GEARBOX_UART_RX_PIN`, `GEARBOX_UART_TX_PIN`) para enviar comandos de cambios (`>GEAR:UP\n`, `>GEAR:DOWN\n`). Ajusta los pines cuando se definan en hardware.
- `taskQuadLogic` corre cada 30 ms. Si no recibe actualizaciones en 250 ms aplica failsafe (duty 0 y puente deshabilitado). Al detectar flancos en `aux1`/`aux2` envia los comandos de caja por `Serial1`.
- Las trazas del modulo se controlan via `debug::kLogQuad`.

## Conexion rapida

1. Conectarse al WiFi "ESPcuatri" (clave `teamcit2024`).
2. Abrir una consola Telnet con `telnet 192.168.4.1 23`.
3. Usar UART a 115200 baudios, 8N1, para consola local.
4. Subir firmware OTA con `pio run --target upload` o `espota.py` apuntando a 192.168.4.1:3232.

## Extensiones sugeridas

- Definir el protocolo definitivo de la caja y ajustar `sendGearCommand()` para que emita los bytes reales.
- Implementar rampas/limitadores en `taskQuadLogic` (p. ej. filtros de pedal o curva de direccion).
- Enviar telemetria resumida por Telnet/TCP para visualizar el estado del cuatriciclo en tiempo real.

---
Proyecto Team CIT 2024
