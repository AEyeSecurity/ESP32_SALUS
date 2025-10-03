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
| `taskRcMonitor`  | 1    | 1         | 100 ms                     | Lee canales RC (GPIO0, GPIO2, GPIO4 y GPIO16) y notifica cambios por UART/Telnet |
| `taskPid`        | 1    | 2         | 30 ms                      | Ejecuta el controlador PID de la dirección y comanda el puente H |
| `loop()`         | 1*   | 1 (default) | 50 ms                     | Atiende mensajes UART y libera CPU |

*La tarea `loop()` es la tarea Arduino por defecto, que corre en el mismo core que otras tareas de aplicacion.*

Las tareas se crean en `setup()` mediante `xTaskCreatePinnedToCore`, fijando afinidad y prioridad. Si una tarea no puede inicializarse, se notifica por UART y Telnet.

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
- Ajustar las ganancias PID y mapas de entrada en `include/pid.h` segun la mecanica real.
- Anadir comandos extra via Telnet para diagnostico o streaming de sensores.

## Control de dirección PID

El controlador de dirección vive en el módulo `steering_pid` (`include/pid.h`, `src/pid.cpp`). Su tarea es:

- Tomar el ángulo absoluto de la rueda medido por `pwm_steering` (proporcional al duty del servo).
- Convertirlo en un ángulo relativo alrededor de un centro configurable (`steeringCenterAbsDeg`).
- Mapear la orden del canal RC en GPIO4 (–100..100) a un setpoint de ±`steeringMaxAngleDeg` grados.
- Ejecutar un PID discreto cada 30 ms (`taskPid`) y aplicar el duty resultante al puente H respetando límites y zonas muertas.

La configuración editable está centralizada en `steering_pid::config` (`include/pid.h`):

- `rcInputMin` / `rcInputMax`: rango esperado del radio control (por defecto –100..100). Permite adaptar emisoras que entreguen valores distintos.
- `rcDeadband`: zona muerta en la palanca. Valores dentro de este margen generan setpoint cero.
- `steeringCenterAbsDeg`: ángulo absoluto leido por el sensor cuando las ruedas están derechas. Usa la lectura del PWM para calibrarlo (p. ej. 155°).
- `steeringMaxAngleDeg`: giro máximo permitido a cada lado (±30° por defecto).
- `kp`, `ki`, `kd`: ganancias PID. `ki` viene en cero para evitar wind-up inicial; ajusta según respuesta deseada.
- `integralLimit`: tope del acumulador integral.
- `minDutyPercent` / `maxDutyPercent`: límites de PWM aplicados al puente H para superar fricción sin saturar la mecánica.
- `outputDeadbandPercent`: zona muerta en porcentaje; si la orden es menor se detiene el motor.
- `enableSteeringActuator`: útil para desactivar el actuador y validar lógica sin mover el hardware.

### Flujo interno

1. `taskRcMonitor` actualiza el valor RC via `steering_pid::setRcValue`.
2. `taskPid` consulta `steering_pid::rcToSetpointDegrees`, lee el ángulo absoluto y lo convierte con `steering_pid::absoluteToRelative`.
3. El método `steering_pid::update` devuelve un duty proporcional al error, aplicando anti-windup y derivada.
4. `steering_pid::applyControl` limita/filtra el duty y acciona `bridge_turn_left/right`. Si el error o setpoint es pequeño se llama `steering_pid::stopActuator`.

### Puesta a punto

1. Calibra `steeringCenterAbsDeg` posicionando las ruedas rectas y leyendo el ángulo reportado en Telnet (`PWM steering -> angle`).
2. Ajusta `steeringMaxAngleDeg` según el giro mecánico real (máx. 30° cada lado en el cuatriciclo).
3. Incrementa `kp` hasta obtener un seguimiento rápido sin oscilaciones excesivas. Solo luego suma `kd` y, si hace falta, `ki`.
4. Ajusta `minDutyPercent` para vencer la fricción pero sin golpes. Verifica que `maxDutyPercent` no exceda el límite seguro del actuador.
5. Modifica `rcInputMin/max` si tu receptor entrega un rango distinto a –100..100; mantener simetría asegura un setpoint centrado.

---
Proyecto Team CIT 2024
