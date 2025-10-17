# ESP32 SALUS - Mapa de tareas FreeRTOS

Firmware para ESP32 centrado en el control de direccion de un quad con sensor magnetico AS5600. El proyecto combina OTA+Telnet, un lazo PID, control de acelerador LEDC y servos de freno. Este documento detalla la organizacion de las tareas FreeRTOS, sus prioridades, cadencias y condiciones de activacion.

## Flujo de arranque (src/main.cpp)

1. `setup()` inicializa UART (`InicializaUart`), WiFi en modo AP (`InicializaWiFi`), OTA (`InicializaOTA`) y el servidor Telnet (`InicializaTelnet`).
2. Se configuran los limites del controlador PID (`PidController::setTunings`, `setOutputLimits`, `setIntegralLimits`, `reset`).
3. Se configuran pines de entrada para los canales RC (GPIO0, GPIO2, GPIO4, GPIO16) y se inicializa el hardware de freno (`initQuadBrake`).
4. Se crean las tareas FreeRTOS usando `startTaskPinned` (`src/freertos_utils.cpp`), pasando los `*_TaskConfig` con parametros de periodo, logging y auto-inicializacion de hardware.
5. Se ejecuta un autotest del AS5600 (`runAs5600SelfTest`) y se lanza `taskAs5600Monitor`.
6. La tarea Arduino `loop()` queda como supervisor ligero: recibe mensajes por UART y duerme 50 ms entre iteraciones.

## Resumen de tareas FreeRTOS

| Tarea                     | Archivo/funcion          | Stack (palabras ~KB) | Prio | Nucleo | Cadencia / disparador                       | Habilitacion por defecto | Comentario principal |
|---------------------------|--------------------------|----------------------|------|--------|---------------------------------------------|--------------------------|----------------------|
| `taskOtaTelnet`          | `src/ota_telnet.cpp`     | 4096 (~16 KB)        | 3    | 0      | 20 ms periodica (`vTaskDelayUntil`)         | Siempre                  | Ejecuta `ArduinoOTA.handle()` y heartbeat Telnet cada 5 s si se habilita. |
| `taskRcSampler`          | `src/fs_ia6.cpp`         | 2048 (~8 KB)         | 3    | 1      | 10 ms periodica (`vTaskDelayUntil`)         | Siempre                  | Captura los canales FS-iA6 y actualiza el `RcSharedState` compartido. |
| `taskAs5600Monitor`      | `src/AS5600.cpp`         | 3072 (~12 KB)        | 1    | 1      | 30 ms periodica, log cada 500 ms            | Siempre                  | Mide estado del AS5600 y opcionalmente reporta estado de iman y angulo. |
| `taskPidControl`         | `src/pid.cpp`            | 4096 (~16 KB)        | 2    | 1      | 30 ms (`vTaskDelayUntil` + `micros()` para dt) | `debug::kEnablePidTask` (true) | Consume el snapshot RC para el setpoint, cierra el lazo y acciona el puente H. |
| `taskQuadThrottleControl`| `src/quad_functions.cpp` | 2048 (~8 KB)         | 2    | 1      | 30 ms periodica                              | `debug::kEnableThrottleTask` (true) | Usa el snapshot RC, filtra acelerador y actualiza PWM LEDC. |
| `taskQuadBrakeControl`   | `src/quad_functions.cpp` | 2048 (~8 KB)         | 2    | 1      | 30 ms periodica                              | `debug::kEnableBrakeTask` (true) | Usa el acelerador filtrado para posicionar los servos de freno por LEDC. |
| `taskRcMonitor`          | `src/fs_ia6.cpp`         | 2048 (~8 KB)         | 1    | 1      | 100 ms periodica (`vTaskDelay`)              | `debug::kEnableRcTask` (false) | Solo loguea el snapshot compartido; ideal para calibracion. |
| `taskBridgeTest`         | `src/h_bridge.cpp`       | 4096 (~16 KB)        | 2    | 1      | Bucle cooperativo con rampas (80/60 ms)      | `debug::kEnableBridgeTask` (false) | Secuencia de prueba del puente H; no usar junto a `taskPidControl`. |
| `loop()` de Arduino      | `src/main.cpp`           | N/A                  | N/A  | 1      | 50 ms (`vTaskDelay`)                         | Siempre                  | Maneja mensajes UART y reenvia a Telnet cuando `debug::kLogLoop` esta activo. |

> Nota: FreeRTOS en ESP32 interpreta el parametro `stackSize` en palabras de 32 bits. 4096 palabras equivalen a ~16 KB.

## Detalle por tarea

### `taskOtaTelnet` (src/ota_telnet.cpp)
- Configuracion: `OtaTelnetTaskConfig` fija `taskPeriod` (20 ms), `heartbeatInterval` (5000 ms) y `logHeartbeat` (false por defecto).
- Bucle: ejecuta `ArduinoOTA.handle()` en cada tick y amortigua la latencia llamando `vTaskDelayUntil` con el ultimo `TickType_t` registrado.
- Disparadores: 100% temporizados; no usa interrupciones. El heartbeat solo envia mensajes cuando ha pasado el intervalo configurado.
- Interfaz: utiliza `EnviarMensajeTelnet` para los avisos, compartiendo la UART/Telnet con el resto del sistema via `broadcastIf`.

### `taskRcSampler` (src/fs_ia6.cpp)
- Configuracion: `FsIa6SamplerConfig` define periodo (10 ms) y logging. El stack de 2 KB es suficiente para cuatro lecturas `pulseIn`.
- Bucle: captura GPIO0, GPIO2, GPIO4 y GPIO16 usando `readChannel`, normaliza a -100..100, marca timestamp y copia el resultado en un `RcSharedState` protegido por `portMUX`.
- Prioridad: se ejecuta con prioridad 3 en el core 1 para garantizar que el snapshot se actualiza aun cuando PID/acelerador/freno esten ocupados.
- Logging: si `debug::kLogRc` es true emite un resumen cada 100 ms evitando saturar Telnet.
- Consumo: el resto de las tareas obtiene una copia consistente via `rcGetStateCopy` sin volver a invocar `pulseIn`.

### `taskAs5600Monitor` (src/AS5600.cpp)
- Configuracion: `AS5600MonitorConfig` comparte puntero al sensor, flag de log, periodo (30 ms) y `logInterval` (500 ms).
- Bucle: verifica conectividad I2C (`isConnected`), lee STATUS, RAW_ANGLE, ANGLE y MAGNITUDE. Solo cuando `logInterval` expira construye un mensaje detallado.
- Seguridad: si el sensor no responde, no se destruye la tarea; simplemente reporta desconexion y reintenta en el siguiente ciclo.
- Disparador: temporizado via `vTaskDelayUntil`. No emplea interrupciones I2C ni callbacks.

### `taskPidControl` (src/pid.cpp)
- Configuracion: `PidTaskConfig` incluye puntero al sensor, GPIO de referencia (GPIO16), calibres (centro, span, deadband, min duty), periodo (30 ms) y flag `autoInitBridge`.
- Bucle: calcula `dt` con `micros()`. Si el delta es invalido (>1 s o <=0) usa el periodo nominal en segundos (`ticksToSeconds(period)`).
- Lecturas: obtiene el ultimo `RcSharedState` con `rcGetStateCopy`. Si el snapshot esta vencido (>50 ms) fuerza consigna neutra antes de mapear a grados (`mapRcValueToAngle`). El AS5600 se lee en grados; si retorna negativo se detiene el puente y se reinicia el PID.
- Actuacion: llama `bridge_turn_left/right` segun el signo; activa el puente H si estaba deshabilitado. Reinicia el integrador si golpea un final de carrera.
- Protecciones: chequea `bridge_limit_left/right_active`. Los limites generan logs y mantienen latch para evitar spam.
- Cadencia: `vTaskDelayUntil` con periodo de 30 ms. Logs `[PID]` cada `PID_LOG_INTERVAL` (200 ms).

### `taskQuadThrottleControl` (src/quad_functions.cpp)
- Configuracion: `QuadThrottleTaskConfig` define el pin PWM (GPIO17), canal LEDC, frecuencia (20 kHz), resolucion (8 bits) y umbral de activacion (>15%). `autoInitHardware` habilita `initQuadThrottle` al arrancar la tarea.
- Lectura: toma el `RcSharedState` y aplica `updateThrottleFilter`, que suaviza y auto-ajusta el offset en reposo. Si el snapshot esta viejo (>50 ms) usa 0 como entrada.
- Accion: `quadThrottleUpdate` traduce el valor normalizado a un duty dentro de `[pwmMinDuty, pwmMaxDuty]`, saturando con `clampDuty`. El duty y el RC filtrado se loguean si cambia el valor.
- Shared state: marca `g_lastThrottleUpdateTick` con el timestamp del snapshot para que el freno sepa si hay dato fresco.
- Cadencia: `vTaskDelayUntil`, periodo 30 ms.

### `taskQuadBrakeControl` (src/quad_functions.cpp)
- Configuracion: `QuadBrakeTaskConfig` contiene servos en GPIO23/GPIO22 (LEDC 50 Hz, 16 bits) y angulos de reposo/freno. Arranca `initQuadBrake` si `autoInitHardware` es true.
- Datos de entrada: reusa el ultimo valor filtrado por la tarea de acelerador si es reciente (`throttleDataFresh(pdMS_TO_TICKS(60))`); de lo contrario asume 0 y suelta el freno.
- Accion: `quadBrakeUpdate` aplica el angulo de freno cuando el valor es menor al umbral (por defecto -15). `applyBrakeAngle` convierte a pulso microsegundos y a duty LEDC.
- Cadencia: 30 ms via `vTaskDelayUntil`. Logs `[BRAKE]` cuando cambia RC o angulo.

### `taskRcMonitor` (src/fs_ia6.cpp)
- Activacion: controlada por `debug::kEnableRcTask`. Reutiliza el `RcSharedState` mediante `rcGetStateCopy`, sin tocar el hardware.
- Funcion: detecta cambios respecto al ultimo estado y los difunde con `broadcastIf`. Útil para calibrar pulsos del receptor FS-iA6.
- Cadencia: 100 ms mediante `vTaskDelay`. Si no hay cambios, la tarea solo duerme.

### `taskBridgeTest` (src/h_bridge.cpp)
- Activacion: `debug::kEnableBridgeTask`. Mutual exclusion con el PID: en `setup()` no se lanza el PID si esta flag esta en true.
- Funcionamiento: habilita el puente H, recorre rampas de duty 0-100% a izquierda y derecha con pasos de 5% y retardos de 80/60 ms. Entre rampas hay pausas de 300 ms y 2 s.
- Seguridad: usa `bridge_limit_*` para impedir movimiento contra el final de carrera. Muestra mensajes `[HBRIDGE]` periodicamente.
- Cadencia: no tiene un periodo fijo; la tarea usa multiples `vTaskDelay` en cada etapa del ciclo.

### `loop()` (src/main.cpp)
- Corre en el contexto de Arduino (core 1). Si `debug::kLogLoop` es true, reenvia cualquier mensaje recibido por UART al Telnet (`EnviarMensajeTelnet`).
- Para liberar CPU cede 50 ms con `vTaskDelay(pdMS_TO_TICKS(50))`.

## Recursos compartidos y sincronizacion

- `taskRcSampler` es la unica que invoca `readChannel`; el snapshot se protege con `g_rcStateMux` y se entrega con `rcGetStateCopy` para evitar lecturas duplicadas.
- El valor filtrado del acelerador (`g_filteredThrottleValue`) y su marca temporal (`g_lastThrottleUpdateTick`) son `volatile` y se comparten entre las tareas de acelerador y freno.
- Las funciones del puente H (`bridge_turn_*`, `bridge_stop`) no usan mutex; la coordinacion depende de las flags `debug::kEnablePidTask` y `debug::kEnableBridgeTask` para evitar tareas concurrentes sobre el mismo hardware.
- Los logs emplean `broadcastIf` que multiplexa Serial y Telnet, manteniendo consistencia en el formato sin necesidad de colas.

## Flags de habilitacion y logging (namespace `debug` en src/main.cpp)

- `kEnableBridgeTask = false`
- `kEnableRcTask = false`
- `kEnablePidTask = true`
- `kEnableThrottleTask = true`
- `kEnableBrakeTask = true`
- Flags de log (`kLog*`) controlan el volumen de mensajes por tarea sin recompilar el codigo.

## Periodos y constantes relevantes (src/main.cpp)

- `OTA_PERIOD` = 20 ms
- `RC_SAMPLER_PERIOD` = 10 ms; `RC_MONITOR_PERIOD` = 100 ms
- `AS5600_PERIOD` = 30 ms; `AS5600_LOG_INTERVAL` = 500 ms
- `PID_PERIOD` = 30 ms; `PID_LOG_INTERVAL` = 200 ms
- `THROTTLE_PERIOD` = 30 ms
- `BRAKE_PERIOD` = 30 ms

Estos valores se inyectan en los `*_TaskConfig` y definen la cadencia con la que `vTaskDelay` o `vTaskDelayUntil` libera la CPU.

## Conexiones de hardware (resumen)

| Senal / Modulo           | GPIO ESP32 | Descripcion breve                                                      |
|--------------------------|------------|------------------------------------------------------------------------|
| AS5600 SDA / SCL         | 25 / 26    | Bus I2C del sensor magnetico.                                          |
| AS5600 VCC / GND         | 3V3 / GND  | Alimentacion del AS5600.                                               |
| FS-iA6 direccion         | 16         | PWM -100..100 para el setpoint PID.                                    |
| FS-iA6 acelerador        | 4          | PWM -100..100; >15 acelera, <-15 activa freno.                         |
| Salida PWM acelerador    | 17         | LEDC 20 kHz, 8 bits hacia ESC o controlador de motor.                  |
| Servo freno izquierdo    | 23         | LEDC 50 Hz, 16 bits.                                                   |
| Servo freno derecho      | 22         | LEDC 50 Hz, 16 bits.                                                   |
| H-bridge enable / PWM    | 21 / 19 / 18 | Control de motor para direccion, con finales de carrera en GPIO27 y 14. |

## Diagnostico y mejores practicas

- Activa `debug::kLogPid` y `debug::kLogThrottle` cuando necesites validar el lazo y el acelerador; desactivalos para vuelo normal.
- Si `taskAs5600Monitor` reporta `connected=NO`, revisa VCC, GND y pull-ups del bus I2C antes de habilitar el PID.
- Antes de usar el PID, puedes habilitar temporalmente `taskRcMonitor` para asegurarte de que los pulsos del receptor lleguen dentro del rango esperado.
- No ejecutes `taskBridgeTest` mientras el PID este activo; ambas tareas usan el puente H sin mutex y se interferirian mutuamente.

Con esta descripcion puedes ajustar periodos, prioridades o flags con plena visibilidad del impacto en la programacion de FreeRTOS y en la interaccion con el hardware.

