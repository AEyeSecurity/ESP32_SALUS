# ESP32 SALUS - Mapa de tareas FreeRTOS

Firmware para ESP32 centrado en el control de direccion de un quad con sensor magnetico AS5600. El proyecto combina OTA+Telnet, un lazo PID, control de acelerador LEDC y servos de freno. Este documento detalla la organizacion de las tareas FreeRTOS, sus prioridades, cadencias y condiciones de activacion.

## Flujo de arranque (src/main.cpp)

1. `setup()` inicializa UART (`InicializaUart`), WiFi en modo AP (`InicializaWiFi`), OTA (`InicializaOTA`) y el servidor Telnet (`InicializaTelnet`).
2. Se configuran los limites del controlador PID (`PidController::setTunings`, `setOutputLimits`, `setIntegralLimits`, `reset`).
3. Se configuran pines de entrada para los canales RC (GPIO0, GPIO2, GPIO4, GPIO16) y se habilita el lector RMT que captura los pulsos del receptor FS-iA6.
4. Se crean las tareas FreeRTOS usando `startTaskPinned` (`src/freertos_utils.cpp`), pasando los `*_TaskConfig` con parametros de periodo, logging y auto-inicializacion de hardware.
5. Se ejecuta un autotest del AS5600 (`runAs5600SelfTest`) y se lanza `taskAs5600Monitor`.
6. La tarea Arduino `loop()` queda como supervisor ligero: recibe mensajes por UART y duerme 50 ms entre iteraciones.

## Resumen de tareas FreeRTOS

| Tarea                     | Archivo/funcion          | Stack (palabras ~KB) | Prio | Nucleo | Cadencia / disparador                       | Habilitacion por defecto | Comentario principal |
|---------------------------|--------------------------|----------------------|------|--------|---------------------------------------------|--------------------------|----------------------|
| `taskOtaTelnet`          | `src/ota_telnet.cpp`     | 4096 (~16 KB)        | 3    | 0      | 20 ms periodica (`vTaskDelayUntil`)         | Siempre                  | Ejecuta `ArduinoOTA.handle()` y heartbeat Telnet cada 5 s si se habilita. |
| `taskRcSampler`          | `src/fs_ia6.cpp`         | 2048 (~8 KB)         | 4    | 1      | Notificacin RMT (timeout 10 ms)            | Siempre                  | Usa RMT para medir pulsos del FS-iA6, actualiza `RcSharedState` y despierta consumidores. |
| `taskAs5600Monitor`      | `src/AS5600.cpp`         | 3072 (~12 KB)        | 1    | 1      | 30 ms periodica, log cada 500 ms            | Siempre                  | Mide estado del AS5600 y opcionalmente reporta estado de iman y angulo. |
| `taskPidControl`         | `src/pid.cpp`            | 4096 (~16 KB)        | 4    | 0      | Notificacin RC (timeout 30 ms)             | `debug::kEnablePidTask` (true) | Cierra el lazo PID, incluye estado de calibracion y protege limites via finales de carrera. |
| `taskQuadDriveControl`   | `src/quad_functions.cpp` | 4096 (~16 KB)        | 3    | 1      | Notificacin RC (timeout 30 ms)             | `debug::kEnableDriveTask` (true) | Filtra acelerador, actualiza LEDC y servos de freno en una nica tarea coherente. |
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
- Configuracion: `FsIa6SamplerConfig` define periodo de vigilancia (10 ms), umbral de datos frescos y timeout de recepcion RMT.
- Bucle: inicializa cuatro canales RMT (GPIO0, GPIO2, GPIO4, GPIO16) a 1 us de resolucion, consume el ring buffer, normaliza a -100..100 y publica un `RcSharedState` protegido por `portMUX`.
- Prioridad: corre con prioridad 4 en el nucleo 1; cada pulso nuevo dispara `xTaskNotifyGive` a los consumidores registrados mediante `rcRegisterConsumer`.
- Logging: con `debug::kLogRc` envia un resumen cada 500 ms solamente cuando hay lecturas recientes, evitando ruido cuando el receptor esta desconectado.
- Consumo: otras tareas obtienen el snapshot con `rcGetStateCopy` y dependen de las notificaciones para reaccionar con latencia baja sin saturar la CPU.

### `taskAs5600Monitor` (src/AS5600.cpp)
- Configuracion: `AS5600MonitorConfig` comparte puntero al sensor, flag de log, periodo (30 ms) y `logInterval` (500 ms).
- Bucle: verifica conectividad I2C (`isConnected`), lee STATUS, RAW_ANGLE, ANGLE y MAGNITUDE. Solo cuando `logInterval` expira construye un mensaje detallado.
- Seguridad: si el sensor no responde, no se destruye la tarea; simplemente reporta desconexion y reintenta en el siguiente ciclo.
- Disparador: temporizado via `vTaskDelayUntil`. No emplea interrupciones I2C ni callbacks.

### `taskPidControl` (src/pid.cpp)
- Calibracion: responde al comando Telnet `steer.calibrate` lanzando una FSM (mover izquierda → soltar → mover derecha → soltar) que barre los finales de carrera con un duty moderado y registra los angulos reales del AS5600.
- Offset asimetrico: los limites y el centro ajustado se guardan en un estado compartido (`steering_calibration_*`). El comando `steer.offset <deg>` permite compensar mecanica con mas recorrido hacia un lado.
- Persistencia: cada vez que termina la calibracion o se aplica `steer.offset`, los limites izquierdo/derecho y el centro ajustado se almacenan en NVS (`Preferences`). Al reiniciar, `steeringCalibrationInit` restaura esos valores y evita recalibrar salvo que el usuario lo solicite de nuevo.
- Control: cuando no hay calibracion en curso, toma el `RcSharedState`, mapea setpoint segun los limites calibrados y ejecuta el PID que acciona `bridge_turn_left/right`. El PID se resetea al detectar lecturas invalidas o bloqueos por finales.
- Seguridad y logs: monitorea dt (avisos si excede umbral), reporta centro ajustado en cada log `[PID]` y detiene el puente ante datos viejos o limites activos, evitando esfuerzos contra los topes.

### `taskQuadDriveControl` (src/quad_functions.cpp)
- Configuracion: `QuadDriveTaskConfig` agrupa la configuracion del LEDC de acelerador, los servos de freno y el flag `autoInitHardware` que llama `initQuadThrottle/Brake` al crear la tarea.
- Disparo: espera notificaciones RC con `ulTaskNotifyTake` (timeout 30 ms) y utiliza `esp_timer_get_time` para medir la duracion de cada ciclo.
- Flujo: ejecuta `updateThrottleFilter`, aplica el duty con `quadThrottleUpdate` y reutiliza el valor filtrado para calcular el angulo de freno mediante `quadBrakeUpdate`, garantizando coherencia entre ambos actuadores.
- Datos viejos: si el snapshot supera 50 ms sin actualizar, fuerza 0 como entrada y cada 500 ms emite `[DRIVE] sin datos frescos` cuando el logging esta habilitado.
- Logging: combina en un solo mensaje `[DRIVE]` los cambios de RC filtrado, duty y angulo de freno, reduciendo el ruido en Telnet.
- Instrumentacion: reporta ciclos >2 ms una vez por segundo para detectar latencias anormales en la tarea de conduccion.

### `taskRcMonitor` (src/fs_ia6.cpp)
- Activacion: controlada por `debug::kEnableRcTask`. Reutiliza el `RcSharedState` mediante `rcGetStateCopy`, sin tocar el hardware.
- Funcion: detecta cambios respecto al ultimo estado y los difunde con `broadcastIf`. Util para calibrar pulsos del receptor FS-iA6.
- Cadencia: 100 ms mediante `vTaskDelay`. Si no hay cambios, la tarea solo duerme.

### `taskBridgeTest` (src/h_bridge.cpp)
- Activacion: `debug::kEnableBridgeTask`. Mutual exclusion con el PID: en `setup()` no se lanza el PID si esta flag esta en true.
- Funcionamiento: habilita el puente H, recorre rampas de duty 0-100% a izquierda y derecha con pasos de 5% y retardos de 80/60 ms. Entre rampas hay pausas de 300 ms y 2 s.
- Seguridad: usa `bridge_limit_*` para impedir movimiento contra el final de carrera. Muestra mensajes `[HBRIDGE]` periodicamente.
- Cadencia: no tiene un periodo fijo; la tarea usa multiples `vTaskDelay` en cada etapa del ciclo.

### `loop()` (src/main.cpp)
- Corre en el contexto de Arduino (core 1). Si `debug::kLogLoop` es true, reenvia cualquier mensaje recibido por UART al Telnet (`EnviarMensajeTelnet`).
- Para liberar CPU cede 50 ms con `vTaskDelay(pdMS_TO_TICKS(50))`.

## Calibracion de direccion via Telnet

1. Conectate al AP `ESPcuatri` (password `teamcit2024` por defecto) y abre una sesion Telnet contra el puerto 23 (ej: `telnet 192.168.4.1 23`).
2. Ejecuta `steer.help` para ver los comandos disponibles y `steer.status` para revisar los limites actuales. Si nunca calibraste se mostraran los valores por defecto definidos en firmware.
3. Lanza `steer.calibrate`. El PID entra en modo de calibracion, mueve la direccion hacia la izquierda hasta activar el final de carrera, luego hacia la derecha. Durante el proceso veras logs `[PID] Calibracion ...`.
4. Cuando termine, vuelve a consultar `steer.status` para confirmar los nuevos limites (`left/right`) y el centro ajustado. El campo `offsetRange` indica el desplazamiento permitido sin tocar los topes.
5. Si la mecanica es asimetrica, aplica un corrimiento con `steer.offset <grados>` (ejemplo `steer.offset -2.5`). El valor real aplicado se devuelve junto al rango permitido y queda guardado en NVS.
6. Repite `steer.status` para validar el offset final. Desde ese momento el mapeo RC usa los limites calibrados y el PID mantiene el centro compensado cuando el input vuelve a cero.
7. Tras un reinicio, `steer.status` deberia mostrar los mismos limites y centro sin necesidad de relanzar `steer.calibrate`. Vuelve a calibrar solo cuando reemplaces componentes mecanicos o detectes drift significativo.

## Recursos compartidos y sincronizacion

- `taskRcSampler` concentra las lecturas RMT y la entrega mediante `rcRegisterConsumer`/`rcGetStateCopy`; ninguna otra tarea toca los perifericos de captura.
- El valor filtrado del acelerador (`g_filteredThrottleValue`) y su marca temporal (`g_lastThrottleUpdateTick`) siguen almacenados en `quad_functions.cpp` y ahora los consume exclusivamente `taskQuadDriveControl`.
- Las funciones del puente H (`bridge_turn_*`, `bridge_stop`) no usan mutex; la coordinacion depende de las flags `debug::kEnablePidTask` y `debug::kEnableBridgeTask` para evitar tareas concurrentes sobre el mismo hardware.
- Los logs emplean `broadcastIf` que multiplexa Serial y Telnet, manteniendo consistencia en el formato sin necesidad de colas.

## Flags de habilitacion y logging (namespace `debug` en src/main.cpp)

- `kEnableBridgeTask = false`
- `kEnableRcTask = false`
- `kEnablePidTask = true`
- `kEnableDriveTask = true`
- Flags de log (`kLog*`) controlan el volumen de mensajes por tarea sin recompilar el codigo.

## Periodos y constantes relevantes (src/main.cpp)

- `OTA_PERIOD` = 20 ms
- `RC_SAMPLER_PERIOD` = 10 ms; `RC_MONITOR_PERIOD` = 100 ms
- `AS5600_PERIOD` = 30 ms; `AS5600_LOG_INTERVAL` = 500 ms
- `PID_PERIOD` = 30 ms; `PID_LOG_INTERVAL` = 200 ms
- `THROTTLE_PERIOD` = 30 ms (Drive)

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

- Activa `debug::kLogPid` y `debug::kLogDrive` cuando necesites validar el lazo y el acelerador; desactivalos para vuelo normal.
- Si `taskAs5600Monitor` reporta `connected=NO`, revisa VCC, GND y pull-ups del bus I2C antes de habilitar el PID.
- Antes de usar el PID, puedes habilitar temporalmente `taskRcMonitor` para asegurarte de que los pulsos del receptor lleguen dentro del rango esperado.
- No ejecutes `taskBridgeTest` mientras el PID este activo; ambas tareas usan el puente H sin mutex y se interferirian mutuamente.

Con esta descripcion puedes ajustar periodos, prioridades o flags con plena visibilidad del impacto en la programacion de FreeRTOS y en la interaccion con el hardware.

