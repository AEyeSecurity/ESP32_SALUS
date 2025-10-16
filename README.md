# ESP32 SALUS - Control PID de direccion

Firmware para ESP32 que integra comunicacion OTA/Telnet con un lazo PID de posicionamiento angular basado en el sensor magnetico AS5600, un actuador controlado por un puente H, control de acelerador y freno por servos. El receptor RC (FS-iA6) usa GPIO16 para la direccion, GPIO4 para el canal de aceleracion y la salida PWM al motor se genera sobre GPIO17; los servos de freno viven en GPIO23 y GPIO22.

## Conexiones de hardware

| Senal / Modulo        | GPIO ESP32 | Descripcion                                                                 |
|-----------------------|------------|------------------------------------------------------------------------------|
| AS5600 SDA            | 25         | Bus I2C (tirar a 3V3 con 4.7 k si el modulo no lo trae).                    |
| AS5600 SCL            | 26         | Bus I2C compartido con otras mediciones si fuese necesario.                 |
| AS5600 VCC / GND      | 3V3 / GND  | Alimentacion del encoder magnetico.                                         |
| Receptor RC - Direccion | 16         | PWM (-100 a 100) que define la consigna asociada al angulo.                  |
| Receptor RC - Acelerador | 4         | PWM (-100 a 100); >15 acelera, <-15 acciona freno (servos).                 |
| Salida PWM acelerador    | 17        | LEDC canal 2 (20 kHz, 8 bits) hacia el ESC/controlador del quad.            |
| Servo freno izquierdo    | 23        | LEDC canal 3 (50 Hz, 16 bits); reposo 30°, frenado 80°.                     |
| Servo freno derecho      | 22        | LEDC canal 4 (50 Hz, 16 bits); reposo 30°, frenado 80°.                     |
| H-bridge ENABLE       | 21         | Se fuerza en HIGH cuando el PID necesita mover el motor.                    |
| H-bridge LEFT PWM     | 19         | PWM para girar el motor a la izquierda (ver `bridge_turn_left`).            |
| H-bridge RIGHT PWM    | 18         | PWM para girar el motor a la derecha (ver `bridge_turn_right`).             |

> No ejecutes `taskBridgeTest` y el PID en simultaneo: ambos manejan el mismo puente H.

## Logica del control PID

1. **Lectura del mando**: `taskPidControl` usa `readChannel()` sobre `GPIO4` para obtener un valor normalizado entre -100 y 100.
2. **Setpoint angular**: el valor anterior se mapea a grados via `mapRcValueToAngle()` (`centerDeg +/- spanDeg`).
3. **Sensado**: se consulta `AS5600::getAngleDegrees()`. Errores de I2C detienen el motor y reinician el PID.
4. **Error envuelto**: `computeAngleError()` lleva el error a +/-180 grados para evitar saltos cuando el angulo cruza 0.
5. **Actualizacion PID**: `PidController::update()` integra el error con anti-windup y derivada discreta (ver `src/pid.cpp:159`).
6. **Post-procesado**: se aplica *deadband* y un minimo de duty (`applyDeadband`, `applyMinActive`) para vencer la friccion.
7. **Actuacion**: el signo del comando decide sentido; el modulo `h_bridge` genera el PWM correspondiente o detiene el motor.

## Tareas y temporizacion

| Tarea / funcion          | Nucleo | Prioridad | Periodo / Ritmo           | Proposito principal |
|--------------------------|--------|-----------|---------------------------|---------------------|
| `taskOtaTelnet`          | 0      | 3         | 20 ms                     | Gestiona OTA y Telnet. |
| `taskAs5600Monitor`      | 1      | 1         | 30 ms (log cada 500 ms)   | Telemetria del sensor AS5600. |
| `taskPidControl`         | 1      | 2         | 20 ms                     | Ejecuta el lazo PID y controla el puente H. |
| `taskRcMonitor` (opcional)| 1     | 1         | 100 ms                    | Muestra los valores del receptor RC. |
| `taskQuadThrottleControl`| 1      | 1         | 40 ms                     | Genera el PWM de aceleracion con LEDC (umbral >15).     |
| `taskQuadBrakeControl`   | 1      | 1         | 40 ms                     | Posiciona los servos de freno segun el mando (<-15).     |
| `taskBridgeTest` (opcional)| 1    | 2         | Bucle cooperativo         | Rampa de prueba del puente H (no usar junto al PID). |
| `loop()`                 | 1      | -         | 50 ms                     | Manejo basico de UART. |

La tarea PID calcula su `dt` con `micros()` y, ante valores anomalos, usa el periodo nominal (`PID_PERIOD`).

## Parametros configurables clave

- `kRcSteeringPin` (`include/fs_ia6.h:38`): GPIO del canal RC que alimenta el setpoint de direccion.
- `kRcThrottlePin` (`include/fs_ia6.h:39`): GPIO del canal RC reservado para el canal de acelerador/freno.
- `PID_CENTER_DEG` (`src/main.cpp:24`): angulo centrado que corresponde a mando 0. Ej.: 150 grados.
- `PID_SPAN_DEG` (`src/main.cpp:25`): amplitud maxima de correccion (mando +/-100 => +/-span grados).
- `PID_DEADBAND_PERCENT` (`src/main.cpp:26`): zona muerta en % del duty final para evitar oscilaciones pequenas.
- `PID_MIN_ACTIVE_PERCENT` (`src/main.cpp:27`): duty minimo aplicado cuando el comando es distinto de cero.
- Ganancias (`PID_KP`, `PID_KI`, `PID_KD` en `src/main.cpp:28-30`): tunings base iniciales; se pueden variar y recompilar.
- Limite integral (`PID_INTEGRAL_LIMIT`, `src/main.cpp:31`): clamp simetrico del termino I para prevenir windup.
- Periodo y logging (`PID_PERIOD`, `PID_LOG_INTERVAL` en `src/main.cpp:37-38`): definen la cadencia de calculo y cada cuanto loguea.
- Flags de depuracion (`debug::kLogPid`, `debug::kEnablePidTask` en `src/main.cpp:47,50`): activan logs y la tarea PID.
- La estructura `PidTaskConfig` empaqueta estos parametros y se pasa a FreeRTOS (`src/main.cpp:62-71`).
- Parametros de acelerador (`THROTTLE_*` y `debug::kLogThrottle` en `src/main.cpp`): definen pines, rango de PWM (61-227 sobre 255, equivalente a 40-150 en Arduino 5 V), umbral (>15) y logging del control de motor.
- Parametros de freno (`BRAKE_*` y `debug::kLogBrake` en `src/main.cpp`): pines/LEDc para servos, angulos de reposo (30°) y frenado (80°), umbral de accion (-15).

En la inicializacion (`src/main.cpp:80-83`) se fijan las ganancias, limites y se hace `reset()` antes de arrancar la tarea.

## Funcionamiento interno del PID

La implementacion a medida vive en `include/pid.h` y `src/pid.cpp`:

- La clase `PidController` (`include/pid.h:11`) administra ganancias, limites y estado interno (integral, derivada).
- `PidController::update()` (`src/pid.cpp:115`) suma P, integra I con saturacion y calcula una derivada simple filtrada por paso de tiempo.
- `wrapAngleDegrees()` (`src/pid.cpp:141`) mantiene el error dentro de +/-180 grados.
- `taskPidControl()` (`src/pid.cpp:159`) coordina lecturas, PID y PWM; incluye protecciones ante lecturas invalidas y logging periodico.

## Pasos de uso

1. Conecta el hardware segun la tabla y asegura la alimentacion estable del AS5600 y del puente H.
2. Compila/sube con PlatformIO (`pio run --target upload`) o via OTA (`pio run --target upload --upload-port 192.168.4.1`).
3. Opcional: habilita Telnet (`telnet 192.168.4.1 23`) para seguir los logs del PID (`debug::kLogPid = true`).
4. Ajusta el transmisor RC: en reposo debe entregar ~150 grados (mando 0). Verifica que el sensor reporte valores coherentes.
5. Modifica ganancias o rangos en `src/main.cpp` segun la respuesta del sistema y vuelve a compilar.

## Ajuste del PID

- Comienza aumentando `PID_KP` hasta que el sistema responda rapido sin saturar; agrega `PID_KI` para eliminar error estacionario y `PID_KD` si ves sobreoscilaciones.
- Reduce `PID_DEADBAND_PERCENT` si quieres mas sensibilidad; subelo si vibra alrededor del centro.
- `PID_MIN_ACTIVE_PERCENT` ayuda a vencer la friccion estatica; ajustalo al minimo que mueva confiablemente el motor.
- Si el lazo integra demasiado y tarda en recuperarse, baja `PID_INTEGRAL_LIMIT` o `PID_KI`.
- Puedes instrumentar mas datos dentro de `taskPidControl` o enviar comandos por Telnet para ajustar en caliente (idea para mejoras futuras).

## Depuracion y buenas practicas

- Mantener `debug::kEnablePidTask = true` y `debug::kEnableBridgeTask = false`; la tarea PID ya inicializa el puente H por su cuenta.
- Si el AS5600 falla (lectura < 0), el PID congela la salida y detiene el puente; revisa cableado o bus I2C.
- Usa la tarea `taskRcMonitor` para verificar los valores del receptor antes de activar el PID.
- Los logs `[PID]` aparecen cada `PID_LOG_INTERVAL`; ajusta el valor para evitar saturar Telnet.

## Archivos relevantes

- `src/main.cpp`: configuracion de tareas, constantes y arranque del PID.
- `include/pid.h`: definicion del controlador y de `PidTaskConfig`.
- `src/pid.cpp`: implementacion del lazo, helpers de angulo y logica de FreeRTOS.
- `include/h_bridge.h` y `src/h_bridge.cpp`: capa de abstraccion del puente H.
- `include/AS5600.h` y `src/AS5600.cpp`: driver del sensor magnetico.
- `include/quad_functions.h` y `src/quad_functions.cpp`: helpers para el acelerador (LEDC) y servos de freno.
- Servos de freno (`src/quad_functions.cpp`, seccion BRAKE): genera PWM 50 Hz para posicionar los servomotores entre 30° (reposo) y 80° (frenado).

Con esta estructura dispones de un lazo PID limpio, portable y sencillo de ajustar para mantener el motor alrededor de un angulo deseado siguiendo las ordenes del receptor RC.

