# Descifrado del PID de Aceleracion (`spid`)

Esta guia documenta como funciona el PID de aceleracion del proyecto. En este firmware, el "PID de aceleracion" es el **PID de velocidad** (`speed_pid`, comandos `spid.*`), no el PID de direccion (`pid.*`).

## Resumen rapido

- Modos de control: `NORMAL`, `OVERSPEED`, `FAILSAFE`.
- Salidas principales:
  - `throttlePercent` (acelerador).
  - `brakePercent` (freno automatico en overspeed).
- Pipeline del lazo:
  1. `targetRawMps` + clamp.
  2. rampa de setpoint (`targetRampedMps`).
  3. filtrado de velocidad medida.
  4. PID (con feedforward opcional).
  5. saturaciones + anti-windup.
  6. slew final de throttle y brake.

Referencias:
- `include/speed_pid.h`
- `src/speed_pid.cpp`
- `src/quad_functions.cpp`
- `src/ota_telnet.cpp`
- `src/main.cpp`

## 1) Entrada y clamps

En `speedPidCompute(...)`:

- `targetRawMps` se limita a `0..maxSpeedMps`.
- `measuredMps` se limita a `0..maxSpeedMps`.
- Si `dtSeconds` es invalido (`<=0`, `>1`, no finito), se usa `0.03 s`.

Objetivo: evitar NaN/infinito y proteger el lazo contra muestras anormales.

## 2) Rampa de setpoint

`targetRampedMps` sigue a `targetRawMps` con pendiente maxima:

- `maxSetpointRateMps2` (`spid.ramp`)

Comportamiento especial:

- Si la consigna cae a cero, `targetRampedMps` se pone en cero inmediato para evitar cola residual de acelerador.

## 3) Filtro de medicion

El lazo no usa la velocidad Hall raw directa para control fino. Hace:

1. Limitador de salto por pendiente maxima.
2. EMA corta (tau aprox. 45 ms).

Se mantiene telemetria de:

- `speedRaw` (raw)
- `speedFilt` (filtrada)

El control usa `measuredFilteredMps`.

## 4) Seleccion de modo (`NORMAL`/`OVERSPEED`/`FAILSAFE`)

### `FAILSAFE`

Entra cuando no hay feedback Hall valido y ya expiro la gracia de arranque.

Accion:

- throttle = 0
- brake = 0
- reset de terminos de control (P/I/D internos y estados auxiliares)

### `OVERSPEED`

Entra cuando:

- `measuredFilteredMps > targetRampedMps`

Accion:

- throttle forzado a 0
- freno automatico proporcional al exceso de velocidad
- aplica hysteresis y hold para evitar chatter de entrada/salida

### `NORMAL`

Se ejecuta el PID de velocidad con feedforward opcional y launch assist.

## 5) PID en modo normal

Error:

- `errorMps = targetRampedMps - measuredFilteredMps`
- banda muerta: `deadbandMps` (si `|error| < deadband`, error = 0)

Terminos:

- `P = kp * error`
- `D = -kd * d(measuredFiltered)/dt` (derivativo sobre medicion, filtrado por `derivativeFilterHz`)
- `I` con integracion condicionada y anti-windup

Anti-windup implementado:

- Si no satura empujando mas en la misma direccion, integra normal: `I += ki * error * dt`.
- Si satura y seguiria empujando a saturacion, descarga integral:
  - `I -= integratorUnwindGain * I * dt`
- Clamp final del integrador:
  - `I in [-integralLimit, +integralLimit]`

## 6) Feedforward/base throttle (`spid.ff.*`)

Si `throttleBaseEnable` esta activo y el target supera `throttleBaseActivationMinMps`, se calcula una base lineal:

- base a 0 m/s: `throttleBaseAtZeroMpsPercent`
- base a max speed: `throttleBaseAtMaxSpeedPercent`

Luego el PID trabaja como **delta** alrededor de esa base.

Limites del delta PID:

- arriba: `throttleBasePidDeltaUpMaxPercent`
- abajo: `throttleBasePidDeltaDownMaxPercent`

Comando pre-slew:

- con FF activo: `throttleCmdPreSlew = ffBase + ffDelta`
- sin FF: `throttleCmdPreSlew = pidSatOutput`

## 7) Launch assist (cuando FF no esta activo)

Para ayudar arranques desde baja velocidad:

- ventana temporal: `launchAssistWindowMs`
- piso de throttle: `minThrottlePercent`
- solo hasta velocidad: `minThrottleAssistMaxSpeedMps`

Ademas:

- `feedbackLaunchGraceMs` permite tolerar falta temporal de transiciones Hall durante arranque sin cortar inmediatamente.

## 8) Slew final de actuadores

Throttle:

- subida max: `throttleSlewUpPctPerSec`
- bajada max: `throttleSlewDownPctPerSec`

Brake automatico de overspeed:

- subida max: `overspeedBrakeSlewUpPctPerSec`
- bajada max: `overspeedBrakeSlewDownPctPerSec`

Interaccion freno-throttle:

- si freno aplicado supera umbral interno, se inhibe throttle.

## Parametros `spid.*` y efecto de cada uno

Defaults actuales definidos en `src/main.cpp` (pueden ser sobreescritos por NVS si hubo `spid.save`).

### Ganancias PID

- `spid.kp` -> `kp` (default `10.0`): respuesta proporcional.
- `spid.ki` -> `ki` (default `2.0`): elimina error estacionario.
- `spid.kd` -> `kd` (default `0.0`): amortiguacion por derivada de medicion.

### Setpoint y limites base

- `spid.max` -> `maxSpeedMps` (default `4.17`): clamp superior de setpoints y medicion para el lazo.
- `spid.ramp` -> `maxSetpointRateMps2` (default `2.0`): pendiente maxima del target rampado.
- sin comando telnet directo:
  - `integralLimit` (default `100.0`)
  - `deadbandMps` (default `0.05`)

### Asistencia de arranque y slew de throttle

- `spid.minthrottle` -> `minThrottlePercent` (default `90`)
- `spid.thslewup` -> `throttleSlewUpPctPerSec` (default `30`)
- `spid.thslewdown` -> `throttleSlewDownPctPerSec` (default `45`)
- `spid.minth.spd` -> `minThrottleAssistMaxSpeedMps` (default `0.35`)
- `spid.launchwin` -> `launchAssistWindowMs` (default `1200`)

### Feedforward (base + delta PID)

- `spid.ff on|off` -> `throttleBaseEnable` (default `true`)
- `spid.ff.base0` -> `throttleBaseAtZeroMpsPercent` (default `0`)
- `spid.ff.basemax` -> `throttleBaseAtMaxSpeedPercent` (default `55`)
- `spid.ff.du` -> `throttleBasePidDeltaUpMaxPercent` (default `35`)
- `spid.ff.dd` -> `throttleBasePidDeltaDownMaxPercent` (default `45`)
- `spid.ff.minspd` -> `throttleBaseActivationMinMps` (default `0.10`)
- `spid.ff.grace` -> `feedbackLaunchGraceMs` (default `1200`)

### Robustez de integrador y derivativo

- `spid.iunwind` -> `integratorUnwindGain` (default `0.35`)
- `spid.dfilter` -> `derivativeFilterHz` (default `3.0`)

### Overspeed / auto-brake

- `spid.brakecap` -> `overspeedBrakeMaxPercent` (default `30`)
- `spid.hys` -> `overspeedReleaseHysteresisMps` (default `0.3`)
- `spid.brakeslewup` -> `overspeedBrakeSlewUpPctPerSec` (default `35`)
- `spid.brakeslewdown` -> `overspeedBrakeSlewDownPctPerSec` (default `55`)
- `spid.brakehold` -> `overspeedBrakeHoldMs` (default `200`)
- `spid.brakedb` -> `overspeedBrakeDeadbandPercent` (default `3`)

## Como observar todo esto en vivo

Comandos utiles por Telnet:

```text
spid.status
spid.stream on 200
drive.log pid on 100
```

Campos de `spid.status` para tuning:

- `targetRaw`, `target`, `speedRaw`, `speedFilt`, `err`
- `p`, `i`, `d`, `unsat`, `sat`
- `ffBase`, `ffDelta`, `ffPre`, `ffAct`
- `throttle`, `throttleRaw`, `throttleFilt`
- `brake`, `brakeRaw`, `brakeFilt`
- `sat`, `iclamp`, `launch`, `hold`, `mode`, `failsafe`, `overspeed`

## Escenarios de validacion sugeridos

1. Reposo
- Comando: `spid.status`
- Esperado: `target=0`, `throttle=0`, modo `NORMAL` o `FAILSAFE` segun feedback.

2. Escalon de consigna
- Comando: `spid.target 1.5`
- Esperado: `targetRaw` sube inmediato, `target` sube con rampa (`spid.ramp`).

3. Saturacion + anti-windup
- Consigna alta sostenida.
- Esperado: `sat=Y` y/o `iclamp=Y`; observar efecto de `spid.iunwind`.

4. Feedforward ON/OFF
- Comandos: `spid.ff on`, luego `spid.ff off`.
- Esperado: con ON aparecen `ffBase/ffDelta`; con OFF `ffBase` tiende a 0.

5. Overspeed
- Forzar `speed > target`.
- Esperado: `mode=OVERSPEED`, `throttle=0`, `brake>0`; salida por `hys` + `brakehold`.

6. Failsafe de feedback
- Interrumpir feedback Hall.
- Esperado: `mode=FAILSAFE`, `throttle=0`, `failsafe=Y`.

## Supuestos de esta guia

- Se documenta el comportamiento actual del branch local.
- La fuente de verdad tecnica es el codigo (`speed_pid.cpp`, `main.cpp`, `ota_telnet.cpp`).
- No hay cambios de API publica en esta implementacion de documentacion.
