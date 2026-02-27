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
spid.status
steer.status
comms.status
speed.status
speed.reset
speed.stream
speed.uart
pid.stream
spid.stream
spid.target
drive.log
drive.rc.status
drive.rc.stream
drive.rc.cal
sys.rt
sys.stack
sys.jitter
sys.reset
```

Notas de sesion:
- Solo se mantiene un cliente Telnet a la vez.
- Si entra un cliente nuevo, reemplaza al anterior automaticamente.
- Una sesion inactiva se cierra sola (timeout) para evitar bloqueos por clientes colgados.

## Velocidad Hall (`speed.*`)

Backend activo por ISR Hall en `GPIO26/27/14` (active-low).

- `speed.status`: muestra snapshot Hall (`km/h`, `m/s`, `rpm`, máscara Hall, edad/período y contadores).
- `speed.reset`: reinicia contadores Hall y devuelve estado actualizado.
- `speed.stream on [ms] | speed.stream off`: stream periódico de `speed.status` (rango `20..5000 ms`).
- `speed.uart`: responde `N/A source=hall` (sin backend UART de velocidad).

## Diagnóstico de sistema (`sys.*`)

- `sys.rt`: snapshot de runtime por tarea instrumentada (`loopUs`, `maxLoopUs`, `jitter`, `overrun`, `notifyTimeout`, `core`, `prio`).
- `sys.stack`: high-water mark de stack por tarea instrumentada (palabras y bytes).
- `sys.jitter`: estado del stream de jitter.
- `sys.jitter on [ms] | sys.jitter off`: habilita/deshabilita stream periódico de resumen de jitter (`50..5000 ms`).
- `sys.reset [keep|full]`: resetea acumulados de diagnóstico (`keep` por defecto preserva registro de tareas).

## PID de velocidad (`spid.*`)

- `spid.status`: muestra snapshot runtime del PID de velocidad (`target raw/ramped`, `speed`, `error`, `p/i/d`, `unsat/sat`, feedforward `ffBase/ffDelta/ffPre`, `mode`, `overspeed`, `throttle`, `brake`, `brakeRaw`, `brakeFilt`, `hold`, `failsafe`) y configuración (incluye `ff{en,b0,bmax,du,dd,min}` y `flgr`).
- `spid.set <kp> <ki> <kd>`: actualiza ganancias.
- `spid.kp <v> | spid.ki <v> | spid.kd <v>`: ajusta cada ganancia en vivo.
- `spid.ramp <mps2>`: ajusta rampa máxima de setpoint en `m/s^2`.
- `spid.minthrottle <pct>`: piso mínimo de throttle del speed PID en seguimiento (`0..100`, default `90`).
- `spid.thslewup <pctps>`: slew de subida del throttle final (`%/s`, default `30`).
- `spid.thslewdown <pctps>`: slew de bajada del throttle final (`%/s`, default `45`).
- `spid.minth.spd <mps>`: velocidad máxima para aplicar asistencia de arranque con `minthrottle` (default `0.35`).
- `spid.launchwin <ms>`: ventana temporal de asistencia de arranque (default `1200`).
- `spid.ff on|off`: habilita/deshabilita feedforward de throttle base + PID delta.
- `spid.ff.base0 <pct>`: throttle base a `0 m/s` para el mapeo lineal feedforward (default `0`).
- `spid.ff.basemax <pct>`: throttle base a `max_speed_mps` (default `55`).
- `spid.ff.du <pct>`: delta máximo positivo del PID sobre el base (default `35`).
- `spid.ff.dd <pct>`: delta máximo negativo del PID bajo el base (default `45`).
- `spid.ff.minspd <mps>`: velocidad mínima para activar feedforward (default `0.10`).
- `spid.ff.grace <ms>`: tolerancia extra de feedback Hall durante arranque (`feedbackLaunchGraceMs`, default `1200`).
- `spid.iunwind <gain>`: descarga del integrador en saturación (`anti-windup`, default `0.35`).
- `spid.dfilter <hz>`: filtro del derivativo sobre medición Hall (default `3.0`).
- `spid.max <mps>`: ajusta velocidad máxima usada para clamp de setpoints remotos/RC (default `4.17`, equivalente a `15 km/h`).
- `spid.brakecap <pct>`: tope de freno automático por overspeed (`0..100`, default `30`).
- `spid.hys <mps>`: histéresis de salida de overspeed (default `0.3`).
- `spid.brakeslewup <pctps>`: limita subida del freno automático (`%/s`, default `35`).
- `spid.brakeslewdown <pctps>`: limita bajada del freno automático (`%/s`, default `55`).
- `spid.brakehold <ms>`: tiempo mínimo en overspeed antes de liberar (`ms`, default `200`).
- `spid.brakedb <pct>`: banda muerta del freno automático (no mueve servo por debajo de este valor, default `3`).
- `spid.target <mps|off>`: override de setpoint de velocidad por Telnet para pruebas HIL (`0..max_speed_mps`). `off` devuelve el control a PI/RC.
- `drive.rc.status`: snapshot del camino RC hacia `speed_pid` (`raw/filt/norm`, `fresh`, `elig`, `latched`, estado de auto-calibración neutral y `targetRaw/targetShaped`).
- `drive.rc.stream on [ms] | drive.rc.stream off`: stream periódico de `drive.rc.status` (`50..1000 ms`, default `100 ms`).
- `drive.rc.cal on|off`: habilita o congela la auto-calibración del offset neutral del filtro RC.
- `spid.save`: persiste configuración en NVS (`speed_pid`, incluyendo `thsup`, `thsdown`, `minspd`, `lwin`, `ffen`, `ffb0`, `ffbmx`, `ffdu`, `ffdd`, `ffmin`, `flgr`, `iunw`, `dfhz`, `brkcap`, `hys`, `brsu`, `brsd`, `brhms`, `brdb`).
- NVS `speed_pid` versión actual: `ver=4` (migración automática desde `ver=3` preservando tunings y aplicando defaults nuevos).
- `spid.reset`: restaura defaults y persiste en NVS.
- `spid.stream on [ms] | spid.stream off`: stream periódico de `spid.status` para tuning en vivo (`50..5000 ms`).

## PID de direccion (`pid.*`)

- `pid.status`: muestra ganancias y modificadores (`deadband`, `minActive`) del PID de giro.
- `pid.set <kp> <ki> <kd>`: actualiza ganancias de giro.
- `pid.kp <v> | pid.ki <v> | pid.kd <v>`: ajusta cada ganancia en vivo.
- `pid.deadband <pct>`: banda muerta aplicada sobre salida PID.
- `pid.minactive <pct>`: salida minima activa (default `0` para evitar oscilacion en centro).
- `pid.stream on [ms] | pid.stream off`: stream runtime para debug (`src`, `cmd`, `sensorDeg`, `targetDeg`, `errDeg`, `out`, `dt`, limites).

## Logs de drive (`drive.log`)

- `drive.log`: muestra estado actual de logs `[DRIVE]` base y del trace forense PID (`base=ON/OFF pid=ON/OFF period=<ms>`).
- `drive.log on`: habilita logs de `taskQuadDriveControl` en runtime.
- `drive.log off`: deshabilita logs de `taskQuadDriveControl` en runtime.
- `drive.log pid on [ms]`: habilita trace forense periódico (`50..1000 ms`, default `100 ms`).
- `drive.log pid off`: deshabilita trace forense PID.
- recomendación operacional: mantener `drive.log pid off` fuera de diagnósticos (al cerrar sesión Telnet se restablece OFF).
- cuando `src=RC`, agrega campos de diagnóstico: `rcRaw`, `rcFilt`, `rcNorm`, `rcFresh`, `rcElig`, `rcBrake`, `rcLatched`, `rcState`, `rcTargetRawMps`, `rcTargetShapedMps`.
- formato forense:  
  `[DRIVE][PIDTRACE] tMs=... src=PI|RC|TEL mode=... targetRawMps=... targetMps=... speedMps=... errMps=... p=... i=... d=... pidUnsat=... pidOutPct=... pidSatPct=... ffBasePct=... ffDeltaPct=... cmdPreSlewPct=... ffActive=Y/N throttleRawPct=... throttleFiltPct=... pwmCmdPct=... pwmDuty=x/y pwmDutyPct=... autoBrakeRawPct=... autoBrakeFiltPct=... brakeAppliedPct=... brakeA_pct=... brakeB_pct=... launchAssistActive=Y/N launchMs=... throttleSaturated=Y/N integratorClamped=Y/N fb=Y/N fs=Y/N ovs=Y/N estop=Y/N inhibit=...`
- eventos críticos con cooldown:
  - `[DRIVE][EVENT] FAILSAFE_ENTER/EXIT`
  - `[DRIVE][EVENT] OVERSPEED_ENTER/EXIT`
  - `[DRIVE][EVENT] THROTTLE_INHIBIT reason=...`

## Test HIL Speed PID (RC)

Script de pruebas guiadas para validar `speed_pid` y diagnosticar por que no entra RC:

```bash
python3 tools/tests/speed_pid_hil.py --mode interactive
```

Script recomendado para auditoría RT de este branch:

```bash
python3 tools/tests/system_rt_hil.py \
  --host esp32-salus.local \
  --nominal-duration-s 1200 \
  --stress-duration-s 600 \
  --jitter-period-ms 500 \
  --stress-pidtrace-ms 100
```

Validación por defecto del runner para `PiUartRx`: `p95<=800us` y `p99<=2000us`.
Puedes ajustar límites con `--pi-rx-p95-max-us` y `--pi-rx-p99-max-us`.

Opciones utiles:

```bash
python3 tools/tests/speed_pid_hil.py \
  --host esp32-salus.local \
  --port 23 \
  --sample-ms 200 \
  --out artifacts/speed_pid_test_report.json \
  --mode interactive
```

Salida:
- `artifacts/speed_pid_test_report.json`
- `artifacts/speed_pid_test_report.md`

Modo rapido (sin pasos manuales RC, solo baseline + chequeos pasivos):

```bash
python3 tools/tests/speed_pid_hil.py --mode quick
```

## Autocalibracion PID velocidad (Telnet)

Autotuner automatico por escalones en `m/s` usando `spid.target`, pensado para banco con ruedas al aire.
El calibrador usa como fuente principal el trace forense `[DRIVE][PIDTRACE]`, priorizando suavidad y estabilidad.

Ejecucion base (guarda artefactos en `artifacts/`):

```bash
python3 tools/tests/speed_pid_autotune.py --host esp32-salus.local
```

Opciones utiles:

```bash
python3 tools/tests/speed_pid_autotune.py \
  --host esp32-salus.local \
  --port 23 \
  --sample-ms 100 \
  --progress-interval-s 2.0 \
  --movement-threshold-mps 0.15 \
  --no-motion-grace-s 4.5 \
  --no-motion-min-target-mps 1.4 \
  --no-motion-min-throttle-pct 30 \
  --max-consecutive-no-motion 2 \
  --out-json artifacts/speed_pid_autotune_report.json \
  --out-csv artifacts/speed_pid_autotune_timeseries.csv \
  --out-failure-json artifacts/speed_pid_failure_diagnostics.json
```

Sin persistir en NVS (solo evaluacion):

Salidas:
- `artifacts/speed_pid_autotune_report.json`
- `artifacts/speed_pid_autotune_report.md`
- `artifacts/speed_pid_autotune_timeseries.csv`
- `artifacts/speed_pid_baseline_vs_candidate.json`
- `artifacts/speed_pid_failure_diagnostics.json`

Condicion previa:
- `comms.status` con Pi no fresca (`lastFrame=NONE` o edad >120 ms), porque el autotune toma control con `spid.target`.
- Persistencia final manual: el script recomienda comandos `spid.*` pero no ejecuta `spid.save` automáticamente.

## Test de throttle minimo efectivo (Hall)

Busca automaticamente el comando minimo de acelerador (via `spid.target`) que realmente produce movimiento medido por Hall.

```bash
python3 tools/tests/find_min_throttle_hil.py --host esp32-salus.local
```

Salidas:
- `artifacts/min_throttle_hil_report.json`
- `artifacts/min_throttle_hil_report.md`
