# Diagnóstico Hall ↔ telemetría UART

Esta instrumentación es opt-in y sólo agrega texto al canal Telnet. No cambia
la trama binaria `0x55`, el cálculo usado por el control, el PID, el ISR Hall,
los estados de transición ni los filtros. La opción queda desactivada cuando
se cierra la sesión Telnet.

## Lectura exacta

Con el firmware de esta rama, conectar una única sesión Telnet y ejecutar:

```text
comms.halltrace on
```

Desde ese momento se emite una línea `[PI][HALLTRACE]` por cada TX de la trama
ESP32→Pi (`0x55`, nominalmente 100 Hz), además de la respuesta al comando.
Detenerlo con:

```text
comms.halltrace off
```

La línea tiene esta forma:

```text
[PI][HALLTRACE] seq=<n> txUs=<u32> speedCenti=<u16> periodUs=<u32> lastTransitionUs=<u32> eventAgeUs=<u32> hall=0b<CBA> hasTransition=<Y|N> ok=<u32> invState=<u32> invJump=<u32> isr=<u32>
```

Los campos Hall son la misma instantánea leída para producir `speedCenti` en
esa trama: `periodUs` es el período de la última transición válida,
`lastTransitionUs` está en el mismo reloj `micros()` que `txUs`, `eventAgeUs`
es su diferencia modular, `ok` cuenta transiciones válidas, `invState` estados
inválidos, `invJump` saltos inválidos e `isr` interrupciones recibidas. `seq`
identifica el orden de las tramas durante ese arranque de la tarea TX.

Para correlacionar con la captura host, guardar simultáneamente todas las
tramas `0x55` decodificadas y las líneas Telnet. Alinear el primer TX observado
y después comparar por orden (`seq`) y por `speedCenti` (`m/s = speedCenti /`
`100`). El `rx_monotonic` del host sirve para ordenar la recepción; `txUs` es
un reloj local ESP32 y no debe compararse como si fuera tiempo Unix.

La instrumentación usa la cola de logs Telnet existente. Si el diagnóstico
produce pérdida de líneas, el contador de drops de esa cola debe considerarse
parte del resultado y la captura no permite una correlación completa.

## Cálculo de plausibilidad, sin umbral implementado

Con la configuración vigente (`motorPoles=8`, reducción `10.0`, rueda
`0.45 m`), hay `6 × (8/2) = 24` transiciones Hall por revolución de motor:

```text
motor_rpm = 60 000 000 / (periodUs × 24)
speed_mps = motor_rpm / 10 × (π × 0.45) / 60
          = 5890.486225... / periodUs
```

Por tanto, el período mínimo compatible con un límite de velocidad `V` es
`period_min_us = 5890.486225... / V`. Los límites configurados deben leerse
como límites de operación, no como un filtro ya aplicado a Hall:

| Límite real/configurado | `V` | período equivalente mínimo |
| --- | ---: | ---: |
| controlador host UART `max_speed_mps` por defecto | `4.00 m/s` | `1472.62 µs` |
| avance `spid.max` por defecto | `4.17 m/s` | `1412.59 µs` |
| reversa `spid.maxrev` por defecto | `1.30 m/s` | `4531.14 µs` |
| velocidad de sesión reportada en el issue (`≤1.8 m/s`) | `1.80 m/s` | `3272.49 µs` |
| outlier observado | `490.87 m/s` | `12.000 µs` |

Esto cuantifica que `12 µs` es incompatible con los límites anteriores, pero
no fija todavía un período de rechazo: faltan confirmar límites mecánicos,
tolerancias, resolución/latencia de captura y el contrato productivo final.
