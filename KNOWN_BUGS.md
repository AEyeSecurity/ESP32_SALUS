# Bugs conocidos y resueltos

Este archivo registra fallas reales vistas en campo, como se diagnosticaron y que cambio quedo aplicado. La idea es no llenar el `README.md` con investigacion historica, pero conservar el contexto para futuras regresiones.

## RC PPM por RMT se quedaba sin entregar frames

Estado: resuelto en `src/fs_ia6.cpp`.

### Sintoma

Al encender el TX con el RX/ESP32 ya prendido, el vehiculo podia pegar una sacudida inicial y luego dejar de responder al control RC.

Por Telnet, `rc.raw` mostraba una tanda inicial valida de PPM y luego contadores clavados:

```text
[RC][RAW] rmt=READY rawBursts=46 decoded=46 fail=0 rawAgeMs=9652 ...
```

El valor `decoded=46 fail=0` confirmaba que los primeros frames PPM eran validos. El problema era que `rawBursts` dejaba de subir y `rawAgeMs` crecia.

### Diagnostico

Se agrego telemetria cruda en `rc.raw`:

- `rawBursts`: cantidad de bursts RMT recibidos.
- `decoded`: frames PPM decodificados correctamente.
- `fail`: frames recibidos que no pasaron el decoder.
- `rst=ok/err` y `rstErr`: recuperaciones del RMT y ultimo error del driver.
- `rawAgeMs`: edad del ultimo burst RMT.
- `samples`: primeras duraciones `level:duration` del ultimo burst.

Durante el diagnostico se habilito temporalmente conteo de flancos GPIO por interrupcion. El resultado fue:

```text
rawBursts=50 decoded=50 fail=0 rawAgeMs=13151 edges=12745 edgeAgeMs=1
```

Esto separo el problema: la senal fisica en GPIO16 seguia viva (`edges` subia), pero el RMT dejaba de entregar nuevos items.

Los diagnósticos GPIO por interrupcion quedaron apagados por defecto para no cargar los lazos PID. Solo se activan con flags de compilacion:

```ini
-DRC_GPIO_EDGE_DEBUG=1
-DRC_GPIO_PPM_FALLBACK=1
```

### Causa probable

El RMT RX del ESP32 quedaba en un estado donde habia capturado una tanda inicial del frame PPM, pero no continuaba alimentando el ringbuffer. Reiniciar RX con limpieza de memoria no siempre recuperaba la captura.

### Fix aplicado

El sampler RMT ahora:

1. Inicializa RMT RX una vez en GPIO16.
2. Lee y devuelve cada item del ringbuffer.
3. Rearma RX con `rmt_rx_stop()` + `rmt_rx_start(..., false)` despues de consumir el item, sin limpiar memoria.
4. Exige `3` frames consecutivos antes de publicar RC como fresco.
5. Si hubo frames validos y pasan ~200 ms sin bursts nuevos, reinstala el driver RMT completo como recuperacion de ultimo recurso.

La prueba buena queda asi:

```text
[RC][RAW] rmt=READY rawBursts=263 decoded=263 fail=0 rst=1/0 rawAgeMs=4 ...
[RC][RAW] rmt=READY rawBursts=545 decoded=545 fail=0 rst=1/0 rawAgeMs=3 ...
```

`rawBursts` y `decoded` suben continuamente, `fail=0`, `rawAgeMs` queda bajo y `trusted=3`.

### Como verificar una regresion

1. Apagar TX.
2. Apagar RX/ESP32 unos segundos.
3. Encender RX/ESP32.
4. Conectar Telnet y correr `rc.raw`.
5. Encender TX y repetir `rc.raw`.
6. Confirmar con `drive.rc.status`.

Interpretacion rapida:

- `rawBursts` y `decoded` suben con `fail=0`: PPM sano.
- `rawAgeMs` crece y `rawBursts` queda clavado: RMT dejo de entregar frames.
- `rst=N/0`: el watchdog recupero RMT sin error.
- `rstErr != 0`: revisar el error del driver RMT.
- En produccion `edges/gpioPpm` quedan en cero porque el debug GPIO esta deshabilitado.
