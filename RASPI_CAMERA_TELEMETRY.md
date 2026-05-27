# Raspberry Camara y Telemetria HTTP

Este documento describe la Raspberry que actua como hotspot, servidor de captura de camara y receptor de telemetria HTTP enviada por el ESP32.

## Rol en el sistema

- La Raspberry levanta el hotspot WiFi `hotspotagv`.
- El ESP32 se conecta como cliente STA a ese hotspot cuando se compila con el entorno `esp32dev-raspi-hotspot`.
- La Raspberry escucha HTTP en `10.42.0.1:5000`.
- El ESP32 envia `POST /update` periodicos con datos de direccion y freno.
- Cada POST recibido dispara una captura de camara y guarda una imagen con la telemetria embebida en el nombre del archivo.

## Acceso SSH

Desde esta PC queda configurado el alias SSH:

```bash
ssh raspy-cam
```

Equivale a:

```bash
ssh cuatri2@10.42.0.1
```

La autenticacion por llave SSH ya quedo instalada, por lo que no deberia pedir contrasena desde este usuario.

## Red

Datos operativos:

- SSID hotspot Raspberry: `hotspotagv`
- IP Raspberry en hotspot: `10.42.0.1`
- Servidor Flask: `http://10.42.0.1:5000`
- Endpoint de telemetria/captura: `http://10.42.0.1:5000/update`

El ESP32 usa estos valores en `platformio.ini`:

```ini
[env:esp32dev-raspi-hotspot]
build_flags =
  -DWIFI_STA_SSID=\"hotspotagv\"
  -DWIFI_STA_PASS=\"hotspotagv123\"
  -DTELEMETRY_URL=\"http://10.42.0.1:5000/update\"
```

## Servidor HTTP de camara

Implementacion actual:

- Archivo: `servidor_telemetria.py`
- Framework: Flask
- Camara: OpenCV (`cv2.VideoCapture(0)` y fallback a `1`)
- Directorio de imagenes: `/home/admin/camara_imagenes`
- Puerto: `5000`
- Bind: `0.0.0.0`

Arranque manual:

```bash
ssh raspy-cam
python3 servidor_telemetria.py
```

Healthcheck:

```bash
curl http://10.42.0.1:5000/healthz
```

Respuesta esperada:

```json
{"status":"running"}
```

## Protocolo HTTP ESP32 -> Raspberry

Metodo:

```text
POST /update
Content-Type: application/x-www-form-urlencoded
```

Body:

```text
data=stem_in_stem_out_ozh
```

Ejemplo:

```bash
curl -X POST http://10.42.0.1:5000/update -d "data=48_1500_1"
```

Campos:

- `stem_in`: comando de direccion vigente publicado por el PID del ESP32.
- `stem_out`: angulo AS5600 centrado en centigrados; `-32768` si no esta disponible.
- `ozh`: porcentaje de freno aplicado.

Nombre de archivo generado:

```text
YYYY-MM-DD_HH-MM-SS_stemIn-48_stemOut-1500_ozh-1.jpg
```

## Firmware ESP32 relacionado

Archivos principales:

- `src/camera_telemetry.cpp`: arma el body HTTP y envia los POST.
- `include/camera_telemetry.h`: configuracion de la tarea.
- `src/main.cpp`: crea `taskCameraTelemetry` solo si `TELEMETRY_URL` no esta vacio.
- `platformio.ini`: define entornos de red y OTA.

La tarea queda habilitada cuando `TELEMETRY_URL` no esta vacio. En el entorno base `esp32dev` esta vacia, por lo que no envia telemetria HTTP. En `esp32dev-raspi-hotspot` apunta a la Raspberry.

## Entornos PlatformIO

Compilar firmware para el hotspot de la Raspberry:

```bash
pio run -e esp32dev-raspi-hotspot
```

Primer salto desde la red `CIT` hacia firmware que luego se conectara a `hotspotagv`:

```bash
pio run -e esp32dev-ota-sta-to-raspi-hotspot -t upload
```

Subida desde el AP fallback del ESP32 (`esp32-salus`, IP `192.168.4.1`) hacia firmware que luego se conectara a `hotspotagv`:

```bash
pio run -e esp32dev-ota-ap-to-raspi-hotspot -t upload
```

Subidas OTA normales cuando el ESP32 ya esta en `hotspotagv`:

```bash
pio run -e esp32dev-ota-raspi-hotspot -t upload
```

Volver desde `hotspotagv` hacia firmware configurado para `CIT`:

```bash
pio run -e esp32dev-ota-raspi-hotspot-to-sta -t upload
```

Si `pio` no aparece en la terminal, agregar PlatformIO al PATH:

```bash
echo 'export PATH="$HOME/.platformio/penv/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
```

## Troubleshooting

- Si `ssh raspy-cam` no conecta, verificar que la PC este conectada al hotspot `hotspotagv`.
- Si `curl /healthz` falla, revisar que `servidor_telemetria.py` este corriendo en la Raspberry.
- Si `POST /update` responde 500, revisar camara, permisos y disponibilidad de `/dev/video*`.
- Si el ESP32 no envia POSTs, confirmar que fue cargado con `esp32dev-raspi-hotspot` o un entorno OTA que extienda ese entorno.
- Si OTA no resuelve `esp32-salus.local`, usar temporalmente la IP real del ESP32 como `upload_port`.
