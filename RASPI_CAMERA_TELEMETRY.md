# Raspberry, camaras y telemetria HTTP

Este documento es la fuente canonica para la comunicacion entre la Raspberry Pi de camaras y el ESP32 durante la captura de datasets.

La Raspberry inicia cada ciclo, consulta telemetria al ESP32 y luego captura las camaras.

## Rol en el sistema

- La Raspberry levanta el hotspot WiFi `hotspotagv`.
- El ESP32 se conecta como cliente STA a ese hotspot.
- El ESP32 debe usar IP fija `10.42.0.100`.
- El ESP32 expone un servidor HTTP en puerto `80`.
- La Raspberry hace `GET /telemetria` al ESP32 antes de cada ciclo de captura.
- Con la respuesta de telemetria, la Raspberry captura las 3 camaras y guarda las fotos con esos datos en el nombre del archivo.

Los datos se usan como etiquetas para entrenar el modelo de conduccion autonoma por behavioral cloning.

## Red

| Parametro | Valor |
|---|---|
| SSID hotspot Raspberry | `hotspotagv` |
| Contrasena WiFi | `hotspotagv123` |
| IP Raspberry / gateway | `10.42.0.1` |
| IP fija ESP32 | `10.42.0.100` |
| Puerto HTTP ESP32 | `80` |
| Endpoint telemetria | `http://10.42.0.100:80/telemetria` |

La IP fija del ESP32 es parte del contrato: la Raspberry necesita saber siempre a donde consultar.

## Acceso SSH a la Raspberry

Desde esta PC queda configurado el alias SSH:

```bash
ssh raspy-cam
```

Equivale a:

```bash
ssh cuatri2@10.42.0.1
```

La autenticacion por llave SSH ya quedo instalada, por lo que no deberia pedir contrasena desde este usuario.

## Protocolo HTTP Raspberry -> ESP32

Metodo:

```text
GET /telemetria
```

URL completa:

```text
http://10.42.0.100:80/telemetria
```

Respuesta:

```text
<traccion>_<direccion>_<freno>
```

Ejemplo:

```text
180_90_0
```

Campos:

| Campo | Tipo | Rango | Descripcion |
|---|---|---|---|
| `traccion` | entero | `-255..255` | PWM del motor de traccion. Positivo = avanzar, negativo = retroceder, `0` = sin potencia. |
| `direccion` | entero | `0..180` | Posicion de direccion. `0` = maximo izquierda, `90` = recto, `180` = maximo derecha. Si el actuador usa PWM en vez de angulo, adaptar el rango. |
| `freno` | entero | `0..1` | Estado del freno. `0` = sin freno, `1` = frenando. |

`traccion` y `direccion` son las acciones principales que el modelo aprende a predecir. `freno` se registra separado porque `traccion=0` no distingue entre "sin potencia" y "frenando".

## Nombre de archivo

Cada foto se guarda con telemetria en el nombre:

```text
2026-06-01_10-30-45_123456_traccion-180_direccion-90_freno-0.jpg
```

Formato:

```text
{fecha}_{hora}_{microsegundos}_traccion-{valor}_direccion-{valor}_freno-{valor}.jpg
```

## Health check

Para verificar que el ESP32 responde antes de iniciar captura:

```bash
curl http://10.42.0.100:80/telemetria
```

Si responde tres valores separados por `_`, esta listo.

## Frecuencia de captura

La Raspberry captura a un maximo de 5 FPS, es decir un ciclo cada 200 ms.

Cada ciclo:

1. Hace `GET /telemetria` al ESP32.
2. Captura las 3 camaras.
3. Guarda las imagenes con la telemetria en el nombre.
4. Espera lo que reste del intervalo de 200 ms.

## Firmware ESP32 requerido

El firmware debe:

- Conectarse al hotspot `hotspotagv`.
- Configurar IP estatica `10.42.0.100`, gateway `10.42.0.1` y mascara `255.255.255.0`.
- Levantar un servidor HTTP en puerto `80`.
- Exponer `GET /telemetria`.
- Responder `text/plain` con `traccion_direccion_freno`.

Ejemplo minimo de contrato:

```cpp
#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "hotspotagv";
const char* password = "hotspotagv123";

WebServer server(80);

int traccion = 0;
int direccion = 90;
int freno = 0;

void handleTelemetria() {
  String respuesta = String(traccion) + "_" + String(direccion) + "_" + String(freno);
  server.send(200, "text/plain", respuesta);
}

void setup() {
  Serial.begin(115200);

  IPAddress ip(10, 42, 0, 100);
  IPAddress gateway(10, 42, 0, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.config(ip, gateway, subnet);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  server.on("/telemetria", HTTP_GET, handleTelemetria);
  server.begin();
}

void loop() {
  server.handleClient();
}
```

## Firmware en este repositorio

Implementacion actual:

- `src/camera_telemetry.cpp` levanta el servidor HTTP del ESP32 y sirve `GET /telemetria`.
- `src/quad_functions.cpp` publica `tractionPwmSigned` como PWM aplicado real en `-255..255`.
- `src/ota_telnet.cpp` aplica IP estatica STA cuando el entorno define `WIFI_STA_STATIC_IP`, `WIFI_STA_GATEWAY` y `WIFI_STA_SUBNET`.
- `platformio.ini` habilita el servidor solo en `esp32dev-raspi-hotspot` y entornos OTA que extienden ese entorno.

## Entornos PlatformIO relacionados

Compilar firmware para conectarse al hotspot de la Raspberry:

```bash
pio run -e esp32dev-raspi-hotspot
```

Primer salto OTA desde la red `CIT` hacia firmware configurado para `hotspotagv`:

```bash
pio run -e esp32dev-ota-sta-to-raspi-hotspot -t upload
```

Subida desde el AP fallback del ESP32 (`esp32-salus`, IP `192.168.4.1`) hacia firmware configurado para `hotspotagv`:

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
- Si `curl http://10.42.0.100:80/telemetria` falla, verificar que el ESP32 este conectado al hotspot y usando IP fija `10.42.0.100`.
- Si la respuesta esta vacia o malformada, revisar que el firmware actualice `traccion`, `direccion` y `freno`.
- Si la Raspberry no captura a 5 FPS, revisar tiempos de captura de camaras y timeouts del `GET`.
- Si OTA no resuelve `esp32-salus.local`, usar temporalmente la IP real del ESP32 como `upload_port`.
