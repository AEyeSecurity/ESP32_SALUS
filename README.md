# ESP32 Cuatri - Comunicación WiFi con OTA y TelNet

Sistema de comunicación basado en ESP32 que proporciona comunicación WiFi, actualización OTA y TelNet para monitoreo remoto.

## Características

- **WiFi AP**: Punto de acceso "ESPcuatri" (IP: 192.168.4.1)
- **OTA**: Actualización de firmware por WiFi (puerto 3232)
- **TelNet**: Comunicación remota en tiempo real
- **UART**: Comunicación serial a 115200 baudios

## Hardware

- ESP32 Dev Board
- Conexión UART para dispositivos externos

## Uso

### Compilación
```bash
pio run
pio run --target upload
```

### Conexión
1. Conectar a WiFi "ESPcuatri" (contraseña: teamcit2024)
2. Telnet: `telnet 192.168.4.1 23`
3. UART: 115200 baudios

## API

```cpp
// Inicialización
void InicializaUart(long baud = 115200);
void InicializaWiFi(const char* ssid, const char* contrasena);
void InicializaOTA();
void InicializaTelnet();

// Comunicación
void EnviarMensaje(const String& mensaje);
bool RecibirMensaje(String& mensajerecibido);
void EnviarMensajeTelnet(const String& txt);
```

## Aplicaciones

- Comunicación serial remota
- Monitoreo IoT
- Telemetría
- Automatización

---
*Proyecto Team CIT 2024*
