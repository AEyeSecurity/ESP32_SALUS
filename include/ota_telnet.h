#ifndef OTA_TELNET_H
#define OTA_TELNET_H
#include <TelnetStream.h>
#include <Arduino.h>
void InicializaUart(long baud = 115200); //Funcion para llamar que inicializa comunicacion UART
void EnviarMensaje(const String& mensaje); //Funcion para llamar que envia mensaje por UART (STRING)
bool RecibirMensaje(String& mensajerecibido); //Funcion para recibir mensajes por UART (STRING)
void InicializaWiFi(const char* ssid, const char* contrasena); //Funcion que inicia la conexion con la ESP por WiFi
void InicializaOTA(); //Funcion que inicializa OTA
void InicializaTelnet();
void EnviarMensajeTelnet(const String& txt); // Env\u00eda mensaje por Telnet
#endif
