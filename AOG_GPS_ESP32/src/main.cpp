

#include <stdio.h>
#include <string.h>

#include "jsonFunctions.hpp"
#include "main.hpp"

#include <ESPmDNS.h>
#include <WiFi.h>

#include <DNSServer.h>
#include <ESPUI.h>

#include <AsyncElegantOTA.h>

///////////////////////////////////////////////////////////////////////////
// global data
///////////////////////////////////////////////////////////////////////////

GPS_Config gpsConfig, gpsConfigDefaults;
Initialisation initialisation;
Diagnostics diagnostics;
void loadDiagnostics();
void saveDiagnostics();

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

const byte DNS_PORT = 53;
IPAddress apIP( 192, 168, 1, 1 ); //IP address for access point
IPAddress ipDestination( 192, 168, 1, 255 ); //IP address to send UDP data to

///////////////////////////////////////////////////////////////////////////
// external Libraries
///////////////////////////////////////////////////////////////////////////
ESPUIClass ESPUI( Verbosity::Quiet );
DNSServer dnsServer;
AsyncUDP udpRoof;
SoftwareSerial NmeaTransmitter;

///////////////////////////////////////////////////////////////////////////
// helper functions
///////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////
// Application
///////////////////////////////////////////////////////////////////////////
void setup( void ) {
  Serial.begin( 115200 );

  WiFi.disconnect( true );

  if( !LittleFS.begin( true ) ) {
    Serial.println( "LittleFS Mount Failed" );
    return;
  }

  loadSavedConfig();

  Serial1.begin( 115200, SERIAL_8N1, gpsConfig.gpioGpsRX1, gpsConfig.gpioGpsTX1 );
  delay( 10);
  Serial2.begin( 115200, SERIAL_8N1, gpsConfig.gpioGpsRX2, gpsConfig.gpioGpsTX2 );

  Serial.updateBaudRate( gpsConfig.baudrate );
  Serial.println( "Welcome to ESP32 Dual GPS.\nTo configure, please open the WebUI." );

  NmeaTransmitter.begin( gpsConfig.serialNmeaBaudrate, SWSERIAL_8N1, -1, gpsConfig.gpioSerialNmea );
  NmeaTransmitter.enableIntTx(false);
  nmeaMessageDelay = 1000 / gpsConfig.serialNmeaMessagesPerSec;

  pinMode( gpsConfig.gpioWifiLed, OUTPUT );
  pinMode( gpsConfig.gpioDcPowerGood, INPUT );

  initWiFi();
  apIP = WiFi.localIP();

  dnsServer.start( DNS_PORT, "*", apIP );

  Serial.println( "\n\nWiFi parameters:" );
  Serial.print( "Mode: " );
  Serial.println( WiFi.getMode() == WIFI_AP ? "Station" : "Client" );
  Serial.print( "IP address: " );
  Serial.println( WiFi.getMode() == WIFI_AP ? WiFi.softAPIP() : WiFi.localIP() );

  initESPUI();

  if( gpsConfig.enableOTA ) {
    AsyncElegantOTA.begin( ESPUI.server );
  }

  if ( udpRoof.listen( gpsConfig.aogPortSendFrom ))
  {
    Serial.print( "UDP writing to IP: " );
    Serial.println( ipDestination );
    Serial.print( "UDP writing to port: " );
    Serial.println( gpsConfig.aogPortSendTo );
    Serial.print( "UDP writing from port: " );
    Serial.println( gpsConfig.aogPortSendFrom );
  }

  if( WiFi.status() == WL_CONNECTED ) { // digitalWrite doesn't work in Wifi callbacks
    digitalWrite( gpsConfig.gpioWifiLed, gpsConfig.WifiLedOnLevel );
  }

  loadDiagnostics();
  
  initIdleStats();

  initHeadingAndPosition();
  initNmeaOut();
  initSpeedPWM();
  initDiagnosticDisplay();

  if( !MDNS.begin( "gps" )){
    Serial.println( "Error starting mDNS" );
  }
}

void loop( void ) {
  dnsServer.processNextRequest();
  vTaskDelay( 100 );
}
