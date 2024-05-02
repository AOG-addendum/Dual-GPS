
#include <stdio.h>

#include <ESPUI.h>

#include "main.hpp"
#include "jsonFunctions.hpp"

uint16_t labelLoad;
uint16_t labelHeading;
uint16_t labelGpsReceivers;
uint16_t labelPwmOut;
uint16_t buttonReset;

void setResetButtonToRed() {
  ESPUI.getControl( buttonReset )->color = ControlColor::Alizarin;
  ESPUI.updateControlAsync( buttonReset );
}

void initESPUI ( void ) {

  labelLoad = ESPUI.addControl( ControlType::Label, "Load", "", ControlColor::Turquoise );
  labelHeading = ESPUI.addControl( ControlType::Label, "Heading values", "", ControlColor::Sunflower );

  buttonReset = ESPUI.addControl( ControlType::Button, "Store the Settings", "Apply", ControlColor::Sunflower, Control::noParent,
  []( Control * control, int id ) {
    if( id == B_UP ) {
      saveConfig();
    }
  } );

  buttonReset = ESPUI.addControl( ControlType::Button, "If this turns red, you have to", "Apply & Reboot", ControlColor::Sunflower, Control::noParent,
  []( Control * control, int id ) {
    if( id == B_UP ) {
      saveConfig();
      SPIFFS.end();
      ESP.restart();
    }
  } );

  uint16_t tabConfigurations;

  // Status Tab
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "Status", "Status" );
    String buildDate = String( __DATE__ );
    buildDate += String( " " );
    buildDate += String( __TIME__ );
    ESPUI.addControl( ControlType::Label, "Build date", buildDate, ControlColor::Turquoise, tab );

    labelGpsReceivers = ESPUI.addControl( ControlType::Label, "GPS receivers", "", ControlColor::Turquoise, tab );
    labelPwmOut = ESPUI.addControl( ControlType::Label, "Speed output", "", ControlColor::Turquoise, tab );
  }

  // Network Tab
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "Network", "Network" );

    ESPUI.addControl( ControlType::Text, "SSID*", String( gpsConfig.ssid ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      control->value.toCharArray( gpsConfig.ssid, sizeof( gpsConfig.ssid ) );
      setResetButtonToRed();
    } );
    ESPUI.addControl( ControlType::Text, "Password*", String( gpsConfig.password ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      control->value.toCharArray( gpsConfig.password, sizeof( gpsConfig.password ) );
      setResetButtonToRed();
    } );
    ESPUI.addControl( ControlType::Text, "Hostname*", String( gpsConfig.hostname ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      control->value.toCharArray( gpsConfig.hostname, sizeof( gpsConfig.hostname ) );
      setResetButtonToRed();
    } );

    ESPUI.addControl( ControlType::Switcher, "OTA Enabled*", gpsConfig.enableOTA ? "1" : "0", ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      gpsConfig.enableOTA = control->value.toInt() == 1;
      setResetButtonToRed();
    } );
    ESPUI.addControl( ControlType::Number, "Port to send from*", String( gpsConfig.aogPortSendFrom ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      gpsConfig.aogPortSendFrom = control->value.toInt();
      setResetButtonToRed();
    } );

    ESPUI.addControl( ControlType::Number, "Port to send to*", String( gpsConfig.aogPortSendTo ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      gpsConfig.aogPortSendTo = control->value.toInt();
      setResetButtonToRed();
    } );
    ESPUI.addControl( ControlType::Number, "Port to listen to*", String( gpsConfig.aogPortListenTo ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      gpsConfig.aogPortListenTo = control->value.toInt();
      setResetButtonToRed();
    } );
    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Wifi Led*", String( gpsConfig.WifiLedOnLevel ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        gpsConfig.WifiLedOnLevel = control->value.toInt();
        setResetButtonToRed();
      } );
        ESPUI.addControl( ControlType::Option, "On when GPIO is Low", "0", ControlColor::Alizarin, sel );
        ESPUI.addControl( ControlType::Option, "On when GPIO is High", "1", ControlColor::Alizarin, sel );
    }
  }

  // GPS Tab
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "GPS", "GPS" );

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Distance between Dual GPS Antennas", String( gpsConfig.AntDist ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        gpsConfig.AntDist = control->value.toFloat();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "500", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "GPS Antenna Height", String( gpsConfig.AntHeight ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        gpsConfig.AntHeight = control->value.toFloat();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "500", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "GPS Antenna right offset", String( gpsConfig.virtAntRight ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        gpsConfig.virtAntRight = control->value.toFloat();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "500", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "GPS Antenna foreward offset", String( gpsConfig.virtAntForew ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        gpsConfig.virtAntForew = control->value.toFloat();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "500", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Heading angle correction\n(90 for left/right antenna)", String( gpsConfig.headingAngleCorrection ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        gpsConfig.headingAngleCorrection = control->value.toFloat();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "500", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Distance between Antennas max Deviation", String( gpsConfig.AntDistDeviationFactor ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        gpsConfig.AntDistDeviationFactor = control->value.toFloat();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "1.99", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Max heading change (Degrees/Second)", String( gpsConfig.MaxHeadChangPerSec ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        gpsConfig.MaxHeadChangPerSec = control->value.toFloat();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "50", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }

    ESPUI.addControl( ControlType::Switcher, "Check UBX flags", gpsConfig.checkUBXFlags ? "1" : "0", ControlColor::Peterriver, tab,
    []( Control * control, int id ) {
      gpsConfig.checkUBXFlags = control->value.toInt() == 1;
    } );
    ESPUI.addControl( ControlType::Switcher, "Filter position on weak GPS signal", gpsConfig.filterGPSposOnWeakSignal ? "1" : "0", ControlColor::Peterriver, tab,
    []( Control * control, int id ) {
      gpsConfig.filterGPSposOnWeakSignal = control->value.toInt() == 1;
    } );
    ESPUI.addControl( ControlType::Switcher, "Correct position with roll", gpsConfig.GPSPosCorrByRoll ? "1" : "0", ControlColor::Peterriver, tab,
    []( Control * control, int id ) {
      gpsConfig.GPSPosCorrByRoll = control->value.toInt() == 1;
    } );
  }

  // AOG Messages Tab
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "AOG Messages", "AOG Messages" );

    ESPUI.addControl( ControlType::Switcher, "Send $PAOGI", gpsConfig.sendOGI ? "1" : "0", ControlColor::Peterriver, tab,
    []( Control * control, int id ) {
      gpsConfig.sendOGI = control->value.toInt() == 1;
    } );
    ESPUI.addControl( ControlType::Switcher, "Send $VTG", gpsConfig.sendVTG ? "1" : "0", ControlColor::Peterriver, tab,
    []( Control * control, int id ) {
      gpsConfig.sendVTG = control->value.toInt() == 1;
    } );
    ESPUI.addControl( ControlType::Switcher, "Send $GGA", gpsConfig.sendGGA ? "1" : "0", ControlColor::Peterriver, tab,
    []( Control * control, int id ) {
      gpsConfig.sendGGA = control->value.toInt() == 1;
    } );
    ESPUI.addControl( ControlType::Switcher, "Send $HDT", gpsConfig.sendHDT ? "1" : "0", ControlColor::Peterriver, tab,
    []( Control * control, int id ) {
      gpsConfig.sendHDT = control->value.toInt() == 1;
    } );
  }

  // Serial NMEA out Tab
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "Serial NMEA out", "Serial NMEA out" );

    ESPUI.addControl( ControlType::Switcher, "Send $VTG", gpsConfig.sendSerialNmeaVTG ? "1" : "0", ControlColor::Peterriver, tab,
    []( Control * control, int id ) {
      gpsConfig.sendSerialNmeaVTG = control->value.toInt() == 1;
    } );
    ESPUI.addControl( ControlType::Switcher, "Send $GGA", gpsConfig.sendSerialNmeaGGA ? "1" : "0", ControlColor::Peterriver, tab,
    []( Control * control, int id ) {
      gpsConfig.sendSerialNmeaGGA = control->value.toInt() == 1;
    } );
    ESPUI.addControl( ControlType::Switcher, "Send $HDT", gpsConfig.sendSerialNmeaHDT ? "1" : "0", ControlColor::Peterriver, tab,
    []( Control * control, int id ) {
      gpsConfig.sendSerialNmeaHDT = control->value.toInt() == 1;
    } );
    ESPUI.addControl( ControlType::Switcher, "Send $RMC", gpsConfig.sendSerialNmeaRMC ? "1" : "0", ControlColor::Peterriver, tab,
    []( Control * control, int id ) {
      gpsConfig.sendSerialNmeaRMC = control->value.toInt() == 1;
    } );

    {
      uint16_t baudrate = ESPUI.addControl( ControlType::Select, "Serial NMEA Baudrate*", String( gpsConfig.serialNmeaBaudrate ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        gpsConfig.serialNmeaBaudrate = control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "4800", "4800", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "9600", "9600", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "19200", "19200", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "38400", "38400", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "57600", "57600", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "115200", "115200", ControlColor::Alizarin, baudrate );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "NMEA Messages per Second*", String( gpsConfig.serialNmeaMessagesPerSec ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        gpsConfig.serialNmeaMessagesPerSec = control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "8", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }
  }

  // Velocity PWM Tab
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "Velocity PWM out", "Velocity PWM out" );

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Hertz / Mile Per Hour", String( gpsConfig.velocityHzPerMPH ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        gpsConfig.velocityHzPerMPH = control->value.toInt();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "500", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }
  }

  // Serial debug Tab
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "USB Serial debug", "USB Serial debug" );

    ESPUI.addControl( ControlType::Switcher, "Debug Mode", gpsConfig.debugmode ? "1" : "0", ControlColor::Peterriver, tab,
    []( Control * control, int id ) {
      gpsConfig.debugmode = control->value.toInt() == 1;
    } );
    ESPUI.addControl( ControlType::Switcher, "Debug UBX", gpsConfig.debugmodeUBX ? "1" : "0", ControlColor::Peterriver, tab,
    []( Control * control, int id ) {
      gpsConfig.debugmodeUBX = control->value.toInt() == 1;
    } );
    ESPUI.addControl( ControlType::Switcher, "Debug heading", gpsConfig.debugmodeHeading ? "1" : "0", ControlColor::Peterriver, tab,
    []( Control * control, int id ) {
      gpsConfig.debugmodeHeading = control->value.toInt() == 1;
    } );
    ESPUI.addControl( ControlType::Switcher, "Debug virtual antenna", gpsConfig.debugmodeVirtAnt ? "1" : "0", ControlColor::Peterriver, tab,
    []( Control * control, int id ) {
      gpsConfig.debugmodeVirtAnt = control->value.toInt() == 1;
    } );
    ESPUI.addControl( ControlType::Switcher, "Debug filter position", gpsConfig.debugmodeFilterPos ? "1" : "0", ControlColor::Peterriver, tab,
    []( Control * control, int id ) {
      gpsConfig.debugmodeFilterPos = control->value.toInt() == 1;
    } );
    ESPUI.addControl( ControlType::Switcher, "Debug raw", gpsConfig.debugmodeRAW ? "1" : "0", ControlColor::Peterriver, tab,
    []( Control * control, int id ) {
      gpsConfig.debugmodeRAW = control->value.toInt() == 1;
    } );
  }

  // Default Configurations Tab
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "Configurations", "Configurations" );

    ESPUI.addControl( ControlType::Label, "OTA Update:", "<a href='/update'>Update</a>", ControlColor::Carrot, tab );

    ESPUI.addControl( ControlType::Label, "Download the config:", "<a href='config.json'>Configuration</a>", ControlColor::Carrot, tab );

    ESPUI.addControl( ControlType::Label, "Upload the config:", "<form method='POST' action='/upload-config' enctype='multipart/form-data'><input name='f' type='file'><input type='submit'>ESP32 will restart after submitting</form>", ControlColor::Carrot, tab );

    tabConfigurations = tab;

  }
  
  static String title;

  title = "Dual GPS :: ";

  title += gpsConfig.hostname;
  ESPUI.begin( title.c_str() );

  ESPUI.server->on( "/config.json", HTTP_GET, []( AsyncWebServerRequest * request ) {
    request->send( SPIFFS, "/config.json", "application/json", true );
  } );
  
  // upload a file to /upload-config
  ESPUI.server->on( "/upload-config", HTTP_POST, []( AsyncWebServerRequest * request ) {
    request->send( 200 );
  }, [tabConfigurations]( AsyncWebServerRequest * request, String filename, size_t index, uint8_t* data, size_t len, bool final ) {
    if( !index ) {
      request->_tempFile = SPIFFS.open( "/config.json", "w" );
    }

    if( request->_tempFile ) {
      if( len ) {
        request->_tempFile.write( data, len );
      }

      if( final ) {
        request->_tempFile.close();
        delay(10);
        ESP.restart();
      }
    }
  } );
}
