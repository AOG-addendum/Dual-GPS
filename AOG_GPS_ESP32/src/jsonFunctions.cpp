

#include <memory>

#include <LittleFS.h>

#include "main.hpp"
#include "jsonFunctions.hpp"

void loadSavedConfig() {
  {
    auto j = loadJsonFromFile( "/config.json" );
    parseJsonToGpsConfig( j, gpsConfig );
  }
}

void saveConfig() {
  {
    const auto j = parseGpsConfigToJson( gpsConfig );
    saveJsonToFile( j, "/config.json" );
  }
}

json loadJsonFromFile( const char* fileName ) {
  json j;

  if( LittleFS.exists( fileName ) ) {
    File file = LittleFS.open( fileName, "r" );

    if( file ) {
      std::vector<uint8_t> data;
      data.resize( file.size() );

      file.read( data.data(), file.size() );

      try {
        j = json::parse( data/*, nullptr, false*/ );
      } catch( json::exception& e ) {
        // output exception information
        Serial.print( " LOAD/PARSE FILE " );
        Serial.print( "message: " );
        Serial.println( e.what() );
        Serial.print( "exception id: " );
        Serial.println( e.id );
      }
    } else {
      Serial.print( "Could not open file for reading: " );
      Serial.println( fileName );
      Serial.flush();
    }

    file.close();
  }
  else {
    Serial.print( fileName );
    Serial.println( " does not exist" );
  }

  return j;
}

void saveJsonToFile( const json& json, const char* fileName ) {
  // pretty print with 2 spaces indentation
  auto data = json.dump( 2 );

  bool newfile = false;

  if( !LittleFS.exists( fileName ) ) {
    newfile = true;
  }
    
  File file = LittleFS.open( fileName, "w", newfile );
  
  if( file && !file.isDirectory() ) {
    file.write( ( uint8_t* )data.c_str(), data.size() );
  } else {
    Serial.print( "Could not open file for writing: " );
    Serial.println( fileName );
    Serial.flush();
  }

  file.close();
}

json parseGpsConfigToJson( const GPS_Config& config ) {
  json j;

  j["wifi"]["ssid"] = config.ssid;
  j["wifi"]["password"] = config.password;
  j["wifi"]["hostname"] = config.hostname;
  j["wifi"]["retainSettings"] = config.retainWifiSettings;
  j["wifi"]["WifiLedOnLevel"] = config.WifiLedOnLevel;

  j["gps"]["AntennaDist"] = int( config.AntDist );
  j["gps"]["AntennaHeight"] = config.AntHeight;
  j["gps"]["virtualAntennaRight"] = config.virtAntRight;
  j["gps"]["virtualAntennaForeword"] = config.virtAntForew;
  j["gps"]["headingAngleCorrection"] = config.headingAngleCorrection;
  j["gps"]["AntennaDistanceDeviationFactor"] = config.AntDistDeviationFactor;
  j["gps"]["MaxHeadingChangePerSecond"] = config.MaxHeadChangPerSec;
  j["gps"]["checkUBXFlags"] = config.checkUBXFlags;
  j["gps"]["filterGPSpositionOnWeakSignal"] = config.filterGPSposOnWeakSignal;
  j["gps"]["GPSPositionCorrectionByRoll"] = config.GPSPosCorrByRoll;
  j["gps"]["rollAngleCorrection"] = config.rollAngleCorrection;
  
  j["serialNmea"]["sendVTG"] = config.sendSerialNmeaVTG;
  j["serialNmea"]["sendGGA"] = config.sendSerialNmeaGGA;
  j["serialNmea"]["sendHDT"] = config.sendSerialNmeaHDT;
  j["serialNmea"]["sendRMC"] = config.sendSerialNmeaRMC;
  j["serialNmea"]["serialNmeaBaudrate"] = config.serialNmeaBaudrate;
  j["serialNmea"]["serialNmeaMessagesPerSecond"] = config.serialNmeaMessagesPerSec;

  j["messages"]["sendOGI"] = config.sendOGI;
  j["messages"]["sendVTG"] = config.sendVTG;
  j["messages"]["sendGGA"] = config.sendGGA;
  j["messages"]["sendHDT"] = config.sendHDT;

  j["PWM"]["velocityHzPerMPH"] = config.velocityHzPerMPH;

  j["debug"]["debugmode"] = config.debugmode;
  j["debug"]["debugmodeUBX"] = config.debugmodeUBX;
  j["debug"]["debugmodeHeading"] = config.debugmodeHeading;
  j["debug"]["debugmodeVirtualAntenna"] = config.debugmodeVirtAnt;
  j["debug"]["debugmodeFilterPosition"] = config.debugmodeFilterPos;
  j["debug"]["debugmodeRAW"] = config.debugmodeRAW;

  j["connection"]["baudrate"] = config.baudrate;
  j["connection"]["enableOTA"] = config.enableOTA;

  j["connection"]["aog"]["sendFrom"] = config.aogPortSendFrom;
  j["connection"]["aog"]["listenTo"] = config.aogPortListenTo;
  j["connection"]["aog"]["sendTo"] = config.aogPortSendTo;

  return j;
}

void parseJsonToGpsConfig( json& j, GPS_Config& config ) {
  if( j.is_object() ) {
    try {
      {
        std::string str;
        j["wifi"]["ssid"].get_to( str );
        memset( config.ssid, 0, sizeof( config.ssid ) );
        memcpy( config.ssid, str.c_str(), str.size() );
      }
      {
        std::string str;
        j["wifi"]["password"].get_to( str );
        memset( config.password, 0, sizeof( config.password ) );
        memcpy( config.password, str.c_str(), str.size() );
      }
      {
        std::string str;
        j["wifi"]["hostname"].get_to( str );
        memset( config.hostname, 0, sizeof( config.hostname ) );
        memcpy( config.hostname, str.c_str(), str.size() );
      }
      j["wifi"]["retainSettings"].get_to( gpsConfig.retainWifiSettings );
      j["wifi"]["WifiLedOnLevel"].get_to( gpsConfig.WifiLedOnLevel );
      
      j["gps"]["AntennaDist"].get_to( gpsConfig.AntDist );

      j["gps"]["AntennaHeight"].get_to( gpsConfig.AntHeight );
      j["gps"]["virtualAntennaRight"].get_to( gpsConfig.virtAntRight );
      j["gps"]["virtualAntennaForeword"].get_to( gpsConfig.virtAntForew );
      j["gps"]["headingAngleCorrection"].get_to( gpsConfig.headingAngleCorrection );
      j["gps"]["AntennaDistanceDeviationFactor"].get_to( gpsConfig.AntDistDeviationFactor );
      j["gps"]["MaxHeadingChangePerSecond"].get_to( gpsConfig.MaxHeadChangPerSec );
      j["gps"]["checkUBXFlags"].get_to( gpsConfig.checkUBXFlags );
      j["gps"]["filterGPSpositionOnWeakSignal"].get_to( gpsConfig.filterGPSposOnWeakSignal );
      j["gps"]["GPSPositionCorrectionByRoll"].get_to( gpsConfig.GPSPosCorrByRoll );
      j["gps"]["rollAngleCorrection"].get_to( gpsConfig.rollAngleCorrection );

      j["serialNmea"]["sendVTG"].get_to( gpsConfig.sendSerialNmeaVTG );
      j["serialNmea"]["sendGGA"].get_to( gpsConfig.sendSerialNmeaGGA );
      j["serialNmea"]["sendHDT"].get_to( gpsConfig.sendSerialNmeaHDT );
      j["serialNmea"]["sendRMC"].get_to( gpsConfig.sendSerialNmeaRMC );
      j["serialNmea"]["serialNmeaBaudrate"].get_to( gpsConfig.serialNmeaBaudrate );
      j["serialNmea"]["serialNmeaMessagesPerSecond"].get_to( gpsConfig.serialNmeaMessagesPerSec );

      j["messages"]["sendOGI"].get_to( gpsConfig.sendOGI );
      j["messages"]["sendVTG"].get_to( gpsConfig.sendVTG );
      j["messages"]["sendGGA"].get_to( gpsConfig.sendGGA );
      j["messages"]["sendHDT"].get_to( gpsConfig.sendHDT );

      j["PWM"]["velocityHzPerMPH"].get_to( gpsConfig.velocityHzPerMPH );

      j["debug"]["debugmode"].get_to( gpsConfig.debugmode );
      j["debug"]["debugmodeUBX"].get_to( gpsConfig.debugmodeUBX );
      j["debug"]["debugmodeHeading"].get_to( gpsConfig.debugmodeHeading );
      j["debug"]["debugmodeVirtualAntenna"].get_to( gpsConfig.debugmodeVirtAnt );
      j["debug"]["debugmodeFilterPosition"].get_to( gpsConfig.debugmodeFilterPos );
      j["debug"]["debugmodeRAW"].get_to( gpsConfig.debugmodeRAW );

      j["connection"]["baudrate"].get_to( gpsConfig.baudrate );
      j["connection"]["enableOTA"].get_to( gpsConfig.enableOTA );

      j["connection"]["aog"]["sendFrom"].get_to( gpsConfig.aogPortSendFrom );
      j["connection"]["aog"]["listenTo"].get_to( gpsConfig.aogPortListenTo );
      j["connection"]["aog"]["sendTo"].get_to( gpsConfig.aogPortSendTo );

    } catch( json::exception& e ) {
      // output exception information
      Serial.print( " PARSE TO CONFIG " );
      Serial.print( "message: " );
      Serial.println( e.what() );
      Serial.print( "exception id: " );
      Serial.println( e.id );
      Serial.flush();
    }
  }
}

void loadDiagnostics() {
  {
    auto j = loadJsonFromFile( "/diagnostics.json" );
    parseJsonToDiagnostics( j, diagnostics );
  }
}

void saveDiagnostics() {
  {
    const auto j = parseDiagnosticsToJson( diagnostics );
    saveJsonToFile( j, "/diagnostics.json" );
  }
}

json parseDiagnosticsToJson( const Diagnostics& diagnostics ) {
  json j;

  j["count"]["badChecksumNavPVT"] = diagnostics.badChecksumNavPVTCount;
  j["count"]["wrongLengthNavPVT"] = diagnostics.wrongLengthNavPVTCount;
  j["count"]["badChecksumRelPosNED"] = diagnostics.badChecksumRelPosNEDCount;
  j["count"]["wrongLengthRelPosNED"] = diagnostics.wrongLengthRelPosNEDCount;

  return j;
}

void parseJsonToDiagnostics( json& j, Diagnostics& diagnostics ) {
  if( j.is_object() ) {
    try {
      j["count"]["badChecksumNavPVT"].get_to( diagnostics.badChecksumNavPVTCount );
      j["count"]["wrongLengthNavPVT"].get_to( diagnostics.wrongLengthNavPVTCount );
      j["count"]["badChecksumRelPosNED"].get_to( diagnostics.badChecksumRelPosNEDCount );
      j["count"]["wrongLengthRelPosNED"].get_to( diagnostics.wrongLengthRelPosNEDCount );

    } catch( json::exception& e ) {
      // output exception information
      Serial.print( " PARSE TO DIAGNOSTICS " );
      Serial.print( "message: " );
      Serial.println( e.what() );
      Serial.print( "exception id: " );
      Serial.println( e.id );
      Serial.flush();
    }
  }
}
