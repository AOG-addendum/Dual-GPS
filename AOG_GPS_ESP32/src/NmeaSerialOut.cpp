
#include "main.hpp"
uint64_t previousMillis = 0;
uint16_t nmeaMessageDelay;

void NmeaOut (){
	if ( millis() - previousMillis > nmeaMessageDelay ){
		previousMillis += nmeaMessageDelay;
    if( gpsConfig.sendSerialNmeaGGA ){
      NmeaTransmitter.write( GGABuffer, GGAdigit - 1 ); 
      NmeaTransmitter.println();
    }
    if( gpsConfig.sendSerialNmeaVTG ){
      NmeaTransmitter.write( VTGBuffer, VTGdigit - 1 ); 
      NmeaTransmitter.println();
    }
    if( gpsConfig.sendSerialNmeaHDT ){
      NmeaTransmitter.write( HDTBuffer, HDTdigit - 1 ); 
      NmeaTransmitter.println();
    }
  }
}

