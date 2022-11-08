
#include "main.hpp"

void NmeaOut ( void* z ){
  TickType_t xFrequency = 1000 / gpsConfig.serialNmeaMessagesPerSec;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for( ;; ) {

    if( gpsConfig.sendSerialNmeaGGA ){
      for( byte n = 0; n < ( GGAdigit - 1 ); n++ ){
        NmeaTransmitter.write( GGABuffer[n] ); 
      }
      NmeaTransmitter.println();
    }
    if( gpsConfig.sendSerialNmeaVTG ){
      for( byte n = 0; n < ( VTGdigit - 1 ); n++ ){ 
        NmeaTransmitter.write( VTGBuffer[n] ); 
      }
      NmeaTransmitter.println();
    }
    if( gpsConfig.sendSerialNmeaHDT ){
      for( byte n = 0; n < ( HDTdigit - 1 ); n++ ){
        NmeaTransmitter.write( HDTBuffer[n] ); 
      }
      NmeaTransmitter.println();
    }
    
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void initNmeaOut() {
  xTaskCreate( NmeaOut, "NmeaOut", 2048, NULL, 2, NULL );
}

