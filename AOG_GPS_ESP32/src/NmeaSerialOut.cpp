
#include "main.hpp"
uint64_t previousMillis = 0;
uint16_t nmeaMessageDelay;

void NmeaOut ( void* z ){
  TickType_t xFrequency = (( uint16_t ) 1000 / gpsConfig.serialNmeaMessagesPerSec ) - 10;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint8_t HDTReceiveBuffer[24], VTGReceiveBuffer[55], GGAReceiveBuffer[80], RMCReceiveBuffer[80];

	for( ;;  ){
    
    if( gpsConfig.sendSerialNmeaGGA ){
      if( xQueueReceive( GGAQueue, &GGAReceiveBuffer, 0 ) == pdTRUE ){
        NmeaTransmitter.write( GGAReceiveBuffer, 80 );
      }
    }
    if( gpsConfig.sendSerialNmeaVTG ){
      if( xQueueReceive( VTGQueue, &VTGReceiveBuffer, 0 ) == pdTRUE ){
        NmeaTransmitter.write( VTGReceiveBuffer, 55 );
      }
    }
    if( gpsConfig.sendSerialNmeaHDT ){
      if( xQueueReceive( HDTQueue, &HDTReceiveBuffer, 0 ) == pdTRUE ){
        NmeaTransmitter.write( HDTReceiveBuffer, 24 );
      }
    }
    if( gpsConfig.sendSerialNmeaRMC ){
      if( xQueueReceive( RMCQueue, &RMCReceiveBuffer, 0 ) == pdTRUE ){
        NmeaTransmitter.write( RMCReceiveBuffer, 80 );
      }
    }
    xTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void initNmeaOut( ){
  xTaskCreate( NmeaOut, "NmeaOut", 3096, NULL, 5, NULL );
}