
#include "main.hpp"
uint64_t previousMillis = 0;
uint16_t nmeaMessageDelay;

void NmeaOut ( void* z ){
  constexpr TickType_t xFrequency = 190;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint8_t HDTReceiveBuffer[20], VTGReceiveBuffer[50], GGAReceiveBuffer[80], RMCReceiveBuffer[80];

	for( ;;  ){
    
    if( gpsConfig.sendSerialNmeaGGA ){
      if( xQueueReceive( GGAQueue, &GGAReceiveBuffer, 0 ) == pdTRUE ){
        NmeaTransmitter.write( GGAReceiveBuffer, 80 );
      }
    }
    if( gpsConfig.sendSerialNmeaVTG ){
      if( xQueueReceive( VTGQueue, &VTGReceiveBuffer, 0 ) == pdTRUE ){
        NmeaTransmitter.write( VTGReceiveBuffer, 50 );
      }
    }
    if( gpsConfig.sendSerialNmeaHDT ){
      if( xQueueReceive( HDTQueue, &HDTReceiveBuffer, 0 ) == pdTRUE ){
        NmeaTransmitter.write( HDTReceiveBuffer, 20 );
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