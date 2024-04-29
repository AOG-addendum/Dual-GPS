
#include "main.hpp"
uint64_t previousMillis = 0;
uint16_t nmeaMessageDelay;

void NmeaOut ( void* z ){
  constexpr TickType_t xFrequency = 200;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint8_t HDTReceiveBuffer[20], VTGReceiveBuffer[50], GGAReceiveBuffer[80], RMCReceiveBuffer[80];

	for( ;;  ){
    
    if( gpsConfig.sendSerialNmeaGGA ){
      xQueueReceive( GGAQueue, &GGAReceiveBuffer, portMAX_DELAY );
      NmeaTransmitter.write( GGAReceiveBuffer, GGAdigit - 1 ); 
      NmeaTransmitter.println();
    }
    if( gpsConfig.sendSerialNmeaVTG ){
      xQueueReceive( VTGQueue, &VTGReceiveBuffer, portMAX_DELAY );
      NmeaTransmitter.write( VTGReceiveBuffer, VTGdigit - 1 ); 
      NmeaTransmitter.println();
    }
    if( gpsConfig.sendSerialNmeaHDT ){
      xQueueReceive( HDTQueue, &HDTReceiveBuffer, portMAX_DELAY );
      NmeaTransmitter.write( HDTReceiveBuffer, HDTdigit - 1 ); 
      NmeaTransmitter.println();
    }
    if( gpsConfig.sendSerialNmeaRMC ){
      xQueueReceive( RMCQueue, &RMCReceiveBuffer, portMAX_DELAY );
      NmeaTransmitter.write( RMCReceiveBuffer, RMCdigit - 1 ); 
      NmeaTransmitter.println();
    }
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void initNmeaOut( ){
  xTaskCreate( NmeaOut, "NmeaOut", 3096, NULL, 5, NULL );
}