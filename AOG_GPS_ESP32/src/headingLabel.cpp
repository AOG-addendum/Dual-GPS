
#include "main.hpp"

void headingDisplay ( void* z ){
  constexpr TickType_t xFrequency = 100;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  String str;
  str.reserve( 500 );

  for( ;; ) {

    str = "Heading from Dual GPS: ";
    str += HeadingRelPosNED;
    str += "\nHeading from VTG: ";
    str += HeadingVTG;
    str += "\nHeading from Mix: ";
    str += HeadingMix;

    Control* labelHeadingHandle = ESPUI.getControl( labelHeading );
    labelHeadingHandle->value = str;
    ESPUI.updateControlAsync( labelHeadingHandle );

    if( digitalRead( gpsConfig.gpioDcPowerGood ) == HIGH ){
      str = "DC power good: running";
    } else {
      str = "DC power low: not running";
    }
    str += "\nNAV-PVT message received: ";
    time_t millis_elapsed = millis() - NavPvtMillis;
    if( millis_elapsed > 1000 ){
      str += ( millis_elapsed / 1000 );
      str += " seconds ago";
    } else {
      str += millis_elapsed;
      str += " millis ago";
    }
    str += "\nRelPosNED message received: ";
    millis_elapsed = millis() - RelPosNedMillis;
    if( millis_elapsed > 1000 ){
      str += ( millis_elapsed / 1000 );
      str += " seconds ago";
    } else {
      str += millis_elapsed;
      str += " millis ago";

    }

    Control* labelGpsReceiversHandle = ESPUI.getControl( labelGpsReceivers );
    labelGpsReceiversHandle->value = str;
    ESPUI.updateControlAsync( labelGpsReceiversHandle );

    str = "Pwm: ";
    str += mphPwm;
    str += "Hz\ngSpeed: ";
    str += UBXPVT1[UBXRingCount1].gSpeed;

    Control* labelPwmOutHandle = ESPUI.getControl( labelPwmOut );
    labelPwmOutHandle->value = str;
    ESPUI.updateControlAsync( labelPwmOutHandle );

		vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void initHeadingDisplay() {
  xTaskCreate( headingDisplay, "headingDisplay", 2048, NULL, 2, NULL );
}