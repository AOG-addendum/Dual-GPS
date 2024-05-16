
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

    bool power = digitalRead( gpsConfig.gpioDcPowerGood );
    if( power == LOW ) {
      str = "DC power low: not running";
    } else if( powerUnstable == true ){
      str = "DC power unstable ";
      str += (( time_t ) millis() - powerUnstableMillis ) / 1000 ;
      str += " second(s) ago: running";
    } else if( power == HIGH ){
      str = "DC power good: running";
    }
    if( NavPVTValid == true ){
      str += "\nValid ";
    } else {
      str += "\nInvalid ";
    }
    str += "NAV-PVT message received: ";
    time_t NAV_millis_elapsed = millis() - NavPvtMillis;
    if( NAV_millis_elapsed > 1000 ){
      str += ( NAV_millis_elapsed / 1000 );
      str += " seconds ago";
    } else {
      str += NAV_millis_elapsed;
      str += " millis ago";
    }
    if( RelPosNEDValid == true ){
      str += "\nValid ";
    } else {
      str += "\nInvalid ";
    }
    str += "RelPosNED message received: ";
    time_t RPN_millis_elapsed = millis() - RelPosNedMillis;
    if( RPN_millis_elapsed > 1000 ){
      str += ( RPN_millis_elapsed / 1000 );
      str += " seconds ago";
    } else {
      str += RPN_millis_elapsed;
      str += " millis ago";
    }

    Control* labelGpsReceiversHandle = ESPUI.getControl( labelGpsReceivers );
    if( NavPVTValid == false or RelPosNEDValid == false or power == LOW or powerUnstable == true
        or NAV_millis_elapsed > 150 or RPN_millis_elapsed > 150 ){
      labelGpsReceiversHandle->color = ControlColor::Alizarin;
    } else {
      labelGpsReceiversHandle->color = ControlColor::Turquoise;
    }
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