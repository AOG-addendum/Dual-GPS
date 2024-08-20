
#include "main.hpp"

void diagnosticDisplay ( void* z ){
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
    str += "\nNAV-PVT message received: ";
    time_t elapsedNavPvtMillis = millis() - NavPvtMillis;
    if( elapsedNavPvtMillis > 1000 ){
      str += ( elapsedNavPvtMillis / 1000 );
      str += " seconds ago";
    } else {
      str += NavPvtMillis - previousNavPvtMillis;
      str += " millis apart";
    }
    str += "\nRelPosNED message received: ";
    time_t elapsedRelPosNedMillis = millis() - RelPosNedMillis;
    if( elapsedRelPosNedMillis > 1000 ){
      str += ( elapsedRelPosNedMillis / 1000 );
      str += " seconds ago";
    } else {
      str += RelPosNedMillis - previousRelPosNedMillis;
      str += " millis apart";
    }

    Control* labelGpsReceiversHandle = ESPUI.getControl( labelGpsReceivers );
    if( power == LOW or powerUnstable == true or elapsedNavPvtMillis > 1000 or elapsedRelPosNedMillis > 1000 ){
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

    str = "Max millis: ";
    str += gpsHzMaxMillis;
    str += "\nMin millis: ";
    str += gpsHzMinMillis;
    str += "\nCurrent millis: ";
    str += gpsHzCurrentMillis;
    str += "\nNavPVT bad checksum: ";
    str += diagnostics.badChecksumNavPVTCount;
    str += "\nNavPVT wrong length: ";
    str += diagnostics.wrongLengthNavPVTCount;
    str += "\nRelPosNED bad checksum: ";
    str += diagnostics.badChecksumRelPosNEDCount;
    str += "\nRelPosNED wrong length: ";
    str += diagnostics.wrongLengthRelPosNEDCount;

    Control* labelGpsMessageHzHandle = ESPUI.getControl( labelGpsMessageHz );
    labelGpsMessageHzHandle->value = str;
    ESPUI.updateControlAsync( labelGpsMessageHzHandle );

		vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void initDiagnosticDisplay() {
  xTaskCreate( diagnosticDisplay, "diagnosticDisplay", 2048, NULL, 2, NULL );
}