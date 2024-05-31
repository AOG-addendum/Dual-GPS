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
    ESPUI.updateText( labelHeading, String( str ) );

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
    time_t NAV_millis_elapsed = millis() - NavPvtMillis;
    if( NAV_millis_elapsed > 1000 ){
      str += ( NAV_millis_elapsed / 1000 );
      str += " seconds ago";
    } else {
      str += NAV_millis_elapsed;
      str += " millis ago";
    }
    str += "\nRelPosNED message received: ";
    time_t RPN_millis_elapsed = millis() - RelPosNedMillis;
    if( RPN_millis_elapsed > 1000 ){
      str += ( RPN_millis_elapsed / 1000 );
      str += " seconds ago";
    } else {
      str += RPN_millis_elapsed;
      str += " millis ago";
    }

    if( power == LOW or powerUnstable == true or NAV_millis_elapsed > 150 or RPN_millis_elapsed > 150 ){
      ESPUI.setPanelStyle ( labelGpsReceivers, "background-color: #e32636" ); //#e32636 == ControlColor::Alizarin
    } else {
      ESPUI.setPanelStyle ( labelGpsReceivers, "background-color: #40e0d0" ); //#40e0d0 == Turquoise
    }
    ESPUI.updateText( labelGpsReceivers, String( str ) );

    str = "Pwm: ";
    str += mphPwm;
    str += "Hz\ngSpeed: ";
    str += UBXPVT1[UBXRingCount1].gSpeed;
    ESPUI.updateText( labelPwmOut, String( str ) );

  
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
    ESPUI.updateText( labelGpsMessageHz, String( str ) );

		xTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}



void initDiagnosticDisplay() {
  xTaskCreate( diagnosticDisplay, "diagnosticDisplay", 2048, NULL, 2, NULL );
}