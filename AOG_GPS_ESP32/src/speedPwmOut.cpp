
#include "main.hpp"

double mphPwm;

void SpeedPWM ( void* z ){
  constexpr TickType_t xFrequency = 20;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for( ;; ) {

    mphPwm = float( UBXPVT1[UBXRingCount1].gSpeed ) * gpsConfig.velocityHzPerMPH;
    mphPwm *= 0.0022369363; // 1 gSpeed = 0.0036 km/h = 0.0022369363 mile/h 
    ledcWriteTone( 0, mphPwm );
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void initSpeedPWM() {

  pinMode( ( uint8_t ) gpsConfig.gpioVelocityPWM, OUTPUT );
  ledcSetup( 0, ( uint8_t ) gpsConfig.velocityHzPerMPH, 8 );
  ledcAttachPin( ( uint8_t ) gpsConfig.gpioVelocityPWM, 0 );
  ledcWrite( 0, 128 );
  xTaskCreate( SpeedPWM, "SpeedPWM", 2048, NULL, 2, NULL );
}
