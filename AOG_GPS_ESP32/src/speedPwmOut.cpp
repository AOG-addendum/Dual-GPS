
#include "main.hpp"

double mphPwm;

void SpeedPWM ( void* z ){
  constexpr TickType_t xFrequency = 20;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for( ;; ) {

    mphPwm = float( UBXPVT1[UBXRingCount1].gSpeed ) * gpsConfig.velocityHzPerMPH;
    mphPwm *= 0.22; // 1 gSpeed = 0.0036 km/h = 0.0022369363 mile/h 
    mphPwm /= 100;
    ledcWriteTone( gpsConfig.gpioVelocityPWM, mphPwm );
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void initSpeedPWM() {

  ledcAttach( gpsConfig.gpioVelocityPWM, gpsConfig.velocityHzPerMPH, LEDC_TIMER_10_BIT );
  ledcWrite( gpsConfig.gpioVelocityPWM, 512 ); // 50% duty cycle
  xTaskCreate( SpeedPWM, "SpeedPWM", 2048, NULL, 2, NULL );
}
