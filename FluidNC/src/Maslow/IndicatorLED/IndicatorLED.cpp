/***************************************************
 *  This is a library for controlling the indicator LEDs on the Maslow main board
 *
 *  By Bar Smith for Maslow CNC
 ****************************************************/
#include "IndicatorLED.h"

IndicatorLED::IndicatorLED(){
}

void IndicatorLED::test(){
   // _driver->setPWM(_wifiStatusLED, 60000);
   // _driver->setPWM(_axisHealthLED, 60000);
   // _driver->write();
   Serial.println("Indicator LED test");
}