#ifndef IndicatorLED_h
#define IndicatorLED_h 

#include <Arduino.h>

class IndicatorLED{

public:
    IndicatorLED();
    void test();


private:
    void (*_webPrint) (uint8_t client, const char* format, ...);
    int _wifiStatusLED = 11;
    int _axisHealthLED = 10;
    int _maxBrightness = 65535; //Absolute max is 65535


};

#endif