#ifndef MotorUnit_h
#define MotorUnit_h
#include "memory"
// #include "grbl.h"

#include <Arduino.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "../MiniPID/MiniPID.h"     //https://github.com/tekdemo/MiniPID
#include "../DC Motor/DCMotor.h"
#include "../AS5600/AS5600.h"
#include "../../Logging.h"

class MotorUnit{

public:
    MotorUnit();
    std::unique_ptr<MiniPID> positionPID;
    std::unique_ptr<MiniPID> velocityPID;
    DCMotor motor;
    AS5600 encoder;
    void begin(int forwardPin,
               int backwardPin,
               int readbackPin,
               int encoderNum,
               int channel1,
               int channel2);
    void zero();
    bool testEncoder();
    int readAngle();
    int test();
    void setTarget(double newTarget);
    int setPosition(double newPosition);
    double getError();
    bool retract(double targetLength);
    bool comply(unsigned long *timeLastMoved, double *lastPosition, double *amtToMove, double maxSpeed);
    double getTarget();
    void repeatabilityTest();
    double getCurrent();
    double getPosition();
    void stop();
    double recomputePID();
    int  recomputeVelocityPID();
    void updateEncoderPosition();
    void decompressBelt();
    void fullOut();
    void sendVoltage(double voltage);

private:

    double _mmPerRevolution = 44.0;
    double lastInterval = 0.001;
    unsigned long lastUpdate = millis();

    double p = 1600; //2600
    double i = 10; //10
    double d = 0; //0

    bool disabled = false;

    int output = 0;
    double currentState = 0.0;
    double setpoint = 0.0;
    double errorDist = 0.0;
    
    long angleTotal = 0;
    long angleCurrent  = 0;
    long anglePrevious = 0;
    
    int _stallThreshold = 25; //The number of times in a row needed to trigger a warning
    int _stallCurrent = 27;   //The current threshold needed to count
    int _stallCount = 0;
    int _axisID = 0; //A number used to identify which axis this is...is the forward pin
    void (*_webPrint) (uint8_t client, const char* format, ...);
    double removeDeadband(double commandPWM);
    int _numPosErrors = 0; //Keeps track of the number of position errors in a row to detect a stall
    void selectEncoder();
    
    int _forwardPin;
    int _backwardPin;
    int _readbackPin;
    byte _encoderNum;
    int _channel1;
    int _channel2;

};

#endif
