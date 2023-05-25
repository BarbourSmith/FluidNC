
/*

This class defines the maslow machine and the hardware that it will interact with on the board.

This class is a singleton (meaning it can be created only once, and then accessed from anywhere in the code)
because the machine is a single entity, but we need to talk to it in multiple places within FluidNC.

*/

#include <Arduino.h>
#include "../System.h"         // sys.*
#include "../Logging.h"
#include "../Maslow/MotorUnit/MotorUnit.h"
#include "../Maslow/IndicatorLED/IndicatorLED.h"
#include "../Maslow/Calibration/calibration.h"

// Maslow specific defines
#define TLEncoderLine 0
#define TREncoderLine 2
#define BLEncoderLine 1
#define BREncoderLine 3

#define motorPWMFreq 2000
#define motorPWMRes 10

#define tlIn1Pin 45
#define tlIn1Channel 0
#define tlIn2Pin 21
#define tlIn2Channel 1
#define tlADCPin 18

#define trIn1Pin 42
#define trIn1Channel 2
#define trIn2Pin 41
#define trIn2Channel 3
#define trADCPin 6

#define blIn1Pin 37
#define blIn1Channel 4
#define blIn2Pin 36
#define blIn2Channel 5
#define blADCPin 8

#define brIn1Pin 9
#define brIn1Channel 6
#define brIn2Pin 3
#define brIn2Channel 7
#define brADCPin 7

#define DC_TOP_LEFT_MM_PER_REV 44
#define DC_Z_AXIS_MM_PER_REV 1

class Maslow_ {
  private:
    Maslow_() = default; // Make constructor private
    //The xy coordinates of each of the anchor points
    float tlX;
    float tlY;
    float tlZ;
    float trX;
    float trY;
    float trZ;
    float blX;
    float blY;
    float blZ;
    float brX;
    float brY;
    float brZ;
    float centerX;
    float centerY;

    float tlTension;
    float trTension;

    float beltEndExtension;
    float armLength;

    bool axisBLHomed;
    bool axisBRHomed;
    bool axisTRHomed;
    bool axisTLHomed;
    bool calibrationInProgress;  //Used to turn off regular movements during calibration

    //The number of times that the encoder has read 0 or 16383 which can indicate a connection failure
    int tlEncoderErrorCount = 0;
    int trEncoderErrorCount = 0;
    int blEncoderErrorCount = 0;
    int brEncoderErrorCount = 0;

    //Used to keep track of how often the PID controller is updated
    unsigned long lastCallToPID = millis();

    IndicatorLED indicators;
    MotorUnit axisTL;
    MotorUnit axisTR;
    MotorUnit axisBL;
    MotorUnit axisBR;

  public:
    static Maslow_ &getInstance(); // Accessor for singleton instance

    Maslow_(const Maslow_ &) = delete; // no copying
    Maslow_ &operator=(const Maslow_ &) = delete;

  public:
    void begin();
    void updateCenterXY();
    void recomputePID();
    void setTargets(float xTarget, float yTarget, float zTarget);
    void computeTensions(float x, float y);
    float computeTL(float x, float y, float z);
    float computeTR(float x, float y, float z);
    float computeBL(float x, float y, float z);
    float computeBR(float x, float y, float z);
    void runCalibration();
    void lowerBeltsGoSlack();
    float takeMeasurementAvg(float lengths[]);
    void takeMeasurementAvgWithCheck(float lengths[]);
    void moveWithSlack(float x, float y);
    void computeFrameDimensions(float lengthsSet1[], float lengthsSet2[], float machineDimensions[]);
    float computeVertical(float firstUpper, float firstLower, float secondUpper, float secondLower);
    void takeMeasurement(float lengths[]);
    void printMeasurements(float lengths[]);
    void printPrecision(float precision);
    void takeUpInternalSlack();
};

extern Maslow_ &Maslow;