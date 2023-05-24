/*
	custom_code_template.cpp (copy and use your machine name)
	Part of Grbl_ESP32

	copyright (c) 2020 -	Bart Dring. This file was intended for use on the ESP32

  ...add your date and name here.

	CPU. Do not use this with Grbl for atMega328P

	Grbl_ESP32 is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	Grbl_ESP32 is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

	=======================================================================

This is a template for user-defined C++ code functions.  Grbl can be
configured to call some optional functions, enabled by #define statements
in the machine definition .h file.  Implement the functions thus enabled
herein.  The possible functions are listed and described below.

To use this file, copy it to a name of your own choosing, and also copy
Machines/template.h to a similar name.

Example:
Machines/my_machine.h
Custom/my_machine.cpp

Edit machine.h to include your Machines/my_machine.h file

Edit Machines/my_machine.h according to the instructions therein.

Fill in the function definitions below for the functions that you have
enabled with USE_ defines in Machines/my_machine.h

===============================================================================

*/
#include "maslow.h"

MotorUnit axisTL(tlIn1Pin, tlIn2Pin, tlADCPin, TLEncoderLine, tlIn1Channel, tlIn2Channel);
MotorUnit axisTR(trIn1Pin, trIn2Pin, trADCPin, TREncoderLine, trIn1Channel, trIn2Channel);
MotorUnit axisBL(blIn1Pin, blIn2Pin, blADCPin, BLEncoderLine, blIn1Channel, blIn2Channel);
MotorUnit axisBR(brIn1Pin, brIn2Pin, brADCPin, BREncoderLine, brIn1Channel, brIn2Channel);

IndicatorLED indicators();

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


#ifdef USE_MACHINE_INIT
/*
machine_init() is called when Grbl_ESP32 first starts. You can use it to do any
special things your machine needs at startup.
*/
void machine_init()
{   
    Serial.println("Testing reading from encoders: ");
    axisBL.testEncoder();
    axisBR.testEncoder();
    axisTR.testEncoder();
    axisTL.testEncoder();
    
    axisBL.zero();
    axisBR.zero();
    axisTR.zero();
    axisTL.zero();
    
    axisBLHomed = false;
    axisBRHomed = false;
    axisTRHomed = false;
    axisTLHomed = false;

    tlX = -8.339;
    tlY = 2209;
    tlZ = 172;
    trX = 3505; 
    trY = 2209;
    trZ = 111;
    blX = 0;
    blY = 0;
    blZ = 96;
    brX = 3505;
    brY = 0;
    brZ = 131;

    tlTension = 0;
    trTension = 0;
    
    //Recompute the center XY
    updateCenterXY();
    
    beltEndExtension = 30;
    armLength = 114;
}
#endif

void printToWeb (double precision){
    //log_info( "Calibration Precision: %fmm\n", precision);
}

//Called from protocol.cpp
void recomputePID(){
    //Stop everything but keep track of the encoder positions if we are idle or alarm. Unless doing calibration.
    if((sys.state == State::Idle || sys.state == State::Alarm) && !calibrationInProgress){
        axisBL.stop();
        axisBL.updateEncoderPosition();
        axisBR.stop();
        axisBR.updateEncoderPosition();
        axisTR.stop();
        axisTR.updateEncoderPosition();
        axisTL.stop();
        axisTL.updateEncoderPosition();
    }
    else{  //Position the axis
        axisBL.recomputePID();
        axisBR.recomputePID();
        axisTR.recomputePID();
        axisTL.recomputePID();
    }

    int tlAngle = axisTL.readAngle();
    if(tlAngle == 0 || tlAngle == 16383){
        tlEncoderErrorCount++;
    }
    else{
        tlEncoderErrorCount = 0;
    }
    if(tlEncoderErrorCount > 10 && tlEncoderErrorCount < 100){
        //log_info( "Count %i\n", tlEncoderErrorCount);
        //log_info( "Encoder connection issue on TL\n");
    }
    

    int trAngle = axisTR.readAngle();
    if(trAngle == 0 || trAngle == 16383){
        trEncoderErrorCount++;
    }
    else{
        trEncoderErrorCount = 0;
    }
    if(trEncoderErrorCount > 10 && trEncoderErrorCount < 100){
        //log_info( "Encoder connection issue on TR\n");
    }

    int blAngle = axisBL.readAngle();
    if(blAngle == 0 || blAngle == 16383){
        blEncoderErrorCount++;
    }
    else{
        blEncoderErrorCount = 0;
    }
    if(blEncoderErrorCount > 10 && blEncoderErrorCount < 100){
        //log_info( "Encoder connection issue on BL\n");
    }

    int brAngle = axisBR.readAngle();
    if(brAngle == 0 || brAngle == 16383){
        brEncoderErrorCount++;
    }
    else{
        brEncoderErrorCount = 0;
    }
    if(brEncoderErrorCount > 10 && brEncoderErrorCount < 100){
        //log_info( "Encoder connection issue on BR\n");
    }

    int timeSinceLastCall = millis() - lastCallToPID;
    lastCallToPID = millis();
    if(timeSinceLastCall > 50){
        //log_info( "PID not being called often enough. Time since last call: %i\n", timeSinceLastCall);
    }

    // if(random(250) == 0){
    //     grbl_sendf( "Angles: TL %i, TR: %i, BL: %i, BR: %i\n", axisTL.readAngle(), axisTR.readAngle(), axisBL.readAngle(), axisBR.readAngle());
    // }
}

//Updates where the center x and y positions are
void updateCenterXY(){
    
    double A = (trY - blY)/(trX-blX);
    double B = (brY-tlY)/(brX-tlX);
    centerX = (brY-(B*brX)+(A*trX)-trY)/(A-B);
    centerY = A*(centerX - trX) + trY;
    
}

//Computes the tensions in the upper two belts
void computeTensions(float x, float y){
    //This should be a lot smarter and compute the vector tensions to see if the lower belts are contributing positively

    //These internal version move the center to (0,0)

    float tlXi = tlX - trX / 2.0;
    float tlYi = tlY / 2.0;
    float trXi = trX / 2.0;

    float A = atan((y-tlYi)/(trXi - x));
    float B = atan((y-tlYi)/(x-tlXi));

    trTension = 1 / (cos(A) * sin(B) / cos(B) + sin(A));
    tlTension = 1 / (cos(B) * sin(A) / cos(A) + sin(B));

    // if(random(400) == 10){
    //     grbl_sendf( "TL Tension: %f TR Tension: %f\n", tlTension, trTension);
    // }
}

//Bottom left belt
float computeBL(float x, float y, float z){
    //Move from lower left corner coordinates to centered coordinates
    x = x + centerX;
    y = y + centerY;
    float a = blX - x;
    float b = blY - y;
    float c = 0.0 - (z + blZ);

    float length = sqrt(a*a+b*b+c*c) - (beltEndExtension+armLength);

    //Add some extra slack if this belt isn't needed because the upper belt is already very taught
    //Max tension is around -1.81 at the very top and -.94 at the bottom
    float extraSlack = min(max(-34.48*trTension - 32.41, 0.0), 8.0); //limit of 0-2mm of extension

    // if(random(4000) == 10){
    //     grbl_sendf( "BL Slack By: %f\n", extraSlack);
    // }

    return length + extraSlack;
}

//Bottom right belt
float computeBR(float x, float y, float z){
    //Move from lower left corner coordinates to centered coordinates
    x = x + centerX;
    y = y + centerY;
    float a = brX - x;
    float b = brY - y;
    float c = 0.0 - (z + brZ);

    float length = sqrt(a*a+b*b+c*c) - (beltEndExtension+armLength);

    float extraSlack = min(max(-34.48*tlTension - 32.41, 0.0), 8.0); //limit of 0-2mm of extension

    // if(random(4000) == 10){
    //     grbl_sendf( "BR Slack By: %f\n", extraSlack);
    // }

    return length + extraSlack;
}

//Top right belt
float computeTR(float x, float y, float z){
    //Move from lower left corner coordinates to centered coordinates
    x = x + centerX;
    y = y + centerY;
    float a = trX - x;
    float b = trY - y;
    float c = 0.0 - (z + trZ);
    return sqrt(a*a+b*b+c*c) - (beltEndExtension+armLength);
}

//Top left belt
float computeTL(float x, float y, float z){
    //Move from lower left corner coordinates to centered coordinates
    x = x + centerX;
    y = y + centerY;
    float a = tlX - x;
    float b = tlY - y;
    float c = 0.0 - (z + tlZ);
    return sqrt(a*a+b*b+c*c) - (beltEndExtension+armLength);
}

void setTargets(float xTarget, float yTarget, float zTarget){
    
    if(!calibrationInProgress){

        computeTensions(xTarget, yTarget);

        axisBL.setTarget(computeBL(xTarget, yTarget, zTarget));
        axisBR.setTarget(computeBR(xTarget, yTarget, zTarget));
        axisTR.setTarget(computeTR(xTarget, yTarget, zTarget));
        axisTL.setTarget(computeTL(xTarget, yTarget, zTarget));
    }
}


//Runs the calibration sequence to determine the machine's dimensions
void runCalibration(){
    
    //log_info( "Beginning calibration\n");
    
    calibrationInProgress = true;
    
    //Undoes any calls by the system to move these to (0,0)
    axisBL.setTarget(axisBL.getPosition());
    axisBR.setTarget(axisBR.getPosition());
    axisTR.setTarget(axisTR.getPosition());
    axisTL.setTarget(axisTL.getPosition());
    
    float lengths1[4];
    float lengths2[4];
    float lengths3[4];
    float lengths4[4];
    float lengths5[4];
    float lengths6[4];
    float lengths7[4];
    float lengths8[4];
    float lengths9[4];
    
    //------------------------------------------------------Take measurements
    
    takeMeasurementAvgWithCheck(lengths1);
    
    moveWithSlack(-200, 0);
    
    takeMeasurementAvgWithCheck(lengths2);
    
    moveWithSlack(-200, -200);
    
    takeMeasurementAvgWithCheck(lengths3);

    lowerBeltsGoSlack();
    lowerBeltsGoSlack();
    moveWithSlack(0, 200);
    
    takeMeasurementAvgWithCheck(lengths4);
    
    moveWithSlack(0, 0);
    
    takeMeasurementAvgWithCheck(lengths5);
    
    moveWithSlack(0, -200);
    
    takeMeasurementAvgWithCheck(lengths6);
    
    lowerBeltsGoSlack();
    lowerBeltsGoSlack();
    moveWithSlack(200, 200);
    
    takeMeasurementAvgWithCheck(lengths7);
    
    moveWithSlack(200, 0);
    
    takeMeasurementAvgWithCheck(lengths8);
    
    moveWithSlack(200, -200);
    
    takeMeasurementAvgWithCheck(lengths9);
    
    lowerBeltsGoSlack();
    lowerBeltsGoSlack();
    moveWithSlack(0, 0);  //Go back to the center. This will pull the lower belts tight too
    
    axisBL.stop();
    axisBR.stop();
    axisTR.stop();
    axisTL.stop();
    
    //----------------------------------------------------------Do the computation
    
    printMeasurements(lengths1);
    printMeasurements(lengths2);
    printMeasurements(lengths3);
    printMeasurements(lengths4);
    printMeasurements(lengths5);
    printMeasurements(lengths6);
    printMeasurements(lengths7);
    printMeasurements(lengths8);
    printMeasurements(lengths9);
    
    double measurements[][4] = {
        //TL              TR           BL           BR
        {lengths1[3], lengths1[2], lengths1[0], lengths1[1]},
        {lengths2[3], lengths2[2], lengths2[0], lengths2[1]},
        {lengths3[3], lengths3[2], lengths3[0], lengths3[1]},
        {lengths4[3], lengths4[2], lengths4[0], lengths4[1]},
        {lengths5[3], lengths5[2], lengths5[0], lengths5[1]},
        {lengths6[3], lengths6[2], lengths6[0], lengths6[1]},
        {lengths7[3], lengths7[2], lengths7[0], lengths7[1]},
        {lengths8[3], lengths8[2], lengths8[0], lengths8[1]},
        {lengths9[3], lengths9[2], lengths9[0], lengths9[1]},
    };
    double results[6] = {0,0,0,0,0,0};
    computeCalibration(measurements, results, printToWeb, tlX, tlY, trX, trY, brX, tlZ, trZ, blZ, brZ);
    
    //log_info( "After computing calibration %f\n", results[5]);
    
    if(results[5] < 2){
        //log_info( "Calibration successful with precision %f \n\n", results[5]);
        tlX = results[0];
        tlY = results[1];
        trX = results[2];
        trY = results[3];
        blX = 0;
        blY = 0;
        brX = results[4];
        brY = 0;
        updateCenterXY();
        //log_info( "tlx: %f tly: %f trx: %f try: %f blx: %f bly: %f brx: %f bry: %f \n", tlX, tlY, trX, trY, blX, blY, brX, brY);
    }
    else{
        //log_info( "Calibration failed: %f\n", results[5]);
    }
    
    //---------------------------------------------------------Finish
    
    
    //Move back to center after the results are applied
    moveWithSlack(0, 0);
    
    //For safety we should pull tight here and verify that the results are basically what we expect before handing things over to the controller.
    takeMeasurementAvg(lengths1);
    takeMeasurementAvg(lengths1);
    
    double blError = (lengths1[0]-(beltEndExtension+armLength))-computeBL(0,0,0);
    double brError = (lengths1[1]-(beltEndExtension+armLength))-computeBR(0,0,0);
    
    //log_info( "Lower belt length mismatch: bl: %f, br: %f\n", blError, brError);
    
    calibrationInProgress = false;
    //log_info( "Calibration finished\n");
    
}

void printMeasurements(float lengths[]){
    //log_info( "{bl:%f,   br:%f,   tr:%f,   tl:%f},\n", lengths[0], lengths[1], lengths[2], lengths[3]);
}

void lowerBeltsGoSlack(){
    //log_info( "Lower belts going slack\n");
    
    unsigned long startTime = millis();

    axisBL.setTarget(axisBL.getPosition() + 2);
    axisBR.setTarget(axisBR.getPosition() + 2);
    
    while(millis()- startTime < 600){
        
        //The other axis hold position
        axisBL.recomputePID();
        axisBR.recomputePID();
        axisTR.recomputePID();
        axisTL.recomputePID();
        
        // Delay without blocking
        unsigned long time = millis();
        unsigned long elapsedTime = millis()-time;
        while(elapsedTime < 10){
            elapsedTime = millis()-time;
        }
    }

    //Then stop them before moving on
    startTime = millis();

    axisBL.setTarget(axisBL.getPosition());
    axisBR.setTarget(axisBR.getPosition());

    while(millis()- startTime < 600){
        
        axisBL.recomputePID();
        axisBR.recomputePID();
        axisTR.recomputePID();
        axisTL.recomputePID();

    }

    //grbl_sendf( "Going slack completed\n");
}

float printMeasurementMetrics(double avg, double m1, double m2, double m3, double m4, double m5){
    
    //grbl_sendf( "Avg: %f m1: %f, m2: %f, m3: %f, m4: %f, m5: %f\n", avg, m1, m2, m3, m4, m5);
    
    double m1Variation = myAbs(avg - m1);
    double m2Variation = myAbs(avg - m2);
    double m3Variation = myAbs(avg - m3);
    double m4Variation = myAbs(avg - m4);
    double m5Variation = myAbs(avg - m5);

    //grbl_sendf( "m1Variation: %f m2Variation: %f, m3Variation: %f, m4Variation: %f, m5Variation: %f\n", m1Variation, m2Variation, m3Variation, m4Variation, m5Variation);
    
    double maxDeviation = max(max(max(m1Variation, m2Variation), max(m3Variation, m4Variation)), m5Variation);
    
    //log_info( "Max deviation: %f\n", maxDeviation);

    //double avgDeviation = (m1Variation + m2Variation + m3Variation + m4Variation + m5Variation)/5.0;
    
    //grbl_sendf( "Avg deviation: %f\n", avgDeviation);

    return maxDeviation;
}

//Checks to make sure the deviation within the measurement avg looks good before moving on
void takeMeasurementAvgWithCheck(float lengths[]){
    //grbl_sendf( "Beginning takeMeasurementAvg\n");
    float threshold = 0.9;
    while(true){
        float repeatability = takeMeasurementAvg(lengths);
        if(repeatability < threshold){
            //log_info( "Using measurement with precision %f\n", repeatability);
            break;
        }
        //log_info( "Repeating measurement\n");
    }
}

//Takes 5 measurements and return how consistent they are
float takeMeasurementAvg(float avgLengths[]){
    //grbl_sendf( "Beginning to take averaged measurement.\n");
    
    //Where our five measurements will be stored
    float lengths1[4];
    float lengths2[4];
    float lengths3[4];
    float lengths4[4];
    float lengths5[4];
    
    takeMeasurement(lengths1);
    lowerBeltsGoSlack();
    takeMeasurement(lengths1);  //Repeat the first measurement to discard the one before everything was pulled taught
    lowerBeltsGoSlack();
    takeMeasurement(lengths2);
    lowerBeltsGoSlack();
    takeMeasurement(lengths3);
    lowerBeltsGoSlack();
    takeMeasurement(lengths4);
    lowerBeltsGoSlack();
    takeMeasurement(lengths5);
    
    avgLengths[0] = (lengths1[0]+lengths2[0]+lengths3[0]+lengths4[0]+lengths5[0])/5.0;
    avgLengths[1] = (lengths1[1]+lengths2[1]+lengths3[1]+lengths4[1]+lengths5[1])/5.0;
    avgLengths[2] = (lengths1[2]+lengths2[2]+lengths3[2]+lengths4[2]+lengths5[2])/5.0;
    avgLengths[3] = (lengths1[3]+lengths2[3]+lengths3[3]+lengths4[3]+lengths5[3])/5.0;
    
    float m1 = printMeasurementMetrics(avgLengths[0], lengths1[0], lengths2[0], lengths3[0], lengths4[0], lengths5[0]);
    float m2 = printMeasurementMetrics(avgLengths[1], lengths1[1], lengths2[1], lengths3[1], lengths4[1], lengths5[1]);
    float m3 = printMeasurementMetrics(avgLengths[2], lengths1[2], lengths2[2], lengths3[2], lengths4[2], lengths5[2]);
    float m4 = printMeasurementMetrics(avgLengths[3], lengths1[3], lengths2[3], lengths3[3], lengths4[3], lengths5[3]);

    float avgMaxDeviation = (m1+m2+m3+m4)/4.0;
    
    //log_info( "Average Max Deviation: %f\n", avgMaxDeviation);

    return avgMaxDeviation;
}

//Retract the lower belts until they pull tight and take a measurement
void takeMeasurement(float lengths[]){
    //grbl_sendf( "Taking a measurement.\n");

    axisBL.stop();
    axisBR.stop();

    bool axisBLDone = false;
    bool axisBRDone = false;

    float BLDist = .01;
    float BRDist = .01;
    
    while(!axisBLDone || !axisBRDone){  //As long as one axis is still pulling
        
        //If any of the current values are over the threshold then stop and exit, otherwise pull each axis a little bit tighter by incrementing the target position
        int currentThreshold = 11;
        
        if(axisBL.getCurrent() > currentThreshold || axisBLDone){  //Check if the current threshold is hit
            axisBLDone = true;
        }
        else{                                                       //If not
            axisBL.setTarget(axisBL.getPosition() - BLDist);                  //Pull in a little more
            BLDist = min(.2, BLDist + .01);                                     //Slowly ramp up the speed
        }
        
        if(axisBR.getCurrent() > currentThreshold || axisBRDone){
            axisBRDone = true;
        }
        else{
            axisBR.setTarget(axisBR.getPosition() - BRDist);
            BRDist = min(.2, BRDist + .01);
        }
        
        // Delay without blocking
        unsigned long time = millis();
        unsigned long elapsedTime = millis()-time;
        while(elapsedTime < 25){
            elapsedTime = millis()-time;
            recomputePID();  //This recomputes the PID four all four servos
        }
    }
    
    axisBL.setTarget(axisBL.getPosition());
    axisBR.setTarget(axisBR.getPosition());
    //axisTR.setTarget(axisTR.getPosition());
    //axisTL.setTarget(axisTL.getPosition());
    
    axisBL.stop();
    axisBR.stop();
    axisTR.stop();
    axisTL.stop();
    
    lengths[0] = axisBL.getPosition()+beltEndExtension+armLength;
    lengths[1] = axisBR.getPosition()+beltEndExtension+armLength;
    lengths[2] = axisTR.getPosition()+beltEndExtension+armLength;
    lengths[3] = axisTL.getPosition()+beltEndExtension+armLength;
    
    //log_info( "Measured:\n%f, %f \n%f %f \n",lengths[3], lengths[2], lengths[0], lengths[1]);
    
    return;
}

//Reposition the sled without knowing the machine dimensions
void moveWithSlack(float x, float y){
    
    //log_info( "Moving to (%f, %f) with slack lower belts\n", x, y);
    
    //The distance we need to move is the current position minus the target position
    double TLDist = axisTL.getPosition() - computeTL(x,y,0);
    double TRDist = axisTR.getPosition() - computeTR(x,y,0);
    
    //Record which direction to move
    double TLDir  = constrain(TLDist, -1, 1);
    double TRDir  = constrain(TRDist, -1, 1);
    
    double stepSize = .15;
    
    //Only use positive dist for incrementing counter (float int conversion issue?)
    TLDist = abs(TLDist);
    TRDist = abs(TRDist);
    
    //Make the lower arms compliant and move retract the other two until we get to the target distance
    
    unsigned long timeLastMoved1 = millis();
    unsigned long timeLastMoved2 = millis();
    double lastPosition1 = axisBL.getPosition();
    double lastPosition2 = axisBR.getPosition();
    double amtToMove1 = 1;
    double amtToMove2 = 1;
    
    while(TLDist > 0 || TRDist > 0){
        
        //Set the lower axis to be compliant. PID is recomputed in comply()
        axisBL.comply(&timeLastMoved1, &lastPosition1, &amtToMove1, 3);
        axisBR.comply(&timeLastMoved2, &lastPosition2, &amtToMove2, 3);
        
        // grbl_sendf( "BRPos: %f, BRamt: %f, BRtime: %l\n", lastPosition2, amtToMove2, timeLastMoved2);
        
        //Move the upper axis one step
        if(TLDist > 0){
            TLDist = TLDist - stepSize;
            axisTL.setTarget((axisTL.getTarget() - (stepSize*TLDir)));
        }
        if(TRDist > 0){
            TRDist = TRDist - stepSize;
            axisTR.setTarget((axisTR.getTarget() - (stepSize*TRDir)));
        }
        axisTR.recomputePID();
        axisTL.recomputePID();
        
        // Delay without blocking
        unsigned long time = millis();
        unsigned long elapsedTime = millis()-time;
        while(elapsedTime < 10){
            elapsedTime = millis()-time;
        }
    }
    
    //grbl_sendf( "Positional errors at the end of move <-%f, %f ->\n", axisTL.getError(), axisTR.getError());
    
    axisBL.setTarget(axisBL.getPosition());
    axisBR.setTarget(axisBR.getPosition());
    axisTR.setTarget(axisTR.getPosition());
    axisTL.setTarget(axisTL.getPosition());
    
    axisBL.stop();
    axisBR.stop();
    axisTR.stop();
    axisTL.stop();
    
    //Take up the internal slack to remove any slop between the spool and roller
    takeUpInternalSlack();
}

//This function removes any slack in the belt between the spool and the roller. 
//If there is slack there then when the motor turns the belt won't move which triggers the
//current threshold on pull tight too early. It only does this for the bottom axis.
void takeUpInternalSlack(){
    //Set the target to be .5mm in
    axisBL.setTarget(axisBL.getPosition() - 0.5);
    axisBR.setTarget(axisBR.getPosition() - 0.5);

    //Setup flags
    bool blDone = false;
    bool brDone = false;

    //Position hold until both axis are able to pull in until 
    while(!blDone && !brDone){

        //Check if they have pulled in fully
        if(axisBL.getPosition() < axisBL.getTarget()){
            blDone = true;
        }
        if(axisBR.getPosition() < axisBR.getTarget()){
            brDone = true;
        }

        recomputePID();

        // Delay without blocking
        unsigned long time = millis();
        unsigned long elapsedTime = millis()-time;
        while(elapsedTime < 10){
            elapsedTime = millis()-time;
        }
    }

    axisBL.stop();
    axisBR.stop();
    axisTR.stop();
    axisTL.stop();
}

float computeVertical(float firstUpper, float firstLower, float secondUpper, float secondLower){
    //Derivation at https://math.stackexchange.com/questions/4090346/solving-for-triangle-side-length-with-limited-information
    
    float b = secondUpper;   //upper, second
    float c = secondLower; //lower, second
    float d = firstUpper; //upper, first
    float e = firstLower;  //lower, first

    float aSquared = (((b*b)-(c*c))*((b*b)-(c*c))-((d*d)-(e*e))*((d*d)-(e*e)))/(2*(b*b+c*c-d*d-e*e));

    float a = sqrt(aSquared);
    
    return a;
}

void computeFrameDimensions(float lengthsSet1[], float lengthsSet2[], float machineDimensions[]){
    //Call compute verticals from each side
    
    float leftHeight = computeVertical(lengthsSet1[3],lengthsSet1[0], lengthsSet2[3], lengthsSet2[0]);
    float rightHeight = computeVertical(lengthsSet1[2],lengthsSet1[1], lengthsSet2[2], lengthsSet2[1]);
    
    //log_info( "Computed vertical measurements:\n%f \n%f \n%f \n",leftHeight, rightHeight, (leftHeight+rightHeight)/2.0);
}

#ifdef USE_CUSTOM_HOMING
/*
  user_defined_homing() is called at the beginning of the normal Grbl_ESP32 homing
  sequence.  If user_defined_homing() returns false, the rest of normal Grbl_ESP32
  homing is skipped if it returns false, other normal homing continues.  For
  example, if you need to manually prep the machine for homing, you could implement
  user_defined_homing() to wait for some button to be pressed, then return true.
*/
bool user_defined_homing(uint8_t cycle_mask)
{
  //log_info( "Extending\n");
  
  if(cycle_mask == 1){  //Top left
    axisTLHomed = axisTL.retract(computeTL(-200, 200, 0));
  }
  else if(cycle_mask == 2){  //Top right
    axisTR.testEncoder();
    axisTRHomed = axisTR.retract(computeTR(-200, 200, 0));
  }
  else if(cycle_mask == 4){ //Bottom right
    if(axisBLHomed && axisBRHomed && axisTRHomed && axisTLHomed){
        runCalibration();
    }
    else{
       axisBRHomed = axisBR.retract(computeBR(-200, 300, 0));
    }
  }
  else if(cycle_mask == 0){  //Bottom left
    axisBL.testEncoder();
    axisBLHomed = axisBL.retract(computeBL(-200, 300, 0));
  }
  
  if(axisBLHomed && axisBRHomed && axisTRHomed && axisTLHomed){
      //log_info( "All axis ready.\n");
  }
  
  return true;
}
#endif

#ifdef USE_KINEMATICS
/*
  Inverse Kinematics converts X,Y,Z Cartesian coordinate to the steps
  on your "joint" motors.  It requires the following three functions:
*/

/*
  inverse_kinematics() looks at incoming move commands and modifies
  them before Grbl_ESP32 puts them in the motion planner.

  Grbl_ESP32 processes arcs by converting them into tiny little line segments.
  Kinematics in Grbl_ESP32 works the same way. Search for this function across
  Grbl_ESP32 for examples. You are basically converting cartesian X,Y,Z... targets to

    target = an N_AXIS array of target positions (where the move is supposed to go)
    pl_data = planner data (see the definition of this type to see what it is)
    position = an N_AXIS array of where the machine is starting from for this move
*/
void inverse_kinematics(float *target, plan_line_data_t *pl_data, float *position)
{
  // this simply moves to the target. Replace with your kinematics.
  mc_line(target, pl_data);
}

/*
  kinematics_pre_homing() is called before normal homing
  You can use it to do special homing or just to set stuff up

  cycle_mask is a bit mask of the axes being homed this time.
*/
bool kinematics_pre_homing(uint8_t cycle_mask))
{
  return false; // finish normal homing cycle
}

/*
  kinematics_post_homing() is called at the end of normal homing
*/
void kinematics_post_homing()
{
}
#endif

#ifdef USE_FWD_KINEMATIC
/*
  The status command uses forward_kinematics() to convert
  your motor positions to Cartesian X,Y,Z... coordinates.

  Convert the N_AXIS array of motor positions to Cartesian in your code.
*/
void forward_kinematics(float *position)
{
  // position[X_AXIS] =
  // position[Y_AXIS] =
}
#endif

#ifdef USE_TOOL_CHANGE
/*
  user_tool_change() is called when tool change gcode is received,
  to perform appropriate actions for your machine.
*/
void user_tool_change(uint8_t new_tool)
{
}
#endif

#if defined(MACRO_BUTTON_0_PIN) || defined(MACRO_BUTTON_1_PIN) || defined(MACRO_BUTTON_2_PIN)
/*
  options.  user_defined_macro() is called with the button number to
  perform whatever actions you choose.
*/
void user_defined_macro(uint8_t index)
{
}
#endif

#ifdef USE_M30
/*
  user_m30() is called when an M30 gcode signals the end of a gcode file.
*/
void user_m30()
{
}
#endif

#ifdef USE_MACHINE_TRINAMIC_INIT
/*
  machine_triaminic_setup() replaces the normal setup of trinamic
  drivers with your own code.  For example, you could setup StallGuard
*/
void machine_trinamic_setup()
{
}
#endif

// If you add any additional functions specific to your machine that
// require calls from common code, guard their calls in the common code with
// #ifdef USE_WHATEVER and add function prototypes (also guarded) to grbl.h
