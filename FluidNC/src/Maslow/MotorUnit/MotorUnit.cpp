/***************************************************
 *  This is a library for control of a DC motor via a TLC Led driver with angle
 *  sensor feedback on an ESP32.
 *
 *  By Alexander Martin-Ginnold for Maslow CNC
 ****************************************************/
#include "MotorUnit.h"

#define TCAADDR 0x70

/*!
 *  @brief  Instantiates a new MotorUnit class. Instantiates classes for
 *          all other necessary controls.
 *  @param  forwardPin Output pin number for the motor controller. If this pin is at max
 *          output and the other pin is at 0 the motor turns forward
 *  @param  backwardPin Output pin number for the motor controller. If this pin is at
 *          max output and the other pin is at 0 the motor turns backward
 *  @param  readbackPin ESP32 adc_channel_t pin number for current readback
 *  @param  encoderNum The number of the encoder on the I2C multiplexer 
 *
 */
MotorUnit::MotorUnit(){
    _mmPerRevolution = 44;
}

//---------------------Functions related to testing the motor unit--------------------------------------------------


void MotorUnit::begin(int forwardPin,
               int backwardPin,
               int readbackPin,
               int encoderNum,
               int channel1,
               int channel2){

    _axisID = forwardPin;

    //_webPrint = webPrint;

    _forwardPin = forwardPin;
    _backwardPin = backwardPin;
    _readbackPin = readbackPin;
    _encoderNum = encoderNum;
    _channel1 = channel1;
    _channel2 = channel2;
    
    zero();
    motor.begin(_forwardPin, _backwardPin, _readbackPin, _channel1, _channel2);
    selectEncoder();
    encoder.begin();
    zero();

    positionPID.reset(new MiniPID(p,i,d));
    positionPID->setOutputLimits(-1023,1023);
}

//Used to set the multiplexer to read from the right encoder
void MotorUnit::selectEncoder() {
  uint8_t i = _encoderNum;
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

/*!
 *  @brief  Tests the function of the encoder
 */
bool MotorUnit::testEncoder(){
    log_info("Calling testEncoder from within MotorUnit!");
    motor.stop();
    
    // Test reading from angle sensor
    selectEncoder();
    if(encoder.isConnected()){
        Serial.print(encoder.readAngle());
        Serial.println(" -- encoder pass");
        return true;
    }
    else{
        Serial.println("encoder fail");
        return false;
    }
}

/*!
 *  @brief  Tests all the functions of this axis
 */
int MotorUnit::test(){
    
    bool testPass = true;
    
    testPass = testEncoder();
    
     // Test writing to motor
    // motor.fullOut();
    
    // delay(1000);
    
    // if(motor.readCurrent() > 0){
    //     Serial.print(".");
    //     Serial.print("Direction one nominal at ");
    //     Serial.print(motor.readCurrent());
    //     Serial.println("% current draw");
    // }
    // else{
    //     Serial.print("~");
    //     testPass = false;
    //     Serial.print("Direction one fail at ");
    //     Serial.print(motor.readCurrent());
    //     Serial.println("% current draw");
    // }
    
    
    // motor.stop();
    
    // delay(1000);
    
    // motor.fullIn();
    
    // delay(1000);
    
    // if(motor.readCurrent() > 0){
    //     Serial.print(".");
    //     Serial.print("Direction two nominal at ");
    //     Serial.print(motor.readCurrent());
    //     Serial.println(" ");
    // }
    // else{
    //     Serial.print("~");
    //     testPass = false;
    //     Serial.print("Direction two fail at ");
    //     Serial.print(motor.readCurrent());
    //     Serial.println(" ");
    // }
    
    motor.stop();
    
     // Test moving to a commanded position
    zero();
    Serial.println("Setpoint testing");
    Serial.println(setpoint);
    //setpoint = 25;
    Serial.println(setpoint);
    Serial.println("Trying to move to 25mm");
    setTarget(25);
    unsigned long time = millis();
    unsigned long elapsedTime = millis()-time;
    while(elapsedTime < 3000){
        elapsedTime = millis()-time;
        recomputePID();
    }
    
    if(abs(getError()) > 2){
        Serial.print("~");
        testPass = false;
        Serial.print("Failed to move to target position 25mm with error: ");
        Serial.println(getError());
    }
    else{
        Serial.print(".");
        Serial.print("Pass move to target position 25mm with error: ");
        Serial.println(getError());
    }
    
    Serial.println("Trying to move back -25mm");
    zero();
    setTarget(-25);
    time = millis();
    elapsedTime = millis()-time;
    while(elapsedTime < 3000){
        elapsedTime = millis()-time;
        recomputePID();
    }
    
    if(abs(getError()) > 2){
        Serial.print("~");
        testPass = false;
        Serial.print("Failed to move to target position -25mm with error: ");
        Serial.println(getError());
    }
    else{
        Serial.print(".");
        Serial.print("Pass move to target position -25mm with error: ");
        Serial.println(getError());
    }
    
    motor.stop();
    
    if(testPass){
        Serial.println("Pass");
        return true;
    }
    else{
        Serial.println("Fail");
        return false;
    }
    
}


//---------------------Functions related to the motor unit's position interface-------------------------------------


/*!
 *  @brief  Resets the axis position to zero
 */
void MotorUnit::zero(){
    selectEncoder();
    encoder.resetCumulativePosition();
}

/*!
 *  @brief  Read the angle directly from the sensor, useful for testing
 */
int MotorUnit::readAngle(){
    selectEncoder();
    return encoder.readAngle();
}

/*!
 *  @brief  Sets the target location
 */
void MotorUnit::setTarget(double newTarget){
    setpoint = newTarget;
}

/*!
 *  @brief  Gets the target location
 */
double MotorUnit::getTarget(){
    return setpoint;
}

/*!
 *  @brief  Sets the position of the cable
 */
int MotorUnit::setPosition(double newPosition){
    int angleTotal = (newPosition*4096)/_mmPerRevolution;
    selectEncoder();
    encoder.resetCumulativePosition(angleTotal);

    return true;
}

/*!
 *  @brief  Reads the current position of the axis
 */
double MotorUnit::getPosition(){
    selectEncoder();
    return (encoder.getCumulativePosition()/4096.0)*_mmPerRevolution*-1;
}

/*!
 *  @brief  Gets the current motor power draw
 */
double MotorUnit::getCurrent(){
    return motor.readCurrent();
}

/*!
 *  @brief  Computes and returns the error in the axis positioning
 */
double MotorUnit::getError(){
    
    double errorDist = setpoint - getPosition();
    
    return errorDist;
    
}

/*!
 *  @brief  Stops the motor
 */
void MotorUnit::stop(){
    motor.stop();
}

/*!
 *  @brief  Runs the motor out at full speed...where is this used? Why is it not symmetric with a fullIn() option?
 */
void MotorUnit::fullOut(){
    motor.fullOut();
}

//---------------------Functions related to maintaining the PID controllers-----------------------------------------



/*!
 *  @brief  Reads the encoder value and updates it's position and measures the velocity since the last call
 */
void MotorUnit::updateEncoderPosition(){
    selectEncoder();
    encoder.getCumulativePosition(); //This updates and returns the encoder value
}

/*!
 *  @brief  Recomputes the PID and drives the output
 */
double MotorUnit::recomputePID(){
    
    updateEncoderPosition();
    

    //Read the motor current and check for stalls
    double currentNow = getCurrent();
    if(currentNow > _stallCurrent){
        _stallCount = _stallCount + 1;
    }
    else{
        _stallCount = 0;
    }
    if(_stallCount > _stallThreshold){
        // if(_axisID == 1){    
        //     _webPrint(0xFF,"BR stalled at current: %f\n", currentNow);
        // }
        // else if(_axisID == 3){    
        //     _webPrint(0xFF,"TR stalled at current: %f\n", currentNow);
        // }
        // else if(_axisID == 7){    
        //     _webPrint(0xFF,"BL stalled at current: %f\n", currentNow);
        // }
        // else if(_axisID == 9){    
        //     _webPrint(0xFF,"TL stalled at current: %f\n", currentNow);
        // }
        // else{    
        //     _webPrint(0xFF,"%i stalled at current: %f\n",_axisID, currentNow);
        // }
        _stallCount = 0;
    }
    
    double commandPWM = positionPID->getOutput(getPosition(),setpoint);

    int currentMeasurement = motor.readCurrent();

    //Add some monitoring to the top right axis...this can crash the processor because it prints out so much data
    // if(_axisID == 3){
    //     _webPrint(0xFF,"TR PID: %f\n", commandPWM);
    // }

    if(abs(getPosition() - setpoint ) > 5){
        _numPosErrors = _numPosErrors + 1;

        // if(_numPosErrors > 2){
        //     if(_axisID == 1){    
        //         _webPrint(0xFF,"BR position error of %fmm ", getPosition() - setpoint);
        //         _webPrint(0xFF,"BR current draw %i ", currentMeasurement);
        //         _webPrint(0xFF,"BR PID output %f\n", commandPWM);
        //     }
        //     else if(_axisID == 3){
        //         _webPrint(0xFF,"TR position error of %fmm ", getPosition() - setpoint);
        //         _webPrint(0xFF,"TR current draw %i ", currentMeasurement);
        //         _webPrint(0xFF,"TR PID output %f\n", commandPWM);
        //     }
        //     else if(_axisID == 7){
        //         _webPrint(0xFF,"BL position error of %fmm ", getPosition() - setpoint);
        //         _webPrint(0xFF,"BL current draw %i ", currentMeasurement);
        //         _webPrint(0xFF,"BL PID output %f\n", commandPWM);
        //     }
        //     else if(_axisID == 9){
        //         _webPrint(0xFF,"TL position error of %fmm ", getPosition() - setpoint);
        //         _webPrint(0xFF,"TL current draw %i ", currentMeasurement);
        //         _webPrint(0xFF,"TL PID output %f\n", commandPWM);
        //     }
        //     else{
        //         _webPrint(0xFF,"%i position error of %fmm\n",_axisID, getPosition() - setpoint);
        //     }
        // }
    }
    else{
        _numPosErrors = 0;
    }

    if(commandPWM > 0){
        commandPWM = commandPWM + 7000;
    }

    if(commandPWM > 65530){
        commandPWM = 65530;
    }

    motor.runAtPWM(commandPWM);

    return commandPWM;
}

void MotorUnit::sendVoltage(double voltage){
    motor.runAtPWM(voltage);
}

/*!
 *  @brief  Runs the motor to extend for a little bit to put some slack into the coiled belt. Used to make it easier to extend.
 */
void MotorUnit::decompressBelt(){
    unsigned long time = millis();
    unsigned long elapsedTime = millis()-time;
    while(elapsedTime < 500){
        elapsedTime = millis()-time;
        motor.fullOut();
        updateEncoderPosition();
    }
}

/*!
 *  @brief  Sets the motor to comply with how it is being pulled
 */
bool MotorUnit::comply(unsigned long *timeLastMoved, double *lastPosition, double *amtToMove, double maxSpeed){
    
    //Update position and PID loop
    recomputePID();
    
    //If we've moved any, then drive the motor outwards to extend the belt
    float positionNow = getPosition();
    float distMoved = positionNow - *lastPosition;

    Serial.print("Dist moved: ");
    Serial.print(distMoved);
    Serial.print("  Target: ");
    Serial.println(getTarget());

    //If the belt is moving out, let's keep it moving out
    if( distMoved > .001){
        //Increment the target
        setTarget(positionNow + *amtToMove);
        
        *amtToMove = *amtToMove + 1;
        
        *amtToMove = min(*amtToMove, maxSpeed);
        
        //Reset the last moved counter
        *timeLastMoved = millis();
    
    //If the belt is moving in we need to stop it from moving in
    }else if(distMoved < -.04){
        *amtToMove = 0;
        setTarget(positionNow + .1);
        stop();
    }
    //Finally if the belt is not moving we want to spool things down
    else{
        *amtToMove = *amtToMove / 2;
        setTarget(positionNow);
        stop();
    }
    

    *lastPosition = positionNow;

    //Return indicates if we have moved within the timeout threshold
    if(millis()-*timeLastMoved > 5000){
        return false;
    }
    else{
        return true;
    }
}

/*!
 *  @brief  Fully retracts this axis and zeros it out or if it is already retracted extends it to the targetLength
 */
bool MotorUnit::retract(double targetLength){
    
    Serial.println("Retracting");
    log_info("Retracting called within MotorUnit!");
    int absoluteCurrentThreshold = 1600;
    int incrementalThreshold = 40;
    int incrementalThresholdHits = 0;
    float alpha = .2;
    float baseline = 700;

    uint16_t speed = 0;

    //Keep track of the elapsed time
    unsigned long time = millis();
    unsigned long elapsedTime = millis()-time;
    
    //Pull until taught
    while(true){
        
        //Gradually increase the pulling speed
        if(random(0,7) == 2){ //This is a hack to make it speed up more slowly
            speed = min(speed + 1, 1023);
        }
        motor.backward(speed);

        updateEncoderPosition();
        //When taught
        int currentMeasurement = motor.readCurrent();

        Serial.print("Current: ");
        Serial.print(currentMeasurement);
        Serial.print("  Baseline: ");
        Serial.print(baseline);
        Serial.print("  Difference: ");
        Serial.print(currentMeasurement - baseline);
        Serial.print("  Hits: ");
        Serial.println(incrementalThresholdHits);

        //_webPrint(0xFF,"Current: %i, Baseline: %f, difference: %f \n", currentMeasurement, baseline, currentMeasurement - baseline);
        baseline = alpha * float(currentMeasurement) + (1-alpha) * baseline;

        if(currentMeasurement - baseline > incrementalThreshold){
            Serial.println("Dynamic thershold hit");
            //_webPrint(0xFF,"Dynamic threshold hit\n");
            incrementalThresholdHits = incrementalThresholdHits + 1;
        }
        else{
            incrementalThresholdHits = 0;
        }

        if(currentMeasurement > absoluteCurrentThreshold || incrementalThresholdHits > 4){
            motor.stop();

            //Print how much the length of the belt changed compared to memory
            //_webPrint(0xFF,"Belt position after retract: %f\n", getPosition());
            Serial.print("Belt positon after retract: ");
            Serial.println(getPosition());

            zero();
            
            //If we hit the current limit immediately because there wasn't any slack we will extend
            elapsedTime = millis()-time;
            if(elapsedTime < 1500){

                Serial.println("Immediate hit detected");
                
                //Extend some belt to get things started
                decompressBelt();
                
                unsigned long timeLastMoved = millis();
                double lastPosition = getPosition();
                double amtToMove = 0.1;
                
                Serial.println("Got to the comply part");
                while(getPosition() < targetLength){
                    //Check for timeout
                    if(!comply(&timeLastMoved, &lastPosition, &amtToMove, 500)){//Comply updates the encoder position and does the actual moving
                        
                        //Stop and return
                        setTarget(getPosition());
                        motor.stop();
                        
                        return false;
                    }
                    
                    // Delay without blocking
                    unsigned long time = millis();
                    unsigned long elapsedTime = millis()-time;
                    while(elapsedTime < 50){
                        elapsedTime = millis()-time;
                    }
                }
                Serial.println("After the comply part");
                
                //Position hold for 2 seconds to make sure we are in the right place
                setTarget(targetLength);
                time = millis();
                elapsedTime = millis()-time;
                while(elapsedTime < 500){
                    elapsedTime = millis()-time;
                    recomputePID();
                }
                
                motor.stop();
                return true;
            }
            else{
                return false;
            }
        }
    }
}




