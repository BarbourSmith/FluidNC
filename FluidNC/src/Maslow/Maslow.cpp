
#include "Maslow.h"
#include "../Report.h"

// Maslow specific defines
#define TLEncoderLine 2
#define TREncoderLine 1
#define BLEncoderLine 3
#define BREncoderLine 0

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

#define coolingFanPin 47

#define SERVOFAULT 40

int ENCODER_READ_FREQUENCY_HZ = 100; //max frequency for pollingn the encoders

//------------------------------------------------------
//------------------------------------------------------ Main function loops
//------------------------------------------------------

// Initialization function 
void Maslow_::begin(void (*sys_rt)()) {
  Wire.begin(5,4, 200000);
  I2CMux.begin(TCAADDR, Wire);

  axisTL.begin(tlIn1Pin, tlIn2Pin, tlADCPin, TLEncoderLine, tlIn1Channel, tlIn2Channel);
  axisTR.begin(trIn1Pin, trIn2Pin, trADCPin, TREncoderLine, trIn1Channel, trIn2Channel);
  axisBL.begin(blIn1Pin, blIn2Pin, blADCPin, BLEncoderLine, blIn1Channel, blIn2Channel);
  axisBR.begin(brIn1Pin, brIn2Pin, brADCPin, BREncoderLine, brIn1Channel, brIn2Channel);
  
  axisBLHomed = false;
  axisBRHomed = false;
  axisTRHomed = false;
  axisTLHomed = false;

  tlX = 29.747433451550485;
  tlY = 2068.582533945923;
  tlZ = 116 + 38;
  trX = 2974.1176084487693; 
  trY = 2068.512981755607;
  trZ = 69 + 38;
  blX = 0;
  blY = 0;
  blZ = 47 + 38;
  brX = 2958.908589577277;
  brY = 0;
  brZ = 89 + 38;
//Roman frame, approx
//   tlX = 5.5;
//   tlY = 2150;
//   tlZ = 0;
//   trX = 3135; 
//   trY = 2150;
//   trZ = 0;
//   blX = 0;
//   blY = 0;
//   blZ = 0;
//   brX = 3095;
//   brY = 0;
//   brZ = 0;
//Roman frame, after calibration on 25 points
tlX = 54.2907358762372;
tlY = 2153.828537541466; 
trX = 3097.9245785193057;
trY = 2110.326736121985;
 blX = 0;
 blY =  0; 
 brX = 3088.4898198722663;
 blY =  0;

  
  //Recompute the center XY
  updateCenterXY();

  pinMode(coolingFanPin, OUTPUT);
  pinMode(SERVOFAULT, INPUT);

  currentThreshold = 1500;
  lastCallToUpdate = millis();
  generate_calibration_grid();
  log_info("Starting Maslow v 1.00");
}

// Maslow main loop, everything is processed here
void Maslow_::update(){
    //Make sure we're running maslow config file
    if(!Maslow.using_default_config){
        lastCallToUpdate = millis();

        Maslow.updateEncoderPositions(); //We always update encoder positions in any state,
        
        axisTL.update(); //update motor currents and belt speeds like this for now
        axisTR.update();
        axisBL.update();
        axisBR.update();

        if(safetyOn) safety_control();

        //quick solution for delay without blocking
        if(holding && millis() - holdTimer > holdTime){
            holding = false;
        }
        else if (holding) return;


        //Remove the slack from the belts after attaching to the frame
        if(takeSlack){
            if(take_measurement_avg_with_check(0,UP)){
                //print the measurements and expected measurements for center point:
                double off = _beltEndExtension+_armLength;
                log_info("Center point measurements : TL: " << calibration_data[0][0]-off << " TR: " << calibration_data[1][0]-off << " BL: " << calibration_data[2][0]-off << " BR: " << calibration_data[3][0]-off );
                
                log_info("Center point expected (0,0,0): TL: " << computeTL(0,0,0) << " TR: " << computeTR(0,0,0) << " BL: " << computeBL(0,0,0) << " BR: " << computeBR(0,0,0));
                //log deviation of the measurements from the expected values
                log_info("Center point deviation: TL: " << calibration_data[0][0]-off - computeTL(0,0,0) << " TR: " << calibration_data[1][0]-off - computeTR(0,0,0) << " BL: " << calibration_data[2][0]-off - computeBL(0,0,0) << " BR: " << calibration_data[3][0]-off - computeBR(0,0,0));
                takeSlack = false;
                log_info("Take up slack complete");
            }
        }

        //------------------------ Maslow State Machine

        //-------Jog or G-code execution. Maybe need to add more modes here like Hold? 
        //Jog doesn't work for lack of feedback, HOLD doesn't get automatically called after jog, so IDK what (TODO)
        if( sys.state() == State::Jog || sys.state() == State::Cycle  ){

            Maslow.setTargets(steps_to_mpos(get_axis_motor_steps(0),0), steps_to_mpos(get_axis_motor_steps(1),1), steps_to_mpos(get_axis_motor_steps(2),2));
            Maslow.recomputePID();
        }
        
        //--------Homing routines
        else if(sys.state() == State::Homing){
            home();
        }
        else { //In any other state, keep motors off  
            digitalWrite(coolingFanPin, HIGH);
            if(!test) Maslow.stopMotors();
 
        }

        //------------------------ End of Maslow State Machine

        //if the update function is not being called enough, stop everything to prevent damage
        if(millis() - lastCallToUpdate > 500){
            Maslow.panic();
            int elapsedTime = millis()-lastCallToUpdate; 
            log_error("Emergency stop. Update function not being called enough."  << elapsedTime << "ms since last call" );
        }

    }
    
}

// -Maslow homing loop
void Maslow_::home() {
  //run all the retract functions untill we hit the current limit
  if (retractingTL) {
      if (axisTL.retract()){
          retractingTL = false;
          axis_homed[0] = true;
      }
  }
  if (retractingTR) {
      if (axisTR.retract()){
          retractingTR = false;
          axis_homed[1] = true;
      }
  }
  if (retractingBL) {
      if (axisBL.retract()){
          retractingBL = false;
          axis_homed[2] = true;
      }
  }
  if (retractingBR) {
      if (axisBR.retract()){
          retractingBR = false;
          axis_homed[3] = true;
      }
  }

  // $EXT - extend mode
  if (extendingALL) {
      //decompress belts for the first half second
      if (millis() - extendCallTimer < 700) {
          if (millis() - extendCallTimer > 0)
              axisBR.decompressBelt();
          if (millis() - extendCallTimer > 150)
              axisBL.decompressBelt();
          if (millis() - extendCallTimer > 250)
              axisTR.decompressBelt();
          if (millis() - extendCallTimer > 350)
              axisTL.decompressBelt();
      }
      //then make all the belts comply until they are extended fully, or user terminates it
      else {
          if (!extendedTL)
              extendedTL = axisTL.extend(computeTL(0,0, 0));
          if (!extendedTR)
              extendedTR = axisTR.extend(computeTR(0,0, 0));
          if (!extendedBL)
              extendedBL = axisBL.extend(computeBL(0,300, 0));
          if (!extendedBR)
              extendedBR = axisBR.extend(computeBR(0,300, 0));
          if (extendedTL && extendedTR && extendedBL && extendedBR) {
              extendingALL = false;
              log_info("All belts extended to center position");
          }
      }
  }
  // $CMP - comply mode
  if (complyALL) {
      //decompress belts for the first half second
      if (millis() - complyCallTimer < 700) {
          if (millis() - complyCallTimer > 0)
              axisBR.decompressBelt();
          if (millis() - complyCallTimer > 150)
              axisBL.decompressBelt();
          if (millis() - complyCallTimer > 250)
              axisTR.decompressBelt();
          if (millis() - complyCallTimer > 350)
              axisTL.decompressBelt();
      } else {
          axisTL.comply();  
          axisTR.comply();
          axisBL.comply();
          axisBR.comply();
      }
  }
  
  // $CAL - calibration mode
  if(calibrationInProgress){
        calibration_loop();
  }

  //if we are done with all the homing moves, switch system state back to Idle?
  if (!retractingTL && !retractingBL && !retractingBR && !retractingTR && !extendingALL && !complyALL && !calibrationInProgress) {
      sys.set_state(State::Idle);
  }
}

// --Maslow calibration loop
void Maslow_::calibration_loop(){

     static int waypoint = 0; 
     static int direction = UP;
        static bool measurementInProgress = false;
        //Taking measurment once we've reached the point
        if(measurementInProgress){
            if(take_measurement_avg_with_check(waypoint, direction)){ //Takes a measurement and returns true if it's done
                
                measurementInProgress = false;
                waypoint++;                                 //Increment the waypoint counter

                if(waypoint > CALIBRATION_GRID_SIZE-1 ){ //If we have reached the end of the calibration process
                    calibrationInProgress = false;
                    waypoint = 0;
                    log_info("Calibration complete");
                    print_calibration_data();
                    sys.set_state(State::Idle);
                }
                else{ //Otherwise move to the next point
                    log_info("Moving from: " << calibrationGrid[waypoint-1][0] << " " << calibrationGrid[waypoint-1][1] << " to: " << calibrationGrid[waypoint][0] << " " << calibrationGrid[waypoint][1] << " direction: " << get_direction(calibrationGrid[waypoint-1][0], calibrationGrid[waypoint-1][1], calibrationGrid[waypoint][0], calibrationGrid[waypoint][1]));
                    hold(250);
                }
            }
        }

        //travel to the start point
        else if(waypoint == 0){
            //move to the start point
            
            if(move_with_slack(centerX, centerY, calibrationGrid[0][0], calibrationGrid[0][1])){
                measurementInProgress = true;
                direction = get_direction(centerX, centerY, calibrationGrid[0][0], calibrationGrid[0][1]);
                log_info("arrived at the start point");
                x = calibrationGrid[0][0];
                y = calibrationGrid[0][1];
                hold(250);
            }

        }

        //perform the calibrartion steps in the grid
        else{
            
            if(move_with_slack(calibrationGrid[waypoint-1][0], calibrationGrid[waypoint-1][1], calibrationGrid[waypoint][0], calibrationGrid[waypoint][1])){
                measurementInProgress = true;
                direction = get_direction(calibrationGrid[waypoint-1][0], calibrationGrid[waypoint-1][1], calibrationGrid[waypoint][0], calibrationGrid[waypoint][1]);
                x = calibrationGrid[waypoint][0];
                y = calibrationGrid[waypoint][1];
                hold(250);
            }

        }
}


//------------------------------------------------------
//------------------------------------------------------ Core utility functions
//------------------------------------------------------


//updating encoder positions for all 4 arms, cycling through them each call, at ENCODER_READ_FREQUENCY_HZ frequency
bool Maslow_::updateEncoderPositions(){
    bool success = true;
    static unsigned long lastCallToEncoderRead = millis();

    static int encoderFailCounter[4] = {0,0,0,0}; 
    static unsigned long encoderFailTimer = millis();

    if(!readingFromSD  && (millis() - lastCallToEncoderRead > 1000/(ENCODER_READ_FREQUENCY_HZ) ) ) {
        static int encoderToRead = 0;
        switch(encoderToRead){
            case 0:
                if(!axisTL.updateEncoderPosition()){
                    encoderFailCounter[0]++;
                }
                break;
            case 1:
                if(!axisTR.updateEncoderPosition()){
                    encoderFailCounter[1]++;
                }
                break;
            case 2:
                if(!axisBL.updateEncoderPosition()){
                    encoderFailCounter[2]++;
                }
                break;
            case 3:
                if(!axisBR.updateEncoderPosition()){
                    encoderFailCounter[3]++;
                }
                break;
        }
        encoderToRead++;
        if(encoderToRead > 3) {
            encoderToRead = 0;
            lastCallToEncoderRead = millis();
        }
    }

    // if more than 10% of readings fail, warn user, if more than 50% fail, stop the machine and raise alarm
    if(millis() - encoderFailTimer > 1000){
        for(int i = 0; i < 4; i++){
            //turn i into proper label
            String label = axis_id_to_label(i);
            if(encoderFailCounter[i] > 0.5*ENCODER_READ_FREQUENCY_HZ){
                // log error statement with appropriate label
                log_error("Failure on " << label.c_str() << " encoder, failed to read " << encoderFailCounter[i] << " times in the last second");
                Maslow.panic();
            }
            else if(encoderFailCounter[i] > 0.1*ENCODER_READ_FREQUENCY_HZ){
                log_info("Bad connection on " << label.c_str() << " encoder, failed to read " << encoderFailCounter[i] << " times in the last second");
            }
            encoderFailCounter[i] = 0;
            encoderFailTimer = millis();
        }
    }
    
    return success;
}

// This computes the target lengths of the belts based on the target x and y coordinates and sends that information to each arm.
void Maslow_::setTargets(float xTarget, float yTarget, float zTarget, bool tl, bool tr, bool bl, bool br){

        //Store the target x and y coordinates for the getTargetN() functions
        targetX = xTarget;
        targetY = yTarget;
        targetZ = zTarget;

        if(tl){
            axisTL.setTarget(computeTL(xTarget, yTarget, zTarget));
        }
        if(tr){
            axisTR.setTarget(computeTR(xTarget, yTarget, zTarget));
        }
        if(bl){
            axisBL.setTarget(computeBL(xTarget, yTarget, zTarget));
        }
        if(br){
            axisBR.setTarget(computeBR(xTarget, yTarget, zTarget));
        }
}

//updates motor powers for all axis, based on targets set by setTargets()
void Maslow_::recomputePID(){

    axisBL.recomputePID();
    axisBR.recomputePID();
    axisTR.recomputePID();
    axisTL.recomputePID();

    digitalWrite(coolingFanPin, HIGH);  //keep the cooling fan on

    if (digitalRead(SERVOFAULT) == 1) { //The servo drives have a fault pin that goes high when there is a fault (ie one over heats). We should probably call panic here. Also this should probably be read in the main loop
        log_info("Servo fault!");
    }
}

//This is the function that should prevent machine from damaging itself
void Maslow_::safety_control() {

  //We need to keep track of average belt speeds and motor currents for every axis
    static bool tick[4] = {false, false, false, false};
    static unsigned long spamTimer = millis();
    static int tresholdHitsBeforePanic = 10;
    static int panicCounter[4] = {0}; 
  
  MotorUnit* axis[4] = { &axisTL, &axisTR, &axisBL, &axisBR };
  for (int i = 0; i < 4; i++) {
      //If the current exceeds some absolute value, we need to call panic() and stop the machine
      if (axis[i]->getMotorCurrent() > currentThreshold+2500  && !tick[i]) {
        panicCounter[i]++;
        if(panicCounter[i] > tresholdHitsBeforePanic){
          log_error("Motor current on " << axis_id_to_label(i).c_str() << " axis exceeded threshold of " << currentThreshold+2500
                                        << "mA, current is " << int(axis[i]->getMotorCurrent()) << "mA");
          Maslow.panic();
          tick[i] = true;
        }
      }
      else {
        panicCounter[i] = 0;
      }

      //If the motor torque is high, but the belt is not moving
      //  if motor is moving IN, this means the axis is STALL, we should warn the user and lower torque to the motor
      //  if the motor is moving OUT, that means the axis has SLACK, so we should warn the user and stop the motor, until the belt starts moving again
      // don't spam log, no more than once every 5 seconds
      
      static int axisSlackCounter[4] = {0,0,0,0};


      if(axis[i]->getMotorPower() > 450 && abs (axis[i]->getBeltSpeed() ) < 0.1 && !tick[i]){
            axisSlackCounter[i]++;
            if(axisSlackCounter[i] > 200){
                log_info("SLACK:" << axis_id_to_label(i).c_str() << " motor power is " << int(axis[i]->getMotorPower()) << ", but the belt is not moving");
                log_info("Pull on " << axis_id_to_label(i).c_str() << " and restart!");
                tick[i] = true;
                axisSlackCounter[i] = 0;
                Maslow.panic();
            }
      }
        else axisSlackCounter[i] = 0;
      
}

if(millis() - spamTimer > 5000){
    for(int i = 0; i < 4; i++){
        tick[i] = false;
    }
    spamTimer = millis();
}
}

// Compute target belt lengths based on X-Y-Z coordinates
float Maslow_::computeBL(float x, float y, float z){
    //Move from lower left corner coordinates to centered coordinates
    x = x + centerX;
    y = y + centerY;
    float a = blX - x;
    float b = blY - y;
    float c = 0.0 - (z + blZ);

    float length = sqrt(a*a+b*b+c*c) - (_beltEndExtension+_armLength);

    return length ; //+ lowerBeltsExtra;
}
float Maslow_::computeBR(float x, float y, float z){
    //Move from lower left corner coordinates to centered coordinates
    x = x + centerX;
    y = y + centerY;
    float a = brX - x;
    float b = brY - y;
    float c = 0.0 - (z + brZ);

    float length = sqrt(a*a+b*b+c*c) - (_beltEndExtension+_armLength);

    return length ; //+ lowerBeltsExtra;
}
float Maslow_::computeTR(float x, float y, float z){
    //Move from lower left corner coordinates to centered coordinates
    x = x + centerX;
    y = y + centerY;
    float a = trX - x;
    float b = trY - y;
    float c = 0.0 - (z + trZ);
    return sqrt(a*a+b*b+c*c) - (_beltEndExtension+_armLength);
}
float Maslow_::computeTL(float x, float y, float z){
    //Move from lower left corner coordinates to centered coordinates
    x = x + centerX;
    y = y + centerY;
    float a = tlX - x;
    float b = tlY - y;
    float c = 0.0 - (z + tlZ);
    return sqrt(a*a+b*b+c*c) - (_beltEndExtension+_armLength);
}


//------------------------------------------------------
//------------------------------------------------------ Homing and calibration functions
//------------------------------------------------------

// Takes one measurement; returns true when it's done
bool Maslow_::take_measurement(int waypoint, int dir, int run){

 if (orientation == VERTICAL) {
      //first we pull two bottom belts tight one after another, if x<0 we pull left belt first, if x>0 we pull right belt first
      static bool BL_tight = false;
      static bool BR_tight = false;
      axisTL.recomputePID();
      axisTR.recomputePID();  

      //On the left side of the sheet we want to pull the left belt tight first
      if (x < 0) {
          if (!BL_tight) {
              if (axisBL.pull_tight()) {
                  BL_tight = true;
                  //log_info("Pulled BL tight");
              }
              return false;
          }
          if (!BR_tight) {
              if (axisBR.pull_tight()) {
                  BR_tight = true;
                    //log_info("Pulled BR tight");
              }
              return false;
          }
      } 
      
      //On the right side of the sheet we want to pull the right belt tight first
      else {
          if (!BR_tight) {
              if (axisBR.pull_tight()) {
                  BR_tight = true;
                    //log_info("Pulled BR tight");
              }
              return false;
          }
          if (!BL_tight) {

              if (axisBL.pull_tight()) {
                  BL_tight = true;
                    //log_info("Pulled BL tight");
              }
              return false;
          }
      }

      //once both belts are pulled, take a measurement
      if (BR_tight && BL_tight) {
          //take measurement and record it to the calibration data array
          calibration_data[0][waypoint] = axisTL.getPosition()+_beltEndExtension+_armLength;
          calibration_data[1][waypoint] = axisTR.getPosition()+_beltEndExtension+_armLength;
          calibration_data[2][waypoint] = axisBL.getPosition()+_beltEndExtension+_armLength;
          calibration_data[3][waypoint] = axisBR.getPosition()+_beltEndExtension+_armLength;
          BR_tight = false;
          BL_tight = false;
          return true;
      }
      return false;
  }
  // in HoRIZONTAL orientation we pull on the belts depending on the direction of the last move 
  else if (orientation == HORIZONTAL) {
    static MotorUnit* pullAxis1;
    static MotorUnit* pullAxis2;
    static MotorUnit* holdAxis1;
    static MotorUnit* holdAxis2;
    static bool pull1_tight = false;
    static bool pull2_tight = false;
    switch (dir) {
      case UP:
        holdAxis1 = &axisTL;
        holdAxis2 = &axisTR;
        if(x < 0){
            pullAxis1 = &axisBL;
            pullAxis2 = &axisBR;
        }
        else{
            pullAxis1 = &axisBR;
            pullAxis2 = &axisBL;
        }
        break;
        case DOWN:
        holdAxis1 = &axisBL;
        holdAxis2 = &axisBR;
        if(x < 0){
            pullAxis1 = &axisTL;
            pullAxis2 = &axisTR;
        }
        else{
            pullAxis1 = &axisTR;
            pullAxis2 = &axisTL;
        }
        break;
        case LEFT:
        holdAxis1 = &axisTL;
        holdAxis2 = &axisBL;
        if(y < 0){
            pullAxis1 = &axisBR;
            pullAxis2 = &axisTR;
        }
        else{
            pullAxis1 = &axisTR;
            pullAxis2 = &axisBR;
        }
        break;
        case RIGHT:
        holdAxis1 = &axisTR;
        holdAxis2 = &axisBR;
        if(y < 0){
            pullAxis1 = &axisBL;
            pullAxis2 = &axisTL;
        }
        else{
            pullAxis1 = &axisTL;
            pullAxis2 = &axisBL;
        }
        break;
    }
    holdAxis1->recomputePID();
    holdAxis2->recomputePID();
    if (!pull1_tight) {
        if (pullAxis1->pull_tight()) {
            pull1_tight = true;
            String axisLabel = "";
            if(pullAxis1 == &axisTL) axisLabel = "TL";
            if(pullAxis1 == &axisTR) axisLabel = "TR";
            if(pullAxis1 == &axisBL) axisLabel = "BL";
            if(pullAxis1 == &axisBR) axisLabel = "BR";
            //log_info("Pulled 1 tight on " << axisLabel.c_str());
        
        }
        if(run == 0) pullAxis2->comply();
        return false;
    }
    if (!pull2_tight) {
        if (pullAxis2->pull_tight()) {
            pull2_tight = true;
            String axisLabel = "";
            if(pullAxis2 == &axisTL) axisLabel = "TL";
            if(pullAxis2 == &axisTR) axisLabel = "TR";
            if(pullAxis2 == &axisBL) axisLabel = "BL";
            if(pullAxis2 == &axisBR) axisLabel = "BR";
            //log_info("Pulled 2 tight on " << axisLabel.c_str());
        }
        return false;
    }
    if (pull1_tight && pull2_tight) {
        //take measurement and record it to the calibration data array
        calibration_data[0][waypoint] = axisTL.getPosition()+_beltEndExtension+_armLength;
        calibration_data[1][waypoint] = axisTR.getPosition()+_beltEndExtension+_armLength;
        calibration_data[2][waypoint] = axisBL.getPosition()+_beltEndExtension+_armLength;
        calibration_data[3][waypoint] = axisBR.getPosition()+_beltEndExtension+_armLength;
        pull1_tight = false;
        pull2_tight = false;
        return true;
    }

  }

  return false;
}

// Takes a series of measurements, calculates average and records calibration data;  returns true when it's done
bool Maslow_::take_measurement_avg_with_check(int waypoint, int dir) {
  //take 5 measurements in a row, (ignoring the first one), if they are all within 1mm of each other, take the average and record it to the calibration data array
  static int           run                = 0;
  static double        measurements[4][4] = { 0 };
  static double        avg                = 0;
  static double        sum                = 0;
  static unsigned long decompressTimer    = millis();



  if (take_measurement(waypoint,dir,run)) {
      if (run < 3) {
          //decompress lower belts for 500 ms before taking the next measurement
          decompressTimer = millis();
          run++;
          return false;  //discard the first three measurements
      }

      measurements[0][run - 3] = calibration_data[0][waypoint];  //-3 cuz discarding the first 3 measurements
      measurements[1][run - 3] = calibration_data[1][waypoint];
      measurements[2][run - 3] = calibration_data[2][waypoint];
      measurements[3][run - 3] = calibration_data[3][waypoint];

      run++;

      static int criticalCounter = 0;
      if (run > 6) {
          run = 0;

          //check if all measurements are within 1mm of each other
          double maxDeviation[4] = { 0 };
          double maxDeviationAbs = 0;
          for (int i = 0; i < 4; i++) {
              for (int j = 0; j < 3; j++) {
                    //find max deviation between measurements
                    maxDeviation[i] = max(maxDeviation[i], abs(measurements[i][j] - measurements[i][j + 1]));
              }
          }
          //log max deviations at every axis:
          //log_info("Max deviation at BL: " << maxDeviation[2] << " BR: " << maxDeviation[3] << " TR: " << maxDeviation[1] << " TL: " << maxDeviation[0]);
          //find max deviation between all measurements

          for (int i = 0; i < 4; i++) {
              maxDeviationAbs = max(maxDeviationAbs, maxDeviation[i]);
          }
          if (maxDeviationAbs > 2.5) {
              log_error("Measurement error, measurements are not within 2.5 mm of each other, trying again");
              log_info("Max deviation: " << maxDeviationAbs);
              
              //print all the measurements in readable form:
              for (int i = 0; i < 4; i++) {
                    for (int j = 0; j < 4; j++) {
                        //use axis id to label:
                        log_info(axis_id_to_label(i).c_str() << " " << measurements[i][j]);
                    }
              }
              //reset the run counter to run the measurements again
              if (criticalCounter++ > 8) {
                    log_error("Critical error, measurements are not within 1.5mm of each other 8 times in a row, stopping calibration");
                    calibrationInProgress = false;
                    waypoint              = 0;
                    criticalCounter       = 0;
                    return false;
              }

              decompressTimer = millis();
              return false;
          }
          //if they are, take the average and record it to the calibration data array
          for (int i = 0; i < 4; i++) {
              for (int j = 0; j < 4; j++) {
                    sum += measurements[i][j];
              }
              avg                           = sum / 4;
              calibration_data[i][waypoint] = avg;
              sum                           = 0;
              criticalCounter               = 0;
          }
          //log_info("Took measurement at waypoint " << waypoint);
           log_info("{bl:" << calibration_data[2][waypoint] << ",   br:" << calibration_data[3][waypoint] << ",   tr:" << calibration_data[1][waypoint] << ",   tl:" << calibration_data[0][waypoint] << "},");
          return true;
      }

      //decompress lower belts for 500 ms before taking the next measurement
      decompressTimer = millis();
  }

  return false;
}

// Move pulling just two belts depending on the direction of the movement
bool Maslow_::move_with_slack(double fromX, double fromY, double toX, double toY) {
  
  //This is where we want to introduce some slack so the system
  static unsigned long decompressTimer    = millis();
  static bool          decompress         = true;

int direction  = get_direction(fromX, fromY, toX, toY);

//We conly want to decompress at the beginning of each move
if(decompress){
    decompressTimer = millis();
    //log_info("decompressing at " << int(millis()));
    decompress = false;
}

//Decompress belts for 500ms...this happens by returning right away before running any of the rest of the code
  if (millis() - decompressTimer < 750) {
    if( orientation == VERTICAL){
                axisTL.recomputePID();
                axisTR.recomputePID();
                axisBL.decompressBelt();
                axisBR.decompressBelt();
            }
            else{
                axisTL.decompressBelt();
                axisTR.decompressBelt();
                axisBL.decompressBelt();
                axisBR.decompressBelt();
            }
                
      return false;
  }

  //Stop for 50ms
  //we need to stop motors after decompression was finished once
  else if (millis() - decompressTimer < 1000) {
      stopMotors();
      return false;
  }

//   //How big of steps do we want to take with each loop?
//   double stepSize = 0.01;

//   double currentXTarget = getTargetX();
//   double currentYTarget = getTargetY();


//   int xDirection = currentXTarget - toX > 0 ? -1 : 1;
//   int yDirection = currentYTarget - toY > 0 ? -1 : 1;
//   if(abs(currentXTarget - toX) <= stepSize ) xDirection = 0; 
//   if(abs(currentYTarget - toY) <= stepSize ) yDirection = 0; 

  switch (direction) {
    case UP:
        setTargets(toX,toY, 0, true, true, false, false);
        axisTL.recomputePID(500);
        axisTR.recomputePID(500);
        axisBL.comply();
        axisBR.comply();
        //if( onTarget(toX, toY, getTargetX(), getTargetY(), 0.25) ) { // this doesn't check if we actually got to the position with belts
        if(axisTL.onTarget(0.25) && axisTR.onTarget(0.25)){
            stopMotors();
            reset_all_axis();
            decompress = true; //Reset for the next pass
            return true;
        }
        //}
        break;
      case DOWN:
            setTargets(toX,toY, 0, false, false, true, true);
            axisTL.comply();
            axisTR.comply();
            axisBL.recomputePID(500);
            axisBR.recomputePID(500);
            //if( onTarget(toX, toY, getTargetX(), getTargetY(), 0.25) )  {
            if(axisBL.onTarget(0.25) && axisBR.onTarget(0.25)){
                stopMotors();
                reset_all_axis();
                decompress = true; //Reset for the next pass
                return true;
            }
            //}
            break;
      case LEFT:
            setTargets(toX,toY, 0, true, false, true, false);
            axisTL.recomputePID(500);
            axisTR.comply();
            axisBL.recomputePID(500);
            axisBR.comply();
            //if( onTarget(toX, toY, getTargetX(), getTargetY(), 0.25) ) {
                if(axisTL.onTarget(0.25) && axisBL.onTarget(0.25)){
                stopMotors();
                reset_all_axis();
                decompress = true; //Reset for the next pass
                return true;
                }
            //}
            break;
      case RIGHT:
            setTargets(toX,toY, 0, false, true, false, true);
            axisTL.comply();
            axisTR.recomputePID(500);
            axisBL.comply();
            axisBR.recomputePID(500);
            //if( onTarget(toX, toY, getTargetX(), getTargetY(), 0.25) ) {
                if(axisBR.onTarget(0.25) && axisTR.onTarget(0.25)){
                stopMotors();
                reset_all_axis();
                decompress = true; //Reset for the next pass
                return true;       
                }         
            //}
            break;
  }
  return false;
}

// Direction from maslow current coordinates to the target coordinates
int Maslow_::get_direction(double x, double y, double targetX, double targetY){
    
    int direction = 0;

    if(orientation == VERTICAL) return UP;

    if( targetX-x > 1) {
        direction = RIGHT;
    }
    else if( targetX-x < -1) {
        direction = LEFT;
    }
    else if( targetY-y > 1) {
        direction = UP;
    }
    else if( targetY-y < -1) {
        direction = DOWN;
    }

    return direction;
} 

// Generate calibration points based on dimentions (TODO)
void Maslow_::generate_calibration_grid() {

    int gridSizeX = int(sqrt(CALIBRATION_GRID_SIZE));
    int gridSizeY = int(sqrt(CALIBRATION_GRID_SIZE));
    int xSpacing =  (frame_width - 2*calibration_grid_offset) / gridSizeX;
    int ySpacing =  (frame_height - 2*calibration_grid_offset) / gridSizeY;
    int pointCount = 0;

  for(int i = -gridSizeX/2; i <= gridSizeX/2; i++) {
    if(i % 2 == 0) {
      for(int j = -gridSizeY/2; j <= gridSizeY/2; j++) {
        log_info("Point: " << pointCount << "(" << i * xSpacing << ", " << j * ySpacing << ")");
        calibrationGrid[pointCount][0] = i * xSpacing;
        calibrationGrid[pointCount][1] = j * ySpacing;
        pointCount++;
      }
    } else {
      for(int j = gridSizeY/2; j >= -gridSizeY/2; j--) {
        log_info("Point: " << pointCount << "(" << i * xSpacing << ", " << j * ySpacing << ")"); 
        calibrationGrid[pointCount][0] = i * xSpacing;
        calibrationGrid[pointCount][1] = j * ySpacing;
        pointCount++;
      }
    }
  }

}


//------------------------------------------------------
//------------------------------------------------------ User commands
//------------------------------------------------------


void Maslow_::retractTL(){
    //We allow other bells retracting to continue 
    retractingTL = true;
    complyALL = false;
    extendingALL = false;
    axisTL.reset();
    log_info("Retracting Top Left");
}
void Maslow_::retractTR(){
    retractingTR = true;
    complyALL = false;
    extendingALL = false;
    axisTR.reset();
    log_info("Retracting Top Right");
}
void Maslow_::retractBL(){
    retractingBL = true;
    complyALL = false;
    extendingALL = false;
    axisBL.reset();
    log_info("Retracting Bottom Left");
}
void Maslow_::retractBR(){
    retractingBR = true;
    complyALL = false;
    extendingALL = false;
    axisBR.reset();
    log_info("Retracting Bottom Right");
}
void Maslow_::retractALL(){
    retractingTL = true;
    retractingTR = true;
    retractingBL = true;
    retractingBR = true;
    complyALL = false;
    extendingALL = false;
    axisTL.reset();
    axisTR.reset();
    axisBL.reset();
    axisBR.reset();
    log_info("Retracting All");
}
void Maslow_::extendALL(){
    // ADD also shouldn't extend before we get the parameters from the user

    if(!all_axis_homed()){
        log_error("Cannot extend all until all axis are homed"); //I keep getting everything set up for calibration and then this trips me up
        sys.set_state(State::Idle);
        return;
    }

    stop();
    extendingALL = true;
    extendCallTimer = millis();
    log_info("Extending All");
}

void Maslow_::pullBeltsTight(){
    x = 0;
    y = 0;
    takeSlack = true;
}

void Maslow_::runCalibration(){

    stop();
    //if not all axis are homed, we can't run calibration, OR if the user hasnt entered width and height? 
    if(!all_axis_homed()){
        log_error("Cannot run calibration until all axis are homed");
        sys.set_state(State::Idle);
        return;
    }

    if(frame_width < frame_dimention_MIN || frame_width > frame_dimention_MAX || frame_height < frame_dimention_MIN || frame_height > frame_dimention_MAX){
        log_error("Cannot run calibration until frame width and height are set");
        sys.set_state(State::Idle);
        return;
    }
    sys.set_state(State::Homing);
    //generate calibration map 
    generate_calibration_grid();
    calibrationInProgress = true;
}
void Maslow_::comply(){
    complyCallTimer = millis();
    retractingTL = false;
    retractingTR = false;
    retractingBL = false;
    retractingBR = false;
    extendingALL = false;
    complyALL = true;
    axisTL.reset();
    axisTR.reset();
    axisBL.reset();
    axisBR.reset();
    log_info("Complying All");
}
void Maslow_::setSafety(bool state){
    safetyOn = state;
}
void Maslow_::test_(){
    x = 0;
    y = 0;
    test = true;
}
void Maslow_::set_frame_width(double width){
    frame_width = width;
    update_frame_xyz();
    updateCenterXY();
}
void Maslow_::set_frame_height(double height){
    frame_height = height;
    update_frame_xyz();
    updateCenterXY();
}


//------------------------------------------------------
//------------------------------------------------------ Utility functions
//------------------------------------------------------


//non-blocking delay, just pauses everything for specified time 
void Maslow_::hold(unsigned long time){
    holdTime = time;
    holding = true;
    holdTimer = millis();
}

// Resets variables on all 4 axis
void Maslow_::reset_all_axis(){
    axisTL.reset();
    axisTR.reset();
    axisBL.reset();
    axisBR.reset();
}

// True if all axis were zeroed
bool Maslow_::all_axis_homed(){
    return axis_homed[0] && axis_homed[1] && axis_homed[2] && axis_homed[3];
}

// int to string name conversion for axis labels
String Maslow_::axis_id_to_label(int axis_id){
    String label;
    switch(axis_id){
        case 0:
            label = "Top Left";
            break;
        case 1:
            label = "Top Right";
            break;
        case 2:
            label = "Bottom Left";
            break;
        case 3:
            label = "Bottom Right";
            break;
    }
    return label;
}

// function for outputting calibration data in the log line by line like this: {bl:2376.69,   br:923.40,   tr:1733.87,   tl:2801.87},
void Maslow_::print_calibration_data(){
    for(int i = 0; i < CALIBRATION_GRID_SIZE; i++){
        log_info("{bl:" << calibration_data[2][i] << ",   br:" << calibration_data[3][i] << ",   tr:" << calibration_data[1][i] << ",   tl:" << calibration_data[0][i] << "},");
    }
}

// Stop all motors and reset all state variables
void Maslow_::stop(){
    stopMotors();
    retractingTL = false;
    retractingTR = false;
    retractingBL = false;
    retractingBR = false;
    extendingALL = false;
    complyALL = false;
    calibrationInProgress = false;
    takeSlack = false; 
    axisTL.reset();
    axisTR.reset();
    axisBL.reset();
    axisBR.reset();
}

// Stop all the motors
void Maslow_::stopMotors(){
    axisBL.stop();
    axisBR.stop();
    axisTR.stop();
    axisTL.stop();
    //digitalWrite(coolingFanPin, LOW); //Turn off the cooling fan
}

// Panic function, stops all motors and sets state to alarm
void Maslow_::panic(){
    log_error("PANIC! Stopping all motors");
    stop();
    sys.set_state(State::Alarm);
}

// update coordinates of the corners based on the frame width and height, should be called every time frame width or height is changed (TODO)
void Maslow_::update_frame_xyz(){
    blX = 0;
    blY = 0;
    blZ = 0;

    brY = 0;
    brX = frame_width;
    brZ = 0;

    tlX = 0;
    tlY = frame_height;
    tlZ = 0;

    trX = frame_width;
    trY = frame_height;
    trZ = 0;

}

// Get's the most recently set target position in X
double Maslow_::getTargetX(){
    return targetX;
}

// Get's the most recently set target position in Y
double Maslow_::getTargetY(){
    return targetY;
}

//Get's the most recently set target position in Z
double Maslow_::getTargetZ(){
    return targetZ;
}

//Updates where the center x and y positions are
void Maslow_::updateCenterXY(){
    
    double A = (trY - blY)/(trX-blX);
    double B = (brY-tlY)/(brX-tlX);
    centerX = (brY-(B*brX)+(A*trX)-trY)/(A-B);
    centerY = A*(centerX - trX) + trY;
    
}

Maslow_ &Maslow_::getInstance() {
  static Maslow_ instance;
  return instance;
}


Maslow_ &Maslow = Maslow.getInstance();



