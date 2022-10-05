#pragma once
#include "auton_functions.cpp"
#include <vector>
#include <bits/stdc++.h>
#define DEFAULT_STANDSTILL NAN
#define STANDSTILL_OFF UINT16_MAX
#define DISTANCE_STANDSTILL_THRESH 2
#define TURN_STANDSTILL_THRESH 0.02

#define STANDSTILL_TOLERANCE 10
#define STANDSTILL_TOLERANCE_T 10

#define YELLOW_SIG_INDEX 1
#define VISION_CLOSEST 0


//Direction enumeration
enum PID_dir{
  forward,
  backward,
  left,
  right,
  shortest,
  forwardRight,
  forwardLeft,
  forwardShortest,
  backwardRight,
  backwardLeft,
  backwardShortest
};


class Drive{
private:
  //Initialize PID Values
  const float pidConstants[7][8] = {
  /*{kP,kPt,kI,kIt,kD,kDt,kPd,kPs}*/
    {12,123,10,145,15,150,100,0}, //General PID
    {25,225,15,25,70,120,100,0}, //Turns under 45 degrees and short linear movements
    {12,100,10,400,15,150,100,0}, //Mogos loaded
    {50,123,10,145,15,150,100,0}, //Swerve
    {12,30,10,20,15,30,0,0}, //Vision Turns and Swerve
    {50,140,10,145,15,150,100,0}, //Swerve that we use once lol
    {14,225,10,25,15,120,100,0} //Swerve to stay on straight line
  };

  float kP;
  float kP_t;
  float kI;
  float kI_t;
  float kD;
  float kD_t;
  float kP_d;
  float kP_s;

  float slew;
  float slewMin;
  float error;
  float maxVolt;
  float maxVolt_a;
  float standStillInput;
  float motorEfficiency;
  bool SSActive;

  bool isNewPID = false;

  float headingValue();
  float rotationValue();

public:
  //Functions run when Drive is initialized
  Drive(){
    setPID(1);
    setSlew(0);
    setSlewMin(30);
    setStandStill(DEFAULT_STANDSTILL);
  }

  ~Drive(){}

  //Setters
  void setPID(uint8_t n);
  void setMaxVelocity(float velocity);
  void setMaxTurnVelocity(float velocity);
  void setCustomPID(float kp,float kp_t,float ki,float ki_t,float kd,float kd_t,float kp_d,float kp_s);
  void setSlew(float n);
  void setSlewMin(float n);
  void setStandStill(float standStill);
  void addErrorFunc(float onError, void input());

  //Getters
  float getError();
  bool getPIDStatus();
  
  //Movement Functions, Return error after movement is finished
  float move(PID_dir dir, float target, float timeOut, float maxVelocity);
  float swerve(PID_dir dir, float target, float target_a, float timeOut, float maxVel, float maxVel_a);
  float hardStop(PID_dir dir, float targetCutOff, float target, float timeOut, float maxVelocity);

  void platformBalance(float target_a, float vel_drive, float vel_climb);
  float yellowFight(PID_dir dir, float target, float fightError, float timeOut, bool elims);
  float yellowGrab(PID_dir dir, float estimated, float limit, float timeOut, float maxVel);

  float brake(float timeOut);

};Drive drive;


float Drive::headingValue(){
  return imu.get_heading();
}

float Drive::rotationValue(){
  return imu.get_rotation();
}


//Set PID constants
void Drive::setPID(uint8_t n){
  kP = pidConstants[n-1][0];
  kP_t = pidConstants[n-1][1];
  kI = pidConstants[n-1][2];
  kI_t = pidConstants[n-1][3];
  kD = pidConstants[n-1][4];
  kD_t = pidConstants[n-1][5];
  kP_d = pidConstants[n-1][6];
  kP_s = pidConstants[n-1][7];
}

void Drive::setCustomPID(float kp,float kp_t,float ki,float ki_t,float kd,float kd_t,float kp_d,float kp_s){
  kP = kp;
  kP_t = kp_t;
  kI = ki; 
  kI_t = ki_t;
  kD = kd;
  kD_t = kd_t;
  kP_d = kp_d;
  kP_s = kp_s;
}

void Drive::setMaxVelocity(float velocity){
  maxVolt = percentToVoltage(velocity);
}

void Drive::setMaxTurnVelocity(float velocity){
  maxVolt_a = percentToVoltage(velocity);
}

void Drive::setSlew(float n){
  slew = n*2;
}

void Drive::setSlewMin(float n){
  slewMin = n*2;
}

void Drive::setStandStill(float standStill){
  if(standStill != STANDSTILL_OFF){
    SSActive = true;
    standStillInput = standStill;
  }
  else{
    SSActive = false;
    standStillInput = DEFAULT_STANDSTILL;
  }
}

float Drive::getError(){
  return error;
}

bool Drive::getPIDStatus(){
  return isNewPID;
}

/********************************************************************************************************/

//Basic PID movement function
float Drive::move(PID_dir dir, float target, float timeOut, float maxVelocity){
  //Error values//
  float lastError;
  float errorDrift;
  float errorSlop;
  const float initialAngle = rotationValue() + 360;
  const float initialAngle_d = headingValue();
  const float initialMotorAvg = motorAvgAll();
  const float tickTarget = inchToTick(target);
  //Calc var declarations//
  float &proportion = error;
  float integral;
  float derivative;
  float proportionDrift;
  float proportion_slop;
  //integral var declarations//
  const float integralActive = inchToTick(3);
  const float integralActive_t = 4;
  //Motor output var declarations//
  maxVolt = percentToVoltage(maxVelocity);
  float finalVolt;
  //Forward Backward movement multipliers//
  int8_t reverseVal = (dir == backward || dir == right)?(-1):(1);
  //timeOut var declaration//
  uint8_t standStillCount = 0;
  const uint8_t standStillTolerance = ((standStillInput==standStillInput)?(standStillInput):(10));
  const uint8_t standStillTolerance_t = ((standStillInput==standStillInput)?(standStillInput):(10));
  bool standStill = false;
  //Tell the onError task that a new PID has begun//
  isNewPID = true;
  const uint32_t endTime = pros::millis() + timeOut*1000;
  //Begin PID//
  if(dir == forward || dir == backward){
    while(pros::millis() < endTime && !standStill){
      error = tickTarget - fabs(motorAvgAll() - initialMotorAvg);
      if(fabs(error) < integralActive){integral = error;}
      else{integral = 0;}
      derivative = error - lastError;
      if(SSActive&&fabs(lastError-error)<=DISTANCE_STANDSTILL_THRESH){
        standStillCount++;
        if(standStillCount > standStillTolerance){standStill = true;}
      }
      else{standStillCount = 0;}
      master.print(2, 0, "Error: %.2f", tickToInch(error));
      lastError = error;
      //Calculate the value that will be fed into max voltage and slew
      finalVolt = kP*proportion + kI*integral + kD*derivative;
      finalVolt = mapToRange(finalVolt, maxVolt, -maxVolt);
      if(slew && (fabs(actualVelocityAll()) > slewMin)){
        int avgVelocity = actualVelocityAll();
        float tempDelta = voltageToVelocity(finalVolt) - avgVelocity;
        if(fabs(tempDelta) > slew){
          float velDelta = slew;
          if(tempDelta < 0){velDelta *= -1;}
          finalVolt = mapToRange(float(velocityToVoltage(avgVelocity + velDelta)), maxVolt, -maxVolt);
        }
      }      
      errorDrift = fmod((initialAngle_d-headingValue()+540),360) - 180;
      proportionDrift = errorDrift * kP_d;
      //Set Drivetrain
      moveRightDriveTrain((reverseVal*finalVolt)+proportionDrift);
      moveLeftDriveTrain((reverseVal*finalVolt)-proportionDrift);
      //Give PROS time to keep itself in order
      pros::delay(20);
    }
    //Stop the robot for 20 milliseconds
    setBrakeMode(MOTOR_BRAKE_HOLD);
    moveDriveVoltage(0);
    pros::delay(20);
    setBrakeMode(MOTOR_BRAKE_COAST);
    //Tell the onError task that the PID is over
    isNewPID = false;
    //Exit the function
    return tickToInch(error);
  }
  else if(dir == right || dir == left || dir == shortest){
    //Change the reverseVal and target if the direction input is shortest
    if(dir == shortest){
      target = fmod((target-headingValue()+540),360) - 180;
      reverseVal = (target >= 0)?(-1):(1);
      target = fabs(target);
    }
    //Begin PID
    while((pros::millis() < endTime && !standStill)){
      error = target - fabs(rotationValue() + 360 - initialAngle);
      if(fabs(error) < integralActive_t){integral = error;}
      else{integral = 0;}
      derivative = error - lastError;
      if(SSActive && fabs(lastError - error) <= TURN_STANDSTILL_THRESH){
        standStillCount++;
        if(standStillCount > standStillTolerance_t){standStill = true;}
      }
      else{standStillCount = 0;}
      lastError = error;
      finalVolt = kP_t*proportion + kI_t*integral + kD_t*derivative;
      finalVolt = mapToRange(finalVolt, maxVolt, -maxVolt);
      master.print(2,0, "Error: %.2f", error);
      if(slew && (fabs(actualVelocityLeft() - actualVelocityRight()) > slewMin)){
        int avgVelocity = actualVelocityLeft() - actualVelocityRight();
        float tempDelta = voltageToVelocity(finalVolt) - avgVelocity;
        if(fabs(tempDelta) > slew){
          float velDelta = slew;
          if(tempDelta < 0){velDelta *= -1;}
          finalVolt = mapToRange(float(velocityToVoltage(avgVelocity + velDelta)), maxVolt, -maxVolt);
        }
      }
      //TODO CONTINUE TO TEST SLOP MOVEMENT
      errorSlop = initialMotorAvg - motorAvgAll();
      proportion_slop = errorSlop * kP_s;
      //Set drivetrain
      moveRightDriveTrain(reverseVal*(-finalVolt)+proportion_slop);
      moveLeftDriveTrain((reverseVal*finalVolt)+proportion_slop);
      //Give PROS time to keep itself in order
      pros::delay(20);
    }
    //Stop the robot for 20 millisecond
    setBrakeMode(MOTOR_BRAKE_HOLD);
    moveDriveVoltage(0);
    pros::delay(20);
    setBrakeMode(MOTOR_BRAKE_COAST);
    //Tell the onError task that the PID is over
    isNewPID = false;
    //Exit the function
    return error;
  }
  return 0;
}

/********************************************************************************************************/

float Drive::swerve(PID_dir dir, float target, float target_a, float timeOut, float maxVel, float maxVel_a){
  float finalValueLeft;
  float finalValueRight;
  float error_a;
  float lastError;
  float lastError_a;
  const float initialAngle = rotationValue() + 360;
  const float initialMotorAvg = motorAvgAll();
  const float tickTarget = inchToTick(target);
  //Calc var declarations//
  float &proportion = error;
  float &proportion_a = error_a;
  float integral;
  float integral_a;
  float derivative;
  float derivative_a;
  //integral var declarations//
  const float integralActive = inchToTick(3);
  const float integralActive_a = 4;
  //Motor output var declarations//
  maxVolt = percentToVoltage(maxVel);
  maxVolt_a = percentToVoltage(maxVel_a);
  float finalVolt;
  //Take shortest directions into account
  //Forward Backward movement multipliers//
  const int8_t reverseVal = (dir == backwardLeft || dir == backwardRight || dir == backwardShortest)?(-1):(1);
  const int8_t reverseVal_a = 
  (dir == backwardRight || dir == forwardRight || dir == forwardShortest || dir == backwardShortest)?(-1):(1);
  const bool isShortest = (dir == forwardShortest || dir == backwardShortest)?(true):(false);
  //Standstill variables//
  uint8_t standStillCount = 0;
  uint8_t standStillCount_a = 0;
  const uint8_t standStillTolerance = ((standStillInput==standStillInput)?(standStillInput):(10));
  const uint8_t standStillTolerance_a = ((standStillInput==standStillInput)?(standStillInput):(10));
  bool standStill = false;
  bool standStill_a = false;
  //Tell the onError task that a new PID has begun//
  isNewPID = true;
  //End time variable declaration//
  const uint32_t endTime = pros::millis() + timeOut*1000;
  //Begin PID//
  while(pros::millis() < endTime && !(standStill && standStill_a)){
    /********************************************DRIVE***************************************************/
    error = tickTarget - fabs(motorAvgAll() - initialMotorAvg);
    if(fabs(error) < integralActive){integral = error;}
    else{integral = 0;}
    derivative = error - lastError;
    if(SSActive && fabs(lastError - error) <= DISTANCE_STANDSTILL_THRESH){
      standStillCount++;
      if(standStillCount > standStillTolerance){standStill = true;}
    }
    else{standStillCount = 0;}
    lastError = error;
    finalVolt = kP*proportion + kI*integral + kD*derivative;
    finalVolt = mapToRange(finalVolt, maxVolt, -maxVolt);
    if(slew && (fabs(actualVelocityAll()) > slewMin)){
      int avgVelocity = actualVelocityAll();
      float tempDelta = voltageToVelocity(finalVolt) - avgVelocity;
      if(fabs(tempDelta) > slew){
        float velDelta = slew;
        if(tempDelta < 0){velDelta *= -1;}
        finalVolt = mapToRange(float(velocityToVoltage(avgVelocity + velDelta)), maxVolt, -maxVolt);
      }
    }
    finalValueRight = reverseVal*finalVolt;
    finalValueLeft = reverseVal*finalVolt;
    /********************************************TURN****************************************************/
    if(!isShortest){error_a = target_a - fabs(rotationValue() + 360 - initialAngle);}
    else{error_a = fmod((target_a-headingValue()+540),360) - 180;}
    if(fabs(error_a) < integralActive_a){integral_a = error_a;}
    else{integral_a = 0;}
    derivative_a = error_a - lastError_a;
    if(SSActive && fabs(lastError_a - error_a) <= TURN_STANDSTILL_THRESH){
      standStillCount_a++;
      if(standStillCount_a > standStillTolerance_a){standStill_a = true;}
    }
    else{standStillCount_a = 0;}
    lastError_a = error_a;
    finalVolt = kP_t*proportion_a + kI_t*integral_a + kD_t*derivative_a;
    finalVolt = mapToRange(finalVolt, maxVolt_a, -maxVolt_a);
    if(slew){
      int avgVelocity = actualVelocityLeft() - actualVelocityRight();
      float tempDelta = voltageToVelocity(finalVolt) - avgVelocity;
      if(fabs(tempDelta) > slew){
        float velDelta = slew;
        if(tempDelta < 0){velDelta *= -1;}
        finalVolt = mapToRange(float(velocityToVoltage(avgVelocity + velDelta)), maxVolt_a, -maxVolt_a);
      }
    }
    master.print(2,0,"%.2f ,%.2f                  ", tickToInch(error), error_a);
    //master.print(2,0,"%.2f ,%.2f                  ", tickToInch(error), target_a);
    finalValueRight += reverseVal_a*(-finalVolt);
    finalValueLeft += reverseVal_a*(finalVolt);
    //Set drivetrain
    moveRightDriveTrain(finalValueRight);
    moveLeftDriveTrain(finalValueLeft);
    //Give PROS time to keep itself in order
    pros::delay(20);
  }
  //Stop the robot for 20 milliseconds
  setBrakeMode(MOTOR_BRAKE_HOLD);
  moveDriveVoltage(0);
  pros::delay(20);
  setBrakeMode(MOTOR_BRAKE_COAST);
  //Tell the onError task that the PID is over
  isNewPID = false;
  //Exit the function
  return tickToInch(error);
}

/********************************************************************************************************/

float Drive::hardStop(PID_dir dir, float targetCutOff, float target, float timeOut, float maxVelocity){
  float progress;
  float errorDrift;
  float lastError;
  const float initialAngle = rotationValue() + 360;
  const float initialAngle_d = headingValue();
  const float initialMotorAvg = motorAvgAll();
  const float tickTarget = inchToTick(target);
  const float tickTargetCutOff = inchToTick(targetCutOff);
  //Calc var declarations//
  float &proportion = error;
  float proportionDrift;
  float derivative;
  //Motor output var declarations//
  maxVolt = percentToVoltage(maxVelocity);
  float finalVolt;
  float velDelta;
  float tempDelta;
  //Forward Backward movement multipliers
  const int8_t reverseVal = (dir == backward || dir == right)?(-1):(1);
  //Tell the onError task that a new PID has begun
  isNewPID = true;
  //Establish a cutoff in case something goes wrong
  const uint32_t endTime = pros::millis() + timeOut*1000;
  //Begin PID
  while(tickTargetCutOff > fabs(motorAvgAll()-initialMotorAvg) && (pros::millis() < endTime)){
    error = tickTarget - fabs(motorAvgAll() - initialMotorAvg);
    derivative = error - lastError;
    lastError = error;
    finalVolt = mapToRange(float(kP*proportion + kD*derivative), maxVolt, -maxVolt);

    if(slew && (fabs(actualVelocityAll()) > slewMin)){
      int avgVelocity = actualVelocityAll();
      float tempDelta = voltageToVelocity(finalVolt) - avgVelocity;
      if(fabs(tempDelta) > slew){
        float velDelta = slew;
        if(tempDelta < 0){velDelta *= -1;}
        finalVolt = mapToRange(float(velocityToVoltage(avgVelocity + velDelta)), maxVolt, -maxVolt);
      }
    }

    master.print(2,0, "Error: %.2f", error);
    //Set finalVolt to range
    
    errorDrift = fmod((initialAngle_d-headingValue()+540),360) - 180;
    proportionDrift = errorDrift * kP_d;
    //Set Drivetrain
    moveRightDriveTrain((reverseVal*finalVolt)+proportionDrift);
    moveLeftDriveTrain((reverseVal*finalVolt)-proportionDrift);
    //Give PROS time to keep itself in order
    pros::delay(20);
  }
  //Set voltage to 0 in case this is the last function called in an autonomous
  moveDriveVoltage(0);
  //Tell the onError task that the PID is over
  isNewPID = false;
  //Exit the function
  return tickToInch(error);
}

/********************************************************************************************************/

float Drive::yellowFight(PID_dir dir,float target,float fightError,float timeOut,bool elims){
  //Error values//
  float lastError;
  float errorDrift;
  const float initialAngle_d = headingValue();
  const float initialMotorAvg = motorAvgAll();
  const float tickTarget = inchToTick(target);
  //Calc var declarations//
  float &proportion = error;
  float integral;
  float derivative;
  float proportionDrift;
  float tilt;
  const float kP_tilt = -1000;
  const float tiltAdjustment = 3.5;
  //integral var declarations//
  const float integralActive = inchToTick(3);
  //Motor output var declarations//
  maxVolt = MAX_VOLTAGE;
  float finalVolt;
  //Forward Backward movement multipliers//
  const int8_t reverseVal = (dir == backward)?(-1):(1);
  //timeOut var declaration//
  uint8_t standStillCount = 0;
  const uint8_t standStillTolerance = ((standStillInput==standStillInput)?(standStillInput):(10));
  bool standStill = false;
  bool fighting = false;
  float avgFight;
  std::vector<float> fights; 
  //Tell the onError task that a new PID has begun//
  isNewPID = true;
  const uint32_t endTime = pros::millis() + timeOut*1000;
  //Begin PID//
  while(pros::millis() < endTime && !standStill){
    error = tickTarget - fabs(motorAvgAll() - initialMotorAvg);
    if(fabs(error) < integralActive){integral = error;}
    else{integral = 0;}
    derivative = error - lastError;

    tilt = imu.get_roll() - tiltAdjustment;




    fights.push_back(driveEfficiencyAll());
    int sizeAdjusted = MIN(fights.size(),20);
    avgFight = float(std::accumulate(fights.end() - sizeAdjusted, fights.end(), 0))/sizeAdjusted;
    fighting = (avgFight >= 25);




    if(SSActive&&fabs(lastError-error)<=DISTANCE_STANDSTILL_THRESH && fightError>=error && !fighting){
      standStillCount++;
      if(standStillCount > standStillTolerance){standStill = true;}
    }
    else{standStillCount = 0;}
    master.print(2, 0, "%.2f, %.2f                   ", tickToInch(error), avgFight);
    lastError = error;
    //Calculate the value that will be fed into max voltage and slew
    finalVolt = kP*proportion + kI*integral + kD*derivative;
    finalVolt = mapToRange(finalVolt, maxVolt, -maxVolt);
    finalVolt += tilt*kP_tilt;
    if(slew && (fabs(actualVelocityAll()) > slewMin)){
      int avgVelocity = actualVelocityAll();
      float tempDelta = voltageToVelocity(finalVolt) - avgVelocity;
      if(fabs(tempDelta) > slew){
        float velDelta = slew;
        if(tempDelta < 0){velDelta *= -1;}
        finalVolt = mapToRange(float(velocityToVoltage(avgVelocity + velDelta)), maxVolt, -maxVolt);
      }
    }      
    errorDrift = fmod((initialAngle_d-headingValue()+540),360) - 180;
    proportionDrift = errorDrift * kP_d;
    //Set Drivetrain
    moveRightDriveTrain((reverseVal*finalVolt)+proportionDrift);
    moveLeftDriveTrain((reverseVal*finalVolt)-proportionDrift);
    //Give PROS time to keep itself in order
    pros::delay(20);
  }
  //Stop the robot for 20 milliseconds
  setBrakeMode(MOTOR_BRAKE_HOLD);
  moveDriveVoltage(0);
  pros::delay(20);
  setBrakeMode(MOTOR_BRAKE_COAST);
  //Tell the onError task that the PID is over
  isNewPID = false;
  //Exit the function
  return tickToInch(error);
}

/********************************************************************************************************/

float Drive::yellowGrab(PID_dir dir, float estimated, float limit, float timeOut, float maxVel){
  //Error values//
  float ticksTraveled;
  float lastTicksTraveled;
  float errorDrift;
  float proportionDrift;
  const float initialAngle_d = headingValue();
  const float initialMotorAvg = motorAvgAll();
  const float tickLimit = inchToTick(limit);
  const float tickTarget = inchToTick(estimated);
  //Calc var declarations//
  float &proportion = error;
  float derivative;
  //Motor output var declarations//
  float finalVolt;
  maxVolt = percentToVoltage(maxVel);
  const float minVolt = 6000;
  //Forward Backward movement multipliers//
  const int8_t reverseVal = (dir == backward)?(-1):(1);
  //Goal switch variables//
  bool goalSwitched;
  uint8_t goalSwitchCount = 0;
  //Standstill variable declarations//
  uint8_t standStillCount = 0;
  const uint8_t standStillTolerance = ((standStillInput==standStillInput)?(standStillInput):(10));
  bool standStill = false;
  //timeOut var declaration//
  const uint32_t endTime = pros::millis() + timeOut*1000;
  //Begin PID//
  while((pros::millis() < endTime) && (!standStill) && (ticksTraveled <= tickTarget) && !goalSwitched){
    //Check the goal limit switches immediately before beggining the next step
    goalSwitched = goalSwitches.get_value();
    //Calculate distance traveled
    ticksTraveled = fabs(motorAvgAll() - initialMotorAvg);
    error = tickTarget - ticksTraveled;
    derivative = lastTicksTraveled - ticksTraveled; 
    if(SSActive && fabs(lastTicksTraveled - ticksTraveled) <= DISTANCE_STANDSTILL_THRESH){
      standStillCount++;
      if(standStillCount > standStillTolerance){standStill = true;}
    }
    else{standStillCount = 0;}
    master.print(2, 0, "%.2f                   ", tickToInch(ticksTraveled));
    finalVolt = proportion*kP /*+ derivative*kD*/;
    if(finalVolt >= 0){finalVolt = mapToRange(finalVolt, maxVolt, minVolt) * !goalSwitched;}
    //else{finalVolt = mapToRange(finalVolt, -minVolt, -maxVolt) * !goalSwitched;}
    //Drift calculations
    errorDrift = fmod((initialAngle_d-headingValue()+540),360) - 180;
    proportionDrift = errorDrift * kP_d;
    //Set Drivetrain
    moveRightDriveTrain((reverseVal*finalVolt)+proportionDrift);
    moveLeftDriveTrain((reverseVal*finalVolt)-proportionDrift);
    //Give PROS time to keep itself in order
    pros::delay(20);
  }
  //Check if goal actually clamped
  if(goalSwitched){
    moveDriveVoltage(minVolt);
    pros::delay(20);
    liftClamp.set_value(LOW);
    pros::delay(20);
    return tickToInch(ticksTraveled);
  }
  //If not do the rest of the without goal cover because if they already have the goal it does nothing
  
  while((pros::millis() < endTime) && (!standStill) && (ticksTraveled <= tickLimit) && goalSwitchCount <= 2){
    //Calculate distance traveled
    ticksTraveled = fabs(motorAvgAll() - initialMotorAvg);
    if(SSActive && fabs(lastTicksTraveled - ticksTraveled) <= DISTANCE_STANDSTILL_THRESH){
      standStillCount++;
      if(standStillCount > standStillTolerance){standStill = true;}
    }
    else{standStillCount = 0;}
    master.print(2, 0, "%.2f                   ", tickToInch(ticksTraveled));
    finalVolt = maxVolt;
    //Drift calculations
    errorDrift = fmod((initialAngle_d-headingValue()+540),360) - 180;
    proportionDrift = errorDrift * kP_d;
    //Set Drivetrain
    moveRightDriveTrain((reverseVal*finalVolt)+proportionDrift);
    moveLeftDriveTrain((reverseVal*finalVolt)-proportionDrift);
    //Give PROS time to keep itself in order
    pros::delay(20);
    //Check the goal limit switches immediately before beggining the next step
    goalSwitchCount += goalSwitches.get_value();
  }
  liftClamp.set_value(LOW);
  pros::delay(20);
  //Exit the function
  return tickToInch(ticksTraveled);
}

/********************************************************************************************************/

void Drive::platformBalance(float target_a, float vel_drive, float vel_climb){
//Error values//
  float lastError;
  float errorDrift;
  //Calc var declarations//
  float &proportion = error;
  float integral;
  float derivative;
  float proportionDrift;
  //Motor output var declarations//
  float driveVolt = percentToVoltage(vel_drive);
  float climbVolt = percentToVoltage(vel_climb);
  float finalVolt;
  //Tell the onError task that a new PID has begun//
  isNewPID = true;

  //Drive up until the robot is completely on the platform//
  while((imu.get_roll() < 21) && (imu.get_roll() > -15)){
    finalVolt = driveVolt;
    errorDrift = fmod((target_a-headingValue()+540),360) - 180;
    proportionDrift = errorDrift * kP_d;
    //Set Drivetrain
    moveRightDriveTrain(finalVolt+proportionDrift);
    moveLeftDriveTrain(finalVolt-proportionDrift);
    //Give PROS time to keep itself in order
    pros::delay(20);
  }

  //Climb up the platform until the robot begins to tip//
  float estimatedTarget = inchToTick(23);
  float initialMotorAvg = motorAvgAll();
  while((imu.get_roll() > 21) || (imu.get_roll() < -15)){//+- .5
    error = estimatedTarget - fabs(motorAvgAll() - initialMotorAvg);
    finalVolt = climbVolt * (error+2000/estimatedTarget);
    finalVolt = mapToRange(finalVolt, climbVolt, 5000.0f);
    errorDrift = fmod((target_a-headingValue()+540),360) - 180;
    proportionDrift = errorDrift * kP_d;
    //Set Drivetrain
    moveRightDriveTrain(finalVolt+proportionDrift);
    moveLeftDriveTrain(finalVolt-proportionDrift);
    //Give PROS time to keep itself in order
    pros::delay(20);
  }
}

/********************************************************************************************************/

float Drive::brake(float timeOut){
  const uint32_t endTime = pros::millis() + timeOut*1000;
  const float target = motorAvgAll();
  const float kP_brake = 30;
  while(pros::millis() < endTime){
      error = target - motorAvgAll();
      moveDriveVoltage(error*kP_brake);
  }
  return tickToInch(error);
}