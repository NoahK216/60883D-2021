#pragma once
#include "include.cpp"
#define MIN_VOLTAGE -12000
#define MAX_VOLTAGE 12000


void setBrakeMode(pros::motor_brake_mode_e brakeMode){
  frontright.set_brake_mode(brakeMode);
  midright.set_brake_mode(brakeMode);
  backright.set_brake_mode(brakeMode);
  frontleft.set_brake_mode(brakeMode);
  midleft.set_brake_mode(brakeMode);
  backleft.set_brake_mode(brakeMode);
}

void moveLeftDriveTrain(int voltage){
  frontright.move_voltage(voltage);
  midright.move_voltage(voltage);
  backright.move_voltage(voltage);
}

void moveRightDriveTrain(int voltage){
  frontleft.move_voltage(voltage);
  midleft.move_voltage(voltage);
  backleft.move_voltage(voltage);
}

void moveDriveVoltage(int voltage){
  frontright.move_voltage(voltage);
  midright.move_voltage(voltage);
  backright.move_voltage(voltage);
  frontleft.move_voltage(voltage);
  midleft.move_voltage(voltage);
  backleft.move_voltage(voltage);
}

void moveDriveVelocity(float velocity){
  frontleft.move_velocity(velocity);
	midleft.move_velocity(velocity);
	backleft.move_velocity(velocity);
	frontright.move_velocity(velocity);
	midright.move_velocity(velocity);
	backright.move_velocity(velocity);
}

void moveDriveTrain(int voltage, float time){
  moveDriveVoltage(voltage);
  pros::delay(time*1000);
  moveDriveVoltage(0);
}

int motorAvgLeft(){
  return(backleft.get_position()+midleft.get_position()+frontleft.get_position())/3;
}

int motorAvgRight(){
  return(frontright.get_position()+midright.get_position()+backright.get_position())/3;
}

int motorAvgAll(){
  return(motorAvgLeft()+motorAvgRight())/2;
}

float actualVelocityLeft(){
  return(frontleft.get_actual_velocity() + midleft.get_actual_velocity() + backleft.get_actual_velocity())/3;
}

float actualVelocityRight(){
  return(frontright.get_actual_velocity() + midright.get_actual_velocity() + backright.get_actual_velocity())/3;
}

//returns a float between -200 and 200
float actualVelocityAll(){
  return(actualVelocityLeft() + actualVelocityRight())/2;
}

//returns a float between 0 and 100, representing how much energy is being used compared to actual movement
float driveEfficiencyAll(){
  return((frontright.get_efficiency() + midright.get_efficiency() + backright.get_efficiency() + 
          frontleft.get_efficiency() + midleft.get_efficiency() + backleft.get_efficiency())/6);
}

template <typename T>
T mapToRange(T num, T maxVal, T minVal){
  num = MIN(num, maxVal);
  num = MAX(num, minVal);
  return(num);
}

float radToDeg(float radian){
  return(radian * (180/PI));
}

float degToRad(float degree){
  return(degree * (PI/180));
}

/*
 *1800 ticks/rev with 100 rpm cartridge
 *900 ticks/rev with 200 rpm cartridge
 *300 ticks/rev with 600 rpm cartridge
 *642.86 ticks for one full wheel rotation (900 * (60/84) or (5/7))
 *Circumference of 4" omni = 12.96 (4.125*pi)
 *642.86 ticks / 12.96 in = 49.60 ticks per inch
 */
int inchToTick(float inch){
  return(inch * 49.6);
}

float tickToInch(int tick){
  return(tick / 49.6);
}

float percentToVoltage(float percent){
  return(percent * 120);
}

//(voltage / 120)/2
float voltageToVelocity(float voltage){
  return(voltage / 60);
}

int velocityToVoltage(float percent){
  return(percent * 60);
}


/**
 * Find the shortest distance between two angles
 * \param target
 *        Desired angle
 * \returns Shortest rotation between current and target angle in degrees
 *        
 */
float imuTarget(float target){
  return(fabs(fmod((target-imu.get_heading()+540),360) - 180));
}

//Stupid ugly function with nested if loops to get the closest value to the passed through value from an array
int getClosestIndicator(float liftRot){
  liftRot = 360 - (liftRot/100);
  if (liftRot - liftTargets[0] >= liftTargets[1] - liftRot){
    if(liftRot - liftTargets[1] >= liftTargets[2] - liftRot){
      if(liftRot - liftTargets[2] >= liftTargets[3] - liftRot){
        return 3;
      }
      return 2;
    }
    return 1;
  }
  return 0;
}


void setConveyorRaw(int16_t velocity){
  conveyormotor.move_velocity(velocity/2);
}


struct Triangle {
public:
  float a;
  float b;
  float hyp;
  float alpha;
  float beta;
};


/**
 * Does trigonometry to assign values to attributes of a given Triangle object
 *
 * \param obj
 *        Reference to the Triangle object who's attributes will be set
 * \param a
 *        Leg A of the triangle
 * \param reference
 *        Angle to subtract from the current IMU heading. 
 *        Will set hyp equal to A if reference and IMU heading are the same.   
 */
void findTri(Triangle& obj, float a, float reference){
  obj.beta = degToRad(imu.get_heading() - reference);
  obj.alpha = 90 - obj.beta;
  obj.a = a;
  obj.b =  a * tan(obj.beta);
  obj.hyp = sqrt(a*a + obj.b*obj.b);
}