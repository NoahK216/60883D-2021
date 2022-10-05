#pragma once
#include "main.h"

#define PI 3.14159
#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))
#define AUTO_NUMBER 9
#define LIFTVALSIZE 4

//Global Variable Declaration
uint8_t auton = AUTO_NUMBER;          //Subtract Rotation from 360 to find liftTargets
const float liftTargets[LIFTVALSIZE] = {198, 249, 270, 310}; //first used to be 197 third was 270

//Declare the controller
pros::Controller master(CONTROLLER_MASTER);

//Declare motors
pros::Motor frontright(19 ,false);
pros::Motor midright(18, true);
pros::Motor backright(20, false);

pros::Motor frontleft(12, true);
pros::Motor midleft(13, false);
pros::Motor backleft(11, true);

pros::Motor lift(1, true);
pros::Motor conveyormotor(14, false);

//Declare V5 sensors
pros::Rotation liftPot(10);
pros::Imu imu(2);


//Declare ADI Out ports
pros::ADIDigitalOut liftClamp(1);
pros::ADIDigitalOut backClamp(2);
pros::ADIDigitalOut stick(3);

//Declare ADI In ports
pros::ADIDigitalIn goalSwitches(4);
