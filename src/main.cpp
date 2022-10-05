#include "auton.cpp"
#include "lvgl_functions.cpp"
#define AUTO_SWTICH_THRESH 90
#define DRIVE_BRAKE_THRESH 100

#define AUTO_SWITCH(){ \
	switch(auton%AUTO_NUMBER){\
		case 0: master.print(2, 0, "Win Point, %.2f            ", imu.get_heading()); break;\
		case 1: master.print(2, 0, "Left Side, %.2f            ", imu.get_heading()); break;\
		case 2: master.print(2, 0, "18 Left, %.2f           ", imu.get_heading()); break;\
		case 3: master.print(2, 0, "Left Shtick, %.2f           ", imu.get_heading()); break;\
		case 4: master.print(2, 0, "Left Elims, %.2f          ", imu.get_heading()); break;\
		case 5: master.print(2, 0, "18 Mid, %.2f             ", imu.get_heading()); break;\
		case 6: master.print(2, 0, "Mid Swerve, %.2f           ", imu.get_heading()); break;\
		case 7: master.print(2, 0, "Skills, %.2f               ", imu.get_heading()); break;\
		case 8: master.print(2, 0, "Nothing, %.2f              ", imu.get_heading()); break;\
	}\
}


void initialize(){
	initBarGraph();
	pros::Task brainDisplayTask(updateBarGraph_fn);
	liftPot.set_data_rate(5);
	lift.set_brake_mode(MOTOR_BRAKE_HOLD);

	setBrakeMode(MOTOR_BRAKE_COAST);
}


void disabled(){
	while(true){
		//Change auton value
		if(master.get_digital_new_press(DIGITAL_LEFT)){auton--;}
		if(master.get_digital_new_press(DIGITAL_RIGHT)){auton++;}
		
		AUTO_SWITCH();
		pros::delay(20);
	}
}


void competition_initialize(){
	while(true){
		//Change auton value
		if(master.get_digital_new_press(DIGITAL_LEFT)){auton--;}
		if(master.get_digital_new_press(DIGITAL_RIGHT)){auton++;}
		
		AUTO_SWITCH();
		pros::delay(20);
	}
}


void autonomous(){
	switch(auton%AUTO_NUMBER){
		case 0: winpoint();		break;
		case 1: leftSide();		break;
		case 2: left18(); 	break;
		case 3: leftElims();	break;
		case 4: leftShtick();	break;
		case 5: middle18();	break;
		case 6: middle_swerve();break;
		case 7: skills();		break;
		case 8: empty();		break;
	}
}


void opcontrol(){
	//Set initial boolean variables
	bool liftManual = false;
	bool highPostControls = false;
	int8_t highPostTimer = 0;
	//Set piston states from auto
	backClamp.set_value(pneumatics.backPistonToggle);
	liftClamp.set_value(pneumatics.pistonLiftToggle);
	stick.set_value(pneumatics.stickPistonToggle);
	while(true){
		//Assign variables to joystick values for ease of use
		int8_t leftX = master.get_analog(ANALOG_LEFT_X);
		int8_t leftY = master.get_analog(ANALOG_LEFT_Y);
		int8_t rightX = master.get_analog(ANALOG_RIGHT_X);
		int8_t rightY = master.get_analog(ANALOG_RIGHT_Y);


		//Change auton value
		if(master.get_digital_new_press(DIGITAL_LEFT)){auton--;}
		if(master.get_digital_new_press(DIGITAL_RIGHT)){auton++;}


		//Reset the IMU when the up button is called
		if(master.get_digital_new_press(DIGITAL_UP)){
            imu.reset();
            float iter = 0;
            while(imu.is_calibrating()){
                master.print(2,0,"Calibrating: %.2f    ", iter/1000);
                iter += 20;
                pros::delay(20);
            }
		}

		
		//Display current autonomous on the controller
		AUTO_SWITCH();
		
		
		//Set drive motor speeds
		int rightDrive = leftY - rightX;
		frontright = rightDrive;
		midright = rightDrive;
		backright = rightDrive;

		int leftDrive = leftY + rightX;
		frontleft = leftDrive;
		midleft = leftDrive;
		backleft = leftDrive;
		

		//Adjust toggleable values
		if(master.get_digital_new_press(DIGITAL_A)){
			pneumatics.toggleLiftClamp();
		}
		if(master.get_digital_new_press(DIGITAL_B)){
			pneumatics.toggleStick();
		}	
		if(master.get_digital_new_press(DIGITAL_X)){
			pneumatics.toggleBackClamp();
		}
		if(master.get_digital_new_press(DIGITAL_Y)){
			liftManual =! liftManual;
			if(!liftManual){
				liftPID.setTarget(getClosestIndicator(liftPot.get_angle()));
			}
		}


		//Toggle brake control
		//if(rightY >= 110) setBrakeMode(MOTOR_BRAKE_COAST);
		//else if(rightY <= -110) setBrakeMode(MOTOR_BRAKE_HOLD);


		//Set conveyor speed
		if(master.get_digital(DIGITAL_R1)){conveyormotor.move_velocity(200);}
		else if(master.get_digital(DIGITAL_R2)){conveyormotor.move_velocity(-200);}
		else{conveyormotor.move_velocity(0);}


		//Lift control
		if(liftManual){
			if(master.get_digital(DIGITAL_L1)){lift = 127;}
			else if(master.get_digital(DIGITAL_L2)){lift = -127;}
			else{lift = 0;}
		}
		else{
			if(master.get_digital_new_press(DIGITAL_L1) && liftPID.targetIndicator < LIFTVALSIZE-1){
				liftPID.targetIndicator++;
				liftPID.target = liftTargets[liftPID.targetIndicator%LIFTVALSIZE];
			}
			else if(master.get_digital_new_press(DIGITAL_L2) && liftPID.targetIndicator > 0){
				liftPID.targetIndicator--;
				liftPID.target = liftTargets[liftPID.targetIndicator%LIFTVALSIZE];
			}
			liftPID.PID();
		}

		
		//Run the currently selected autonomous when B is pressed
		if(master.get_digital_new_press(DIGITAL_DOWN)){
			#if SKILLSONDOWN
				skills();
			#else
				autonomous();
			#endif
		}


		pros::delay(20);
	}
}
