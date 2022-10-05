#include "PID.cpp"
#include "onErrorPID.cpp"
#include "robotClasses.cpp"
#define SKILLSONDOWN 0
#define ELIMS_AUTO 1
#define USE_LIMIT_SWITCH 0


void winpoint(){
    //Capture time at the beginning of the auto
    const uint32_t startTime = pros::millis();
    //Set the lift target to ring position before starting the lift task
    liftPID.setTarget(LIFT_LOWEST);
    //Initialize the automatic lift and ring control task and the run on error task
    AIOtimeOut = 15000;
    pros::Task AIOsystem(AIOsystem_fn);
    pros::Task runOnError(onError_fn);
    //Create a Triangle object to be used later in the auto
    Triangle tri;

    //Set piston states before beginning the route
    liftClampUp();
    backClampUp();
    stickUp();

    //Drop the first ring off in the alliance
    liftClampDown();
    pros::delay(350);
    liftClampUp();

    //Turn to swerve towards the other alliance mogo
    drive.setPID(1);
    drive.move(backward, 14, 1, 100);
    drive.move(left, imuTarget(305), 1, 80);
    
    //Swerve to the other alliance mogo to pick up and do rings
    drive.addErrorFunc(91, LAMBDA(drive.setMaxVelocity(70)));
    drive.addErrorFunc(91, LAMBDA(drive.setMaxTurnVelocity(100)));
    drive.swerve(forwardRight, 102, imuTarget(0), 3, 100, 35);

    //Turn and back into the second mogo then clamp with back clamp
    drive.setPID(1);
    drive.move(left, imuTarget(180), 1, 100);
    moveDriveTrain(-6000, .8);
    backClampDown();
    moveDriveTrain(-6000, .1);

    //Swerve to pick up a yellow and start rings
    conveyormotor.move_voltage(12000);
    drive.addErrorFunc(41.5, LAMBDA(drive.setMaxTurnVelocity(80)));
    drive.addErrorFunc(10, LAMBDA(drive.setMaxVelocity(30)));
    drive.addErrorFunc(3, liftClampDown);
    drive.swerve(forwardRight, 48, imuTarget(270), 3, 60, 0);

    //Back up with yellow
    drive.swerve(backwardShortest, 50, 260, 2, 80, 100);
    backClampUp();
    pros::delay(750);
    drive.swerve(forwardShortest, 8, 270, 2, 100, 100);

    //Remove tasks used during the run and clear the vector
    AIOsystem.remove();
    runOnError.remove();
    onErrorVector.clear();
}

void leftSide(){
    //Capture time at the beginning of the auto
    const uint32_t startTime = pros::millis();
    //Set the lift target to ring position before starting the lift task
    liftPID.setTarget(LIFT_LOWEST);
    //Initialize the automatic lift and ring control task and the run on error task
    AIOtimeOut = 15000;
    pros::Task AIOsystem(AIOsystem_fn);
    pros::Task runOnError(onError_fn);
    //Create a Triangle object to be used later in the auto
    Triangle tri;

    //Ensure that the pistons are set correctly and give back clamp time to catch
    moveDriveVoltage(-6000);
    pros::delay(100);
    liftClampUp();
    backClampDown();
    moveDriveVoltage(-3000);
    pros::delay(20);

    //Swerve and pick up the mogo.
    drive.addErrorFunc(30, LAMBDA(drive.setMaxVelocity(100)));
    drive.setPID(4);
    drive.addErrorFunc(20, LAMBDA(conveyormotor.move_voltage(12000)));
    drive.addErrorFunc(5, LAMBDA(drive.setMaxVelocity(50)));
    drive.addErrorFunc(3, liftClampDown);
    drive.swerve(forwardRight, 50, imuTarget(100), 2, 62, 100);
    liftClampDown();

    float timeElapsed = float(pros::millis() - startTime)/1000;
    const float timeOut = ((ELIMS_AUTO)?(14.9 - timeElapsed):(13.5 - timeElapsed));
    const float realYellowError = drive.yellowFight(backward, 45, inchToTick(5), timeOut, ELIMS_AUTO);

    if(realYellowError <= 5){
        #if !ELIMS_AUTO
            backClampUp();
            pros::delay(500);
        #endif
        drive.move(forward, 12, 1, 50);
    }
    else{
        #if ELIMS_AUTO
            liftClampUp();
            drive.setPID(1);
            //TODO THIS IS DEFINITELY WRONG
            drive.move(backward, realYellowError, 2, 70);
        #else
            liftClampUp();
            backClampUp();
            drive.setPID(1);
            //TODO THIS IS DEFINITELY WRONG
            drive.move(backward, realYellowError, 2, 70);
            moveDriveTrain(3000, 0.2);
        #endif
    }


    //Remove tasks used during the run and clear the vector
    AIOsystem.remove();
    runOnError.remove();
    onErrorVector.clear();
}

void left18(){
    //Capture time at the beginning of the auto
    const uint32_t startTime = pros::millis();
    //Set the lift target to ring position before starting the lift task
    liftPID.setTarget(LIFT_LOWEST);
    //Initialize the automatic lift and ring control task and the run on error task
    AIOtimeOut = 15000;
    pros::Task AIOsystem(AIOsystem_fn);
    pros::Task runOnError(onError_fn);
    //Create a Triangle object to be used later in the auto
    Triangle tri;

    //Stay in 18 by 18 by 18
    backClampUp();
    pros::delay(300);
    moveDriveTrain(-3000, 0.1);

    //Ensure that the pistons are set correctly and give back clamp time to catch
    moveDriveVoltage(-6000);
    pros::delay(100);
    liftClampUp();
    backClampDown();
    moveDriveVoltage(-3000);
    pros::delay(20);

    //Swerve and pick up the mogo.
    drive.addErrorFunc(30, LAMBDA(drive.setMaxVelocity(100)));
    drive.setPID(4);
    drive.addErrorFunc(20, LAMBDA(conveyormotor.move_voltage(12000)));
    drive.addErrorFunc(5, LAMBDA(drive.setMaxVelocity(50)));
    drive.addErrorFunc(3, liftClampDown);
    drive.swerve(forwardRight, 50, imuTarget(100), 2, 62, 100);
    liftClampDown();

    float timeElapsed = float(pros::millis() - startTime)/1000;
    const float timeOut = ((ELIMS_AUTO)?(14.9 - timeElapsed):(13.5 - timeElapsed));
    const float realYellowError = drive.yellowFight(backward, 45, inchToTick(5), timeOut, ELIMS_AUTO);

    if(realYellowError <= 5){
        #if !ELIMS_AUTO
            backClampUp();
            pros::delay(500);
        #endif
        drive.move(forward, 12, 1, 50);
    }
    else{
        #if ELIMS_AUTO
            liftClampUp();
            drive.setPID(1);
            //TODO THIS IS DEFINITELY WRONG
            drive.move(backward, realYellowError, 2, 70);
        #else
            liftClampUp();
            backClampUp();
            drive.setPID(1);
            //TODO THIS IS DEFINITELY WRONG
            drive.move(backward, realYellowError, 2, 70);
            moveDriveTrain(3000, 0.2);
        #endif
    }


    //Remove tasks used during the run and clear the vector
    AIOsystem.remove();
    runOnError.remove();
    onErrorVector.clear();

}

void leftElims(){
    //Capture time at the beginning of the auto
    const uint32_t startTime = pros::millis();
    //Set the lift target to ring position before starting the lift task
    liftPID.setTarget(LIFT_LOWEST);
    //Initialize the automatic lift and ring control task and the run on error task
    AIOtimeOut = 15000;
    pros::Task AIOsystem(AIOsystem_fn);
    pros::Task runOnError(onError_fn);
    //Create a Triangle object to be used later in the auto
    Triangle tri;

    //Ensure that the lifts are set in the right position
    liftClampUp();
    backClampUp();
    stickUp();

    //Drive backward and clamp the right side yellow in the back
    drive.setCustomPID(25,225,15,25,70,120,300,0);
    drive.setStandStill(3);
    drive.addErrorFunc(24, stickDown);
    drive.hardStop(forward, 32, 33, 1.8, 100);

    //Fight for the yellow maybe
    float timeElapsed = float(pros::millis() - startTime)/1000;
    const float timeOut = ((ELIMS_AUTO)?(14.9 - timeElapsed):(11 - timeElapsed));
    const float realYellowError = drive.yellowFight(backward, 33, inchToTick(10), timeOut, ELIMS_AUTO);

    stickUp();
    
    if(realYellowError <= 5){
        drive.setPID(2);
        drive.move(left, imuTarget(320), 1, 70);
        moveDriveTrain(-6000, 0.5);
        moveDriveTrain(-2000, 0.2);
        backClampDown();
        conveyormotor.move_voltage(12000);
        pros::delay(750);
        backClampUp();
        moveDriveTrain(6000, 0.2);
    }
    else{
        moveDriveTrain(0.1, 12000);
        drive.setStandStill(DEFAULT_STANDSTILL);
        liftClampUp();
        backClampUp();
        drive.setPID(1);
        drive.swerve(backwardShortest, realYellowError, 0, 2, 70, 100);

        drive.setPID(2);
        drive.move(left, imuTarget(320), 1, 70);
        moveDriveTrain(-6000, 0.5);
        moveDriveTrain(-2000, 0.2);
        backClampDown();
        conveyormotor.move_voltage(12000);
        pros::delay(750);
        backClampUp();
        moveDriveTrain(6000, 0.2);
    }




    //Remove tasks used during the run and clear the vector
    AIOsystem.remove();
    runOnError.remove();
    onErrorVector.clear();
}

void leftShtick(){
    //Capture time at the beginning of the auto
    const uint32_t startTime = pros::millis();
    //Set the lift target to ring position before starting the lift task
    liftPID.setTarget(LIFT_LOWEST);
    //Initialize the automatic lift and ring control task and the run on error task
    AIOtimeOut = 15000;
    pros::Task AIOsystem(AIOsystem_fn);
    pros::Task runOnError(onError_fn);
    //Create a Triangle object to be used later in the auto
    Triangle tri;

    //Ensure that the lifts are set in the right position
    liftClampUp();
    backClampUp();
    stickUp();

    //Set ring speed for later use
    setConveyor(400);

    //Drive backward and clamp the right side yellow in the back
    drive.setPID(2);
    drive.setStandStill(3);
    #if USE_LIMIT_SWITCH
        const float driven = drive.yellowGrab(forward, 45, 70, 4, 100);
    #else
        drive.addErrorFunc(2, liftClampDown);
        drive.addErrorFunc(5, LAMBDA(drive.setMaxVelocity(40)));
        const float driven = 46 - drive.hardStop(forward, 45, 46, 1.8, 100);
    #endif

    //Complicated backup yellow fight
    float timeElapsed = float(pros::millis() - startTime)/1000;
    const float timeOut = ((ELIMS_AUTO)?(14.9 - timeElapsed):(11 - timeElapsed));
    const float realYellowError = drive.yellowFight(backward, driven + 5, driven, timeOut, ELIMS_AUTO);
    drive.setStandStill(DEFAULT_STANDSTILL);


    if(realYellowError <= 5){
        drive.setPID(2);
        drive.move(left, imuTarget(325), 1, 80);
        drive.setPID(1);

        drive.addErrorFunc(2, backClampDown);
        drive.move(backward, 15, 1, 60);
        startConveyor();
    }
    else{
        drive.setPID(2);
        liftClampUp();
        drive.move(backward, realYellowError + 2, 2, 100);
        drive.move(left, imuTarget(325), 1, 80);

        drive.setPID(1);
        drive.addErrorFunc(2, backClampDown);
        drive.move(backward, 15, 1, 60);
        startConveyor();
    }

    liftClampDown();

    //Remove tasks used during the run and clear the vector
    AIOsystem.remove();
    runOnError.remove();
    onErrorVector.clear();
}

void middle18(){//Capture time at the beginning of the auto
    const uint32_t startTime = pros::millis();
    //Set the lift target to ring position before starting the lift task
    liftPID.setCustomTarget(205);
    //Initialize the automatic lift and ring control task and the run on error task
    AIOtimeOut = 15000;
    pros::Task AIOsystem(AIOsystem_fn);
    pros::Task runOnError(onError_fn);
    //Create a Triangle object to be used later in the auto
    Triangle tri;

    //Set piston states at the start of the match
    liftClampUp();
    backClampUp();
    stickUp();

    //Stay in 18 by 18 by 18
    backClampUp();
    pros::delay(300);
    moveDriveTrain(-3000, 0.1);

    //Back into the goal and give the back clamp time to catch
    moveDriveVoltage(-6000);
    pros::delay(100);
    backClampDown();
    moveDriveVoltage(-3000);
    pros::delay(20);

    //Swerve and pick up middle mogo then clamp
    drive.setPID(4);
    drive.addErrorFunc(71, LAMBDA(drive.setMaxTurnVelocity(100)));
    drive.addErrorFunc(68, LAMBDA(conveyormotor.move_voltage(12000)));

    drive.addErrorFunc(20, LAMBDA(drive.setMaxVelocity(40)));
    drive.addErrorFunc(13, LAMBDA(liftPID.setTarget(LIFT_LOWEST)));
    drive.addErrorFunc(11, LAMBDA(drive.setMaxVelocity(30)));
    drive.swerve(forwardRight, 72, imuTarget(90 + 49), 2.15, 80, 50);
    liftClampDown();

    //Drive backwards with goal
    drive.setPID(2);
    #if !ELIMS_AUTO
        drive.addErrorFunc(20, backClampUp);
    #endif
    drive.move(backward, 55, 3, 100);




    //Remove tasks used during the run and clear the vector
    AIOsystem.remove();
    runOnError.remove();
    onErrorVector.clear(); 
}

void middle_swerve(){
    //Capture time at the beginning of the auto
    const uint32_t startTime = pros::millis();
    //Set the lift target to ring position before starting the lift task
    liftPID.setCustomTarget(205);
    //Initialize the automatic lift and ring control task and the run on error task
    AIOtimeOut = 15000;
    pros::Task AIOsystem(AIOsystem_fn);
    pros::Task runOnError(onError_fn);
    //Create a Triangle object to be used later in the auto
    Triangle tri;

    //Set piston states at the start of the match
    liftClampUp();
    backClampUp();
    stickUp();

    //Back into the goal and give the back clamp time to catch
    moveDriveVoltage(-6000);
    pros::delay(100);
    backClampDown();
    moveDriveVoltage(-3000);
    pros::delay(20);

    //Swerve and pick up middle mogo then clamp
    drive.setPID(4);
    drive.addErrorFunc(71, LAMBDA(drive.setMaxTurnVelocity(100)));
    drive.addErrorFunc(68, LAMBDA(conveyormotor.move_voltage(12000)));

    drive.addErrorFunc(20, LAMBDA(drive.setMaxVelocity(40)));
    drive.addErrorFunc(13, LAMBDA(liftPID.setTarget(LIFT_LOWEST)));
    drive.addErrorFunc(11, LAMBDA(drive.setMaxVelocity(30)));
    drive.swerve(forwardRight, 72, imuTarget(90 + 49), 2.15, 80, 50);
    liftClampDown();

    //Drive backwards with goal
    drive.setPID(2);
    #if !ELIMS_AUTO
        drive.addErrorFunc(20, backClampUp);
    #endif
    drive.move(backward, 55, 3, 100);




    //Remove tasks used during the run and clear the vector
    AIOsystem.remove();
    runOnError.remove();
    onErrorVector.clear(); 
}

void skills(){    
    //Capture time at the beginning of the auto
    const uint32_t startTime = pros::millis();

    //Set the lift target to ring position before starting the lift task
    liftPID.setTarget(LIFT_LOWEST);

    //Initialize the automatic lift and ring control task and the run on error task
    AIOtimeOut = 0;
    pros::Task AIOsystem(AIOsystem_fn);
    pros::Task runOnError(onError_fn);

    //Create a Triangle object to be used later in the auto
    Triangle tri;
    Triangle secondaryTri;

    //Tare the imus for more precise movement
    //TODO Check how this affects things (should be obvious)
    //imu.tare();

    //Initialize variables to make some movements simpler
    float turnVal;
    float driveVal;

    //Set ring speed for later use
    setConveyor(400);


    //Ensure that the pistons are set correctly and give back clamp time to catch
    moveDriveVoltage(-6000);
    pros::delay(100);
    liftClampUp();
    backClampDown();
    stickUp();
    moveDriveVoltage(-3000);
    pros::delay(20);

    //Swerve and pick up the mogo, then raise the lift and end over the platform
    drive.setPID(4);
    //drive.addErrorFunc(50, startConveyor);
    drive.addErrorFunc(50, LAMBDA(conveyormotor.move_voltage(12000)));
    drive.swerve(forwardShortest, 70, 100, 1.8, 60, 100);
    liftClampDown();
    drive.setCustomPID(12,100,10,400,15,200,100,0);
    liftPID.setTarget(LIFT_HIGHEST);
    drive.swerve(forwardRight, 36, imuTarget(150), 1.2, 60, 100);
    drive.setPID(4);
    drive.addErrorFunc(3, LAMBDA(drive.setMaxTurnVelocity(0)));
    drive.swerve(forwardLeft, 30, imuTarget(102), 1, 60, 70);

    //Capture the heading of the robot after droping the first mogo for later use
    const float firstDropAngle = imu.get_heading();

    //Balance the platform and drop the mogo
    liftPID.setTarget(LIFT_PLATFORM);
    pros::delay(1000);
    liftClampUp();
    
    if(firstDropAngle > 185){
        //Back up and turn to imu 180 in order to pick up next yellow
        drive.setStandStill(3);
        findTri(tri, 6, 90);
        drive.addErrorFunc(tri.hyp - 1, LAMBDA(liftPID.setTarget(LIFT_HIGHEST)));
        drive.move(backward, tri.hyp, 1, 100);
        liftPID.setCustomTarget(LIFT_RINGS_HEIGHT);
        drive.setPID(1);
        drive.move(right, imuTarget(180), 1, 100);

        //Drive and snatch second yellow
                        
        drive.setSlewMin(50);
        drive.setSlew(20);
        drive.addErrorFunc(30, LAMBDA(drive.setSlew(0)));
        drive.move(forward, 43.5 - tri.b, 2, 100);
        liftPID.setCustomTarget(201);
        drive.move(right, imuTarget(270), 1, 100);
        drive.addErrorFunc(15, LAMBDA(drive.setMaxVelocity(50)));
        drive.setSlewMin(50);
        drive.setSlew(10);
        drive.addErrorFunc(17, LAMBDA(drive.setSlew(0)));
        drive.addErrorFunc(10, LAMBDA(drive.setMaxVelocity(40)));
        drive.addErrorFunc(3, liftClampDown);
        drive.move(forward, 30, 2.8, 65);
    }

    else{
        //Back up and turn to imu 180 in order to pick up next yellow
        drive.setStandStill(3);
        findTri(tri, 6, 90);
        drive.addErrorFunc(tri.hyp - 1, LAMBDA(liftPID.setTarget(LIFT_HIGHEST)));
        drive.move(backward, tri.hyp, 1, 100);
        liftPID.setCustomTarget(LIFT_RINGS_HEIGHT);
        drive.setPID(1);
        drive.move(right, imuTarget(180), 1, 100);

        //Drive and snatch second yellow
                        
        drive.setSlewMin(50);
        drive.setSlew(20);
        drive.addErrorFunc(30, LAMBDA(drive.setSlew(0)));
        drive.move(forward, 43.5 - tri.b, 2, 100);
        liftPID.setCustomTarget(201);
        drive.move(right, imuTarget(270), 1, 100);
        drive.addErrorFunc(15, LAMBDA(drive.setMaxVelocity(50)));
        drive.setSlewMin(50);
        drive.setSlew(10);
        drive.addErrorFunc(17, LAMBDA(drive.setSlew(0)));
        drive.addErrorFunc(11, LAMBDA(drive.setMaxVelocity(40)));
        drive.addErrorFunc(3, liftClampDown);
        drive.move(forward, 31, 2.8, 65);
    }

    //Turn with the lift raised slightly then raise it fully while driving to the platform
    turnVal = 70;
    liftPID.setCustomTarget(LIFT_RINGS_WITH_MOGO_HEIGHT);
    drive.setPID(3);
    drive.move(right, imuTarget(turnVal), 1.2, 50);
    liftPID.setTarget(LIFT_OVER_PLATFORM);
    drive.setPID(7);
    drive.swerve(forwardShortest, 53, turnVal, 2, 80, 100);

    //Drop routine
    liftPID.setTarget(LIFT_PLATFORM);
    pros::delay(300);
    liftClampUp();
    pros::delay(300);
    onErrorVector.clear();


    /*******************************************POST SECOND YELLOW*******************************************/


    //Back away from the platform and turn to grab the second alliance
    findTri(tri, 15, 0);
    drive.addErrorFunc(tri.hyp - 1, LAMBDA(liftPID.setTarget(LIFT_HIGHEST)));
    liftPID.setTarget(LIFT_HIGHEST);
    drive.move(backward, tri.hyp, 3, 100);
    liftPID.setCustomTarget(LIFT_RINGS_HEIGHT);
    drive.setPID(1);
    drive.move(left, imuTarget(270), 1, 100);
    findTri(secondaryTri, 82 - tri.b, 270);// was 84.5

    //Drive to the other side of the field to pick up the second alliance then grab it
    drive.setPID(7);
    drive.swerve(forwardShortest, secondaryTri.hyp, 270, 3, 80, 100);
    liftPID.setTarget(LIFT_LOWEST);
    drive.setPID(1);
    drive.move(left, imuTarget(180), 1, 70);
    drive.addErrorFunc(1, liftClampDown);
    drive.move(forward, 24 + secondaryTri.b, 1, 70);
    liftClampDown();
    moveDriveVoltage(6000);
    pros::delay(100);

    //Back up and turn back to the platform to drop it
    turnVal = 66;
    findTri(tri, 7, 90+turnVal);
    liftPID.setTarget(LIFT_HIGHEST);
    drive.setPID(2);
    drive.move(backward, tri.hyp, 1, 100);
    drive.setPID(3);
    drive.setStandStill(DEFAULT_STANDSTILL);
    drive.move(left, imuTarget(turnVal), 1.2, 80);

    //Drive towards the platform and drop the mogo off
    drive.setPID(7);
    drive.setStandStill(3);
    drive.swerve(forwardShortest, 99-tri.b, turnVal, 5.5, 70, 100);
    liftPID.setTarget(LIFT_OVER_PLATFORM);
    pros::delay(300);
    liftClampUp();
    pros::delay(300);

    //Back up line up for the tall mogo
    findTri(tri, 42, 90);
    drive.setPID(1);
    drive.addErrorFunc(tri.hyp - 2, LAMBDA(liftPID.setTarget(LIFT_HIGHEST)));
    drive.addErrorFunc(tri.hyp - 10, LAMBDA(liftPID.setCustomTarget(202)));
    drive.move(backward, tri.hyp, 2, 80);

    //Turn to grab the tall mogo
    drive.setPID(1);
	drive.move(left, imuTarget(0), 1, 70);

	//Drive forward, clamp the goal, then lift
	drive.addErrorFunc(20, LAMBDA(liftPID.setTarget(LIFT_LOWEST)));
	drive.addErrorFunc(17, liftClampDown);
	drive.addErrorFunc(15, LAMBDA(liftPID.setTarget(LIFT_HIGHEST)));
	drive.move(forward, 26 - tri.b, 2, 50);

	//Turn and drop tall goal on the left
	drive.setCustomPID(0,100,0,400,0,200,0,0);
	drive.move(right, imuTarget(92), 1.3, 60);
	drive.setStandStill(3);
	drive.setPID(7);
	drive.addErrorFunc(20, LAMBDA(drive.setMaxVelocity(50)));
	drive.swerve(forwardShortest, 48, 90, 2, 80, 100);
	liftPID.setTarget(LIFT_PLATFORM);
	pros::delay(1300);
	liftClampUp();
	moveDriveTrain(-300, .2);
	liftPID.setTarget(LIFT_OVER_PLATFORM);
	/********************************************POST THIRD YELLOW********************************************/

    //Begin 180
    onErrorVector.clear();
    drive.setPID(2);
    drive.addErrorFunc(12, LAMBDA(liftPID.setTarget(LIFT_LOWEST)));
    drive.addErrorFunc(4, backClampUp);
	drive.move(backward, 17, 1, 100);
    pros::delay(100);
	drive.move(forward, 12, 1, 100);

    //Turn 180 degrees and pick the mogo up
	drive.setPID(1);
	drive.move(left, 180, 2, 100);
	drive.setPID(2);
    drive.addErrorFunc(3, liftClampDown);
	drive.move(forward, 15, 1, 70);

    //Turn and drop the mogo on the platform
    drive.setCustomPID(0,60,0,200,0,150,0,0);
	liftPID.setCustomTarget(LIFT_RINGS_WITH_MOGO_HEIGHT);
	drive.setStandStill(5);
	drive.move(left, 160, 1.25, 60);
	drive.setStandStill(DEFAULT_STANDSTILL);
	drive.setPID(1);
	liftPID.setTarget(LIFT_HIGHEST);
	pros::delay(500);

	drive.move(forward, 35, 1.8, 70);
	liftPID.setCustomTarget(LIFT_OVER_MOGO_HEIGHT);
    pros::delay(400);
    liftClampUp();


    /********************************************BLUES BEFORE PARK********************************************/

    onErrorVector.clear();
    drive.setStandStill(DEFAULT_STANDSTILL);
    drive.setPID(1);
    findTri(tri, 14, 90);
    liftPID.setTarget(LIFT_HIGHEST);
    drive.move(backward, tri.hyp, 1, 70);
    drive.setPID(2);
    drive.move(right, imuTarget(180), 1, 80);
    liftPID.setCustomTarget(LIFT_RINGS_HEIGHT);

	drive.setCustomPID(12,123,10,145,15,150,150,0);
    drive.hardStop(backward, 34 - tri.b, 40, 2, 100);
    moveDriveVoltage(-6000);
    pros::delay(400);
    backClampDown();
    moveDriveVoltage(-3000);
	pros::delay(50);

    drive.setPID(2);
    drive.move(shortest, 180, 1, 100);

    //Do the big long drive to the other side of the platform
    turnVal = 29;
	findTri(tri, 87.5, 180 - turnVal);
    drive.setPID(7);
    drive.swerve(forwardShortest, tri.hyp, 180, 4, 95, 100);

	drive.setPID(1);
	liftPID.setTarget(LIFT_LOWEST);
	drive.move(left, imuTarget(90 - turnVal), 1, 60);
    drive.addErrorFunc(10, LAMBDA(drive.setMaxVelocity(40)));
	drive.move(forward, 18.5, 2, 60);
	liftClampDown();
    pros::delay(100);

	//Back away with mogo and turn for parking/dropping
	drive.setStandStill(DEFAULT_STANDSTILL);
	liftPID.setTarget(LIFT_HIGHEST);
	findTri(secondaryTri, 9, 0);
	drive.move(backward, secondaryTri.hyp, 1, 100);
	

	//Use trig to ensure the final movements have more precision
    turnVal = 270;
    drive.setPID(3);
    drive.move(left, imuTarget(turnVal+5), 1.2, 60);
	findTri(tri, 115 - secondaryTri.b, turnVal);
	drive.setStandStill(3);
    drive.setPID(7);
    drive.swerve(forwardShortest, tri.hyp, turnVal, 3, 90, 100);


    /**************************************************PARK**************************************************/


    //Turn right to park
    liftPID.setTarget(LIFT_OVER_PLATFORM);
    drive.setPID(3);
    drive.move(right, imuTarget(3), 1, 73);
    moveDriveTrain(2000, 0.25);
    liftPID.setCustomTarget(LIFT_RINGS_HEIGHT);
    moveDriveTrain(2000, 0.1);
    pros::delay(300);

    
    //Finally good park code
    drive.platformBalance(0, 90, 65);
	liftPID.setTarget(LIFT_LOWEST);
	backClampUp();
	drive.setPID(3);
	drive.move(backward, 3, 2, 30);


	setBrakeMode(MOTOR_BRAKE_HOLD);
	moveDriveVelocity(0);
	pros::delay(2000);
	setBrakeMode(MOTOR_BRAKE_COAST);


    //Remove tasks used during the run and clear the vector
    AIOsystem.remove();
    runOnError.remove();
    onErrorVector.clear();
}


void empty(){}  
