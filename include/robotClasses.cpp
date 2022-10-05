#pragma once
#include "auton_functions.cpp"


class PneumaticControl{
public:
    bool backPistonToggle = LOW;
    bool pistonLiftToggle = HIGH;
    bool stickPistonToggle = LOW;

    void setBackClamp(bool state);
    void toggleBackClamp();

    void setLiftClamp(bool state);
    void toggleLiftClamp();

    void setStick(bool state);
    void toggleStick();

}; PneumaticControl pneumatics;

void PneumaticControl::toggleBackClamp(){
    backPistonToggle =! backPistonToggle;
	backClamp.set_value(backPistonToggle);
}

void PneumaticControl::setBackClamp(bool state){
    backPistonToggle = state;
    backClamp.set_value(state);
}

void PneumaticControl::toggleLiftClamp(){
    pistonLiftToggle = !pistonLiftToggle;
    liftClamp.set_value(pistonLiftToggle);
}

void PneumaticControl::setLiftClamp(bool state){
    pistonLiftToggle = state;   
    liftClamp.set_value(state);
}

void PneumaticControl::toggleStick(){
    stickPistonToggle =! stickPistonToggle;
	stick.set_value(stickPistonToggle);
}

void PneumaticControl::setStick(bool state){
    stickPistonToggle = state;
    stick.set_value(state);
}

/**********************************************************************************************************/
#define LIFT_LOWEST 0
#define LIFT_PLATFORM 1
#define LIFT_OVER_PLATFORM 2
#define LIFT_HIGHEST 3
#define LIFT_RINGS_HEIGHT 210
#define LIFT_RINGS_WITH_MOGO_HEIGHT 225
#define LIFT_OVER_MOGO_HEIGHT 279


class Lift{
private:
    //Initialize PID Values
    const float kP = 550;
    const float kI = 10;
    const float kD = 0;
    const float intergralActive = 2;
    const float intergralLimit = 40;

    //Variables that need to be read after each PID scope is destroyed
    float error;
    float lastError;
    float intergral;
    
public:
    uint8_t targetIndicator = 0;
    float target = liftTargets[0];
    void PID();
    void waitUntilTargetReached(float timeOut);
    void setTarget(int n);
    void setCustomTarget(float target);

};Lift liftPID;


void Lift::setTarget(int n){
    targetIndicator = mapToRange(n, LIFTVALSIZE-1, 0);
    target = liftTargets[targetIndicator];
}

void Lift::setCustomTarget(float customTarget){
    target = customTarget;
}

void Lift::waitUntilTargetReached(float timeOut){
    const uint32_t endTime = pros::millis() + timeOut*1000;
    while(pros::millis() < endTime && fabs(error) < 3){
        pros::delay(10);
    }
}

void Lift::PID(){
    error = target - (360 - (float(liftPot.get_angle())/100));
    float proportion = error;

    if(fabs(error) <= intergralActive){intergral = error;}
    else{intergral = 0;}
    intergral = mapToRange(intergral, intergralLimit, -intergralLimit);

    float derivative = error - lastError;
    lastError = error;
    if(error == 0){derivative = 0;}

    int finalVolt = kP*proportion + kI*intergral + kD*derivative;

    //Set finalVolt to range
    finalVolt = mapToRange(finalVolt, MAX_VOLTAGE, MIN_VOLTAGE);

    //master.print(2,0,"%.2f, %.0f                ", error, target);

    //Set final lift speeds
    lift.move_voltage(finalVolt);
}


/**********************************************************************************************************/
#define JAM_PROTECTION_ACTIVE true
#define JAM_PROTECTION_INACTIVE false


pros::Mutex ringControlMutex;


class ConveyorControl{
private:
    const uint16_t jamCycleThreshold = 20;
    const uint16_t dejamThreshold = 25;

    int16_t conveyorVel;
    int16_t jamThresh;
    int32_t jamSpeed;
   
    uint8_t intakeFlag = 0;
    uint16_t jamCycles;
    uint16_t reverseCycles;
    uint16_t deadCycles;
    
    bool jamProtection;
    bool isRunning = false;
    bool notifyOff = false;
    bool lastJamDead = true;
    

public:
    ConveyorControl(){
        setJamThresh(350);
        setJamSpeed(-400);
        setJamProtection(JAM_PROTECTION_ACTIVE);
    }

    ~ConveyorControl(){
        conveyormotor.move_voltage(0);
    }

    void startConveyor();
    void stopConveyor();
    void setConveyor(int16_t velocity);
    void setJamThresh(int16_t velocity);
    void setJamSpeed(int16_t velocity);
    void setJamProtection(bool state);

    void run();

    int32_t conveyorSpeed;


}; ConveyorControl conveyor;



void ConveyorControl::startConveyor(){
    isRunning = true;
}

void ConveyorControl::stopConveyor(){
    notifyOff = true;
}

void ConveyorControl::setConveyor(int16_t velocity){
    //Input will range from -400 to 400, voltage ranges from -12000 to 12000
    //12000 / 400 = 30
    conveyorSpeed = velocity*30;
    conveyorVel = velocity/2;
}

void ConveyorControl::setJamThresh(int16_t velocity){
    //Input will range from -400 to 400, getVelocity returns between -200 and 200
    jamThresh = velocity/2;
}

void ConveyorControl::setJamSpeed(int16_t velocity){
    //Input will range from -400 to 400, voltage ranges from -12000 to 12000
    //12000 / 400 = 30
    jamSpeed = velocity*30;
}

void ConveyorControl::setJamProtection(bool state){
    jamProtection = state;
}


void ConveyorControl::run(){
    ringControlMutex.take();
    if(notifyOff){
        notifyOff = false;
        isRunning = false;
        conveyormotor.move_voltage(0);
    }
    if(isRunning){
    master.print(2,0,"%i, %i, %i", jamCycles, deadCycles, reverseCycles);
    switch(intakeFlag){
        //Intake is not jammed or in the process of unjamming
        case 0:
        deadCycles = 0;
        reverseCycles = 0;
        conveyormotor.move_voltage(conveyorSpeed);
        if((conveyorVel - conveyormotor.get_actual_velocity()) > jamThresh){jamCycles++;}
        else{lastJamDead = 0; jamCycles = 0;}

        if(jamCycles >= jamCycleThreshold){intakeFlag = 1 + lastJamDead;}
        break;

        //Stop running the conveyor to see if rings fall naturally and unjam
        case 1:
        jamCycles = 0;
        conveyormotor.move_voltage(0);
        deadCycles++;
        if(deadCycles >= dejamThreshold){deadCycles = 0; intakeFlag = 0; lastJamDead = 1;}
        break;

        //Stopping the conveyor didn't fix the jam and rings will be run backward
        case 2:
        jamCycles = 0;
        conveyormotor.move_voltage(jamSpeed);
        reverseCycles++;
        if(reverseCycles >= dejamThreshold){reverseCycles = 0; intakeFlag = 0; lastJamDead = 0;}
        break;
    }
    }
    ringControlMutex.give();
}

//Set the conveyors speed to run at when operating
void setConveyor(int16_t velocity){conveyor.setConveyor(velocity);}
//Begin running the conveyor at the previously specified speed
void startConveyor(){conveyor.startConveyor();}
//Stop running the conveyor (Does not continously run at zero, rather set speed to zero then quits)
void stopConveyor(){conveyor.stopConveyor();}


/**********************************************************************************************************/

//Test what happens when a thread dies and no more code is executed, could be a problem, could be not a problem
float AIOtimeOut;
void AIOsystem_fn(void *param){
    uint32_t startTime = pros::millis();
    const uint32_t timeOut = startTime + AIOtimeOut;
    if(AIOtimeOut != 0){
    while(timeOut > pros::millis()){
        liftPID.PID();
        conveyor.run();
        pros::Task::delay_until(&startTime, 20);
    }
    }
    else{
    while(true){
        liftPID.PID();
        conveyor.run();
        pros::Task::delay_until(&startTime, 20);
    }
    }
}
