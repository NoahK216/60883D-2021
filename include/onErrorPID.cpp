#include "PID.cpp"
#include "robotClasses.cpp"

#define LAMBDA(func) [](){func;}


struct errorFuncTuple{
   std::function<void()> func;
   float onError;
   bool called;
   errorFuncTuple(std::function<void()> func, float onError, bool called):
   func(func),onError(onError), called(called){}
};

std::vector<errorFuncTuple> onErrorVector;

void Drive::addErrorFunc(float onError, void input()){
    //TODO add easy integration with turning (although not that useful)
    onErrorVector.emplace_back(errorFuncTuple(input, inchToTick(onError), false));
}

pros::Mutex onErrorMutex;
void onError_fn(void* param){
  std::uint32_t startTime = pros::millis();
  while(true){
  onErrorMutex.take();
  if(drive.getPIDStatus()){
    auto iter = onErrorVector.begin();
    while(iter != onErrorVector.end()){
      if (!iter->called && (iter->onError >= drive.getError())){
        iter->func();
        iter->called = true;
      }
      else{iter++;}
      }
    }
    onErrorMutex.give();
    pros::Task::delay_until(&startTime, 10);  
  }
}

//Functions used essentially as lambdas
void backClampDown(){pneumatics.setBackClamp(HIGH);}
void backClampUp(){pneumatics.setBackClamp(LOW);}
void liftClampDown(){pneumatics.setLiftClamp(LOW);}
void liftClampUp(){pneumatics.setLiftClamp(HIGH);}
void stickDown(){pneumatics.setStick(HIGH);}
void stickUp(){pneumatics.setStick(LOW);}