#include "Arduino.h"
#include "ClockHands.h"

AccelStepper* ClockHands::Motor[2];
// AccelStepper* ClockHands::Motor1;
int ClockHands::RevSteps;
bool ClockHands::calibrateMode[] = {0,0};
bool ClockHands::direction[2] = {1, 1};
int ClockHands::moveMode = 0;
long ClockHands::target[2] = {0,0};
int ClockHands::moveStart[2] = {0,0};
// RotateControl ClockHands::RotController;
MultiStepper* ClockHands::PosController;
// Stepper* ClockHands::MinuteMotor;

int abso(int input){
  return (input<0)?-input:input;
}
//
// int sign(int input){
//   return (input<0)?-1:1;
// }

void ClockHands::OpticISR(int motor)
{
    // Serial.print("ISR ");
    // Serial.println(motor);
    if(calibrateMode[motor]){
      Motor[motor]->setCurrentPosition(0);
      calibrateMode[motor] = false;
    }else if(direction[motor] & (moveMode>0) & (abso(moveStart[motor]-Motor[motor]->currentPosition())>20))
    {
      long motorpos = Motor[motor]->currentPosition();

      if(moveMode == 1)
      { //target mode
        // Serial.println("target");
        // Serial.println(Motor[motor]->currentPosition());
        // int32_t posMult = ((abs(motorpos)+(RevSteps/2)) / RevSteps);
        // Motors[motor]->setPosition(sign(motorpos) * posMult * RevSteps);
        int posMult = ((motorpos)+(RevSteps/2)) / RevSteps;
        // target[motor] = posMult * RevSteps;

        Motor[motor]->setCurrentPosition(posMult * RevSteps);
        PosController->moveTo(target);

        moveStart[motor] = Motor[motor]->currentPosition();

        // if(posMult > 0)
        // {
        //   target[motor]=target[motor]-(sign(motorpos)*RevSteps);
        //   Motors[motor]->setTargetAbs(target[motor]);
        // }


        // Serial.println(Motors[motor]->getPosition());
      }else if(moveMode == 2);
      { //Rotate mode set position to 0
        // float speed = Motor[motor]->speed();
        // Motor[motor]->setCurrentPosition(0);
        // Serial.println(Motor[motor]->speed());
        // Motor[motor]->setSpeed(speed);
        // moveStart[motor] = Motor[motor]->currentPosition();
      }
    }
  }





void ClockHands::hourISR(){ClockHands::OpticISR(0);}
void ClockHands::minuteISR(){ClockHands::OpticISR(1);}

void ClockHands::resetPosition()
{
  //Rest Current Postion to first rotation range
  Motor[0]->setCurrentPosition(Motor[0]->currentPosition()%RevSteps);
  if (Motor[0]->currentPosition() < 0){
    Motor[0]->setCurrentPosition(RevSteps + Motor[0]->currentPosition());
  }
  Motor[1]->setCurrentPosition(Motor[1]->currentPosition()%RevSteps);
  if (Motor[1]->currentPosition() < 1){
    Motor[1]->setCurrentPosition(RevSteps + Motor[1]->currentPosition());
  }
}

void ClockHands::stop(){
  //stop any motion that might be happening
  Motor[0]->stop();
  Motor[1]->stop();
  moveMode = 0;
}

ClockHands::ClockHands(const int inStepH, const int inDirH,
                       const int inStepM, const int inDirM,
                       const int inSleep, const int inReset,
                       const int inOpticH, const int inOpticM,
                       const int inSteps)
{



  StepPinH = inStepH;
  DirPinH = inDirH;
  StepPinM = inStepM;
  DirPinM = inDirM;
  SleepPin = inSleep;
  ResetPin = inReset;
  OpticH = inOpticH;
  OpticM = inOpticM;
  RevSteps = inSteps;




  Motor[0] = new AccelStepper(1, StepPinH, DirPinH);
  Motor[1] = new AccelStepper(1, StepPinM, DirPinM);
  PosController = new MultiStepper();


  Motor[0]->setMaxSpeed(100);
  Motor[1]->setMaxSpeed(100);

  PosController->addStepper(*Motor[0]);
  PosController->addStepper(*Motor[1]);

  pinMode(SleepPin, OUTPUT);
  pinMode(ResetPin, OUTPUT);
  digitalWrite(SleepPin, !sleepState);
  digitalWrite(ResetPin, !sleepState);

  pinMode(OpticH, INPUT);
  pinMode(OpticM, INPUT);
  attachInterrupt(OpticH, ClockHands::hourISR, FALLING);
  attachInterrupt(OpticM, ClockHands::minuteISR, FALLING);


}

void ClockHands::setSleep(bool sleep)
{
  sleepState = sleep;
  digitalWrite(SleepPin, !sleepState);
  digitalWrite(ResetPin, !sleepState);
}


void ClockHands::Calibrate()
{
  unsigned long calibrateTime = millis();
  //set clockwise constant motion
  Motor[0]->setMaxSpeed(defaultSpeed);
  Motor[0]->setSpeed(100);
  Motor[1]->setMaxSpeed(defaultSpeed);
  Motor[1]->setSpeed(100);

  //save calibrate mode
  calibrateMode[0] = true;
  calibrateMode[1] = true;
  //Motor Tick Loop
  while((calibrateMode[0] || calibrateMode[1]) & ((millis()-calibrateTime)<10000))
  {
    Motor[0]->runSpeed();
    Motor[1]->runSpeed();
  }
  //Currenlty a failed calibration does nothing
  calibrateFailed[0]=!calibrateMode[0];
  calibrateFailed[1]=!calibrateMode[1];
}

void ClockHands::MoveToTime(int32_t hour, int minute, int mode, int speed, bool dir)
{
  //Rest Current Postion to first rotation range
  ClockHands::stop();
  ClockHands::resetPosition();
  // Motor[0]->setCurrentPosition(Motor[0]->currentPosition()%RevSteps);
  // if (Motor[0]->currentPosition() < 0){
  //   Motor[0]->setCurrentPosition(RevSteps + Motor[0]->currentPosition());
  // }
  // Motor[1]->setCurrentPosition(Motor[1]->currentPosition()%RevSteps);
  // if (Motor[1]->currentPosition() < 1){
  //   Motor[1]->setCurrentPosition(RevSteps + Motor[1]->currentPosition());
  // }

  //Calculate new positions
  //!!!Current hard coded for 400 steps/rev
  int HourPosition = (hour * 33.333333) + (minute * 0.555556);
  int MinutePosition = minute * 6.666667;

  //Remove extra steps over RevSteps
  HourPosition = HourPosition - ((HourPosition/RevSteps)*RevSteps);
  MinutePosition = MinutePosition - ((MinutePosition/RevSteps)*RevSteps);

  //dir true =  clockwise rotation
  //mode 0 = direct move
  //mode 1 = progressive move

  if(dir)
  {//Forward Motion
    if(HourPosition < Motor[0]->currentPosition())
    {
      HourPosition = HourPosition + RevSteps;
    }
    if(MinutePosition < Motor[1]->currentPosition())
    {
      MinutePosition = MinutePosition + RevSteps;
    }
    if(mode)
    {//progressive mode
      int hourMult = (HourPosition - Motor[0]->currentPosition())/33.333333;
      MinutePosition = MinutePosition + (hourMult*RevSteps);
    }
  }else
  {//Reverse Motion
    if(HourPosition > Motor[0]->currentPosition())
    {
      HourPosition = HourPosition - RevSteps;
    }
    if(MinutePosition > Motor[1]->currentPosition())
    {
      MinutePosition = MinutePosition - RevSteps;
    }
    if(mode)
    {//progressive mode
      int hourMult = (Motor[0]->currentPosition()-HourPosition)/33.333333;
      MinutePosition = MinutePosition - (hourMult*RevSteps);
    }
  }

  //set class variables
  direction[0]=dir;
  direction[1]=dir;

  target[0] = HourPosition;
  target[1] = MinutePosition;

  //Calculate speed
  float newSpeed = (defaultSpeed * speed) / 100;
  newSpeed = (newSpeed < 1 ) ? 1 : newSpeed;
  newSpeed = (newSpeed > 500 ) ? 500 : newSpeed;
  Motor[0]->setMaxSpeed(newSpeed);
  Motor[1]->setMaxSpeed(newSpeed);

  //Activate the Move
  moveMode = 1;
  PosController->moveTo(target);

  //save starting position for ISR
  moveStart[0] = Motor[0]->currentPosition();
  moveStart[1] = Motor[1]->currentPosition();
}

void ClockHands::MoveToTime(int hour, int minute)
{
  ClockHands::MoveToTime(hour, minute, 0, 0, true);
}

void ClockHands::RotateHands(int speed, bool dir)
{
  //stop any motion that might be happening
  ClockHands::stop();
  ClockHands::resetPosition();

  //calculate speeds
  float newSpeed = (defaultSpeed * speed) / 100;
  newSpeed = (newSpeed < 1 ) ? 1 : newSpeed;
  newSpeed = (newSpeed > maxSpeed ) ? maxSpeed : newSpeed;
  newSpeed = (dir)?newSpeed: -1*newSpeed;
  float hourSpeed = newSpeed / 12;

  //set class variables
  direction[0]=dir;
  direction[1]=dir;

  //Activate the Move
  Motor[0]->setMaxSpeed(maxSpeed);
  Motor[0]->setSpeed(hourSpeed);
  Motor[1]->setMaxSpeed(maxSpeed);
  Motor[1]->setSpeed(newSpeed);

  moveMode = 2;

  //save starting position for ISR
  moveStart[0] = Motor[0]->currentPosition();
  moveStart[1] = Motor[1]->currentPosition();

}

void ClockHands::RotateRealTime(bool dir){
  //stop any motion that might be happening
  ClockHands::stop();
  ClockHands::resetPosition();

  //calculate speeds
  float newSpeed = float(1)/float(9);
  newSpeed = (dir)?newSpeed: -1*newSpeed;
  float hourSpeed = newSpeed / 12;

  //set class variables
  direction[0]=dir;
  direction[1]=dir;

  //Activate the Move
  Motor[0]->setMaxSpeed(maxSpeed);
  Motor[0]->setSpeed(hourSpeed);
  Motor[1]->setMaxSpeed(maxSpeed);
  Motor[1]->setSpeed(newSpeed);

  moveMode = 2;

  //save starting position for ISR
  moveStart[0] = Motor[0]->currentPosition();
  moveStart[1] = Motor[1]->currentPosition();
}

bool ClockHands::Tick()
{
  switch(moveMode){
    case 1:  //Target Mode
      if(!PosController->run())
      {
          targetReached = 1;
          moveMode = 0;
          return 0;
      }
      break;
    case 2: //Continous rotation
      Motor[0]->runSpeed();
      Motor[1]->runSpeed();
      break;
  }
  return 1;
}
