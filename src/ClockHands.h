#pragma once

#include "Arduino.h"
#include <AccelStepper.h>
#include <MultiStepper.h>


class ClockHands
{
  private:

    static AccelStepper* Motor[2];
    // AccelStepper Motor1;
    // static Stepper *MinuteMotor;

    static MultiStepper* PosController;
    // static RotateControl RotController;


    int StepPinH;
    int DirPinH;
    int StepPinM;
    int DirPinM;
    int SleepPin;
    int ResetPin;
    int OpticH;
    int OpticM;
    static int RevSteps;

    int defaultSpeed = 100;
    int maxSpeed = 500;

    bool sleepState = true;

    static bool calibrateMode[2];
    bool calibrateFailed[2] = {false, false};
    static int moveMode; //0 still, 1 target, 2 rotate
    static bool direction[2];
    static long target[2];
    static int moveStart[2];
    bool targetReached;


    static void OpticISR(int motor);
    static void hourISR();
    static void minuteISR();

    void resetPosition();


  public:
    ClockHands(const int inStepH, const int inDirH,
               const int inStepM, const int inDirM,
               const int inSleep, const int inReset,
               const int inOpticH, const int inOpticM,
               const int inSteps);

    void setSleep(bool sleep);
    void Calibrate();

    void MoveToTime(int32_t hour, int minute, int mode, int speed, bool dir);
    void MoveToTime(int hour, int minute);

    void RotateHands(int speed, bool dir);
    void RotateRealTime(bool dir);
    void stop();
    
    bool Tick();
};
