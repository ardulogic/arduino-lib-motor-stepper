/* 
 * 
 * MotorStepper Library 2019
 * 
 */

#ifndef MotorStepper_h
#define MotorStepper_h

#include "Arduino.h"
#include "MotorWaypoints.h"

// #define DEBUG_MOTOR_MODE_WAYPOINT
// #define STEPPER_DIRECTION_INVERT

#define REVERSE false
#define FORWARD true

#define MODE_CONTINOUS 0
#define MODE_WAYPOINT 1

#define STEPPER_FAST_OVER_PRECISE 0

class MotorStepper {
public:
    MotorStepper(int PIN_DIR, int PIN_CLK);
    void Init();

    // Volatile getters
    static volatile bool MotorStepper::Direction();
    static volatile unsigned int MotorStepper::Frequency();
    static volatile MotorWaypoints* Waypoints();
    
    // Timer Functions
    static byte tCalcPrescaler(int freq);
    static void tSetPrescaler(byte p);
    static void tStop();

    // Essential Functions
    static void Run();
    static void Stop();
    static bool IsRunning();   
    static void SetMode(uint8_t m);    
    static void SetFrequency(int freq);
    static void SetDirection(bool dir);
    static void Step();
    
private:
    int PIN_CLK;
    int PIN_DIR;

    void _tInit();

};

#endif