
#ifndef _COLA_PID_v2_H
#define _COLA_PID_v2_H

#include "PID_v2.h"


int Val, Target;
//int prevVal;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,1,0,0, DIRECT);

void pid_setup()
{
    double imu_roll = imu_roll(); 
    Input = map(imu_roll, -1440, 1440, 30, 150);
    Setpoint = 127.5;

    //turn the PID on
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-20, 20);

}

void pid_loop()
{
    double imu_roll = imu_roll(); 
    Input = map(imu_roll, -1440, 1440, 30, 150);

    myPID.Compute();

    Serial.print(Output);

    delay(100);
}

#endif // _COLA_PID_v2_H
