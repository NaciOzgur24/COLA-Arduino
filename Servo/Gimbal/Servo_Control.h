/*
COLA Arduino
Servo code reading the PID outputs and controls it
*/

// 5V power 333hz update signal
// 73% duty cycle
// 29% duty cycle

#ifndef Servo_Control_h
#define Servo_Control_h

#include "Wire.h"
#include <Servo.h>
#include "Gimbal2Servo_2.h"
#include "COLAPID.cpp" // ***Prob need to change the COLAPID.cpp to a .h file***


Servo servo_x; // Connect to the servo in x-axis
Servo servo_y;

int angle = 0;
int InnerGimbal = 0; // Inner Gimbal is Pitch
int OuterGimbal = 0; // Outer Gimbal is Roll

double PIDPitch = 0;
double PIDRoll = 0;

double pr[2]; // [pitch, roll]   pitch/roll container and gravity vector

// relative pr[x] usage based on sensor orientation when mounted, e.g. pr[PITCH]
#define PITCH 0 // defines the position within pr[x] variable for PITCH; may vary due to sensor orientation when mounted
#define ROLL 1	// defines the position within pr[x] variable for ROLL; may vary due to sensor orientation when mounted

void setup()
{
	servo_x.attach(9);						  // attach the signal pin of servo to pin9 of arduino
	servo_y.attach(10);						  // attach the signal pin of servo to pin10 of arduino
											  // THIS SECTION IS ONLY FOR THE INITIAL TESTING PHASE
	for (angle = -60; angle < 60; angle += 1) // command to move from -60 degrees to 60 degrees
	{
		servo_x.write(angle); // command to rotate the servo to the specified angle x-axis
		servo_y.write(angle);
		delay(500);
	}
	for (angle = 60; angle >= -60; angle -= 1) // command to move from 60 degrees to -60 degrees
	{
		servo_x.write(angle); //command to rotate the servo to the specified angle
		servo_y.write(angle); //command to rotate the servo to the specified angle
		delay(500);
	}
}

void loop()
{
	processAccelGyro();
}

void processAccelGyro()
{
	// Get INT_STATUS byte
	mpuIntStatus = mpu.getIntStatus();

	// Display Euler Angles in degrees
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetPitchRoll(pr, &q, &gravity);

	PIDPitch = pr[PITCH] * 180 / M_PI;
	PIDRoll = pr[ROLL] * 180 / M_PI;
	
	double PIDPitch = colaPIDp(double gPitch);
	double PIDRoll = colaPIDr(double gRoll);

	servo_x.write(-PIDPitch + 90);
	servo_y.write(PIDRoll + 90);
	delay(500);
}

#endif