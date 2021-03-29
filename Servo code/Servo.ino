/*
COLA Arduino
Servo code reading the PID outputs and controls it
*/

#include "Wire.h"
#include <Servo.h>
#include "Gimbal2Servo_2.cpp" // The Gimbal to Servo code ***Pretty sure we need to call a .h file instead of .cpp***

Servo servo_x; // Connect to the servo in x-axis
Servo servo_y; // y-axis

int angle = 0;

//float PIDYaw = 0;
double PIDPitch = 0;
double PIDRoll = 0;

double ypr[2]; // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// relative ypr[x] usage based on sensor orientation when mounted, e.g. ypr[PITCH]
//#define YAW   0     // defines the position within ypr[x] variable for YAW; may vary due to sensor orientation when mounted
#define PITCH   0     // defines the position within ypr[x] variable for PITCH; may vary due to sensor orientation when mounted
#define ROLL  1     // defines the position within ypr[x] variable for ROLL; may vary due to sensor orientation when mounted

void setup()
{
	servo_x.attach(9); // attach the signal pin of servo to pin9 of arduino
	servo_y.attach(10); // attach the signal pin of servo to pin10 of arduino
  // THIS SECTION IS ONLY FOR THE INITIAL TESTING PHASE
	for (angle = -60; angle < 60; angle += 1)    // command to move from -60 degrees to 60 degrees 
	{
		servo_x.write(angle); // command to rotate the servo to the specified angle x-axis
		servo_y.write(angle); // y-axis
		delay(500);
	}
	delay(100);
	for (angle = 60; angle >= -60; angle -= 1) // command to move from 60 degrees to -60 degrees 
	{
		servo_x.write(angle); //command to rotate the servo to the specified angle
		servo_y.write(angle); //command to rotate the servo to the specified angle
		delay(500);
	}
	delay(100);
}

void loop()
{
	processAccelGyro();
}

void processAccelGyro()
{

	/*/ Get INT_STATUS byte
	mpuIntStatus = mpu.getIntStatus();
	// display Euler Angles in degrees
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

	PIDYaw  = ypr[YAW] * 180 / M_PI;
	PIDPitch = ypr[PITCH] * 180 / M_PI;
	PIDRoll = ypr[ROLL] * 180 / M_PI;
	*/

	PIDPitch = colaPIDp();
	PIDRoll = colaPIDr();

	servo_x.write(-PIDPitch + 90);
	servo_y.write(PIDRoll + 90);
	delay(100);

}
}  // processAccelGyro()
