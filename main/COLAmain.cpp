//  ***    ***   *        *      
// *   *  *   *  *       * *     
// *      *   *  *       ***     
// *   *  *   *  *      *   *    
//  ***    ***   *****  *   *    

/* 

    PRE-PROCESSING

*/

//PID
#include PID_v1 // This code uses the Arduino PID library. Let's not re-invent the wheel, eh?

//IGNITION
#ifndef Rocket_Ignition_h
#define Rocket_Ignition_h

//GPS
#include <Wire.h>  // Needed for I2C to GNSS
#include "GPS_I2C.h" // Gets the Altitude from the GPS sensor ***Still NEED to change from .ino file to either .cpp or .h***

//SERVO
#include "pch.h"
#include <iostream>
#include <cmath>

/* 

    FUNCTIONS

*/

//PID
double colaPIDr() { // Call this function every time you want the roll PID to update. 
	myPIDr.Compute(); // Run the roll PID update code
    	return gRoll // The function returns the roll gymbal angle.
}

double colaPIDp() { // Call this function every time you want the pitch PID to update. 
	myPIDp.Compute(); // Run the pitch PID update code
    	return gPitch // The function returns the pitch gymbal angle.
}

//SERVO
double gimbal2servo(double l1, double l2, double l3, double pos1, double pos2, double g_theta) {
	double point2x = cos((g_theta * (pi / 180)) - pi / 2) * l1;
	double point2y = sin((g_theta * (pi / 180)) - pi / 2) * l1;
	double d = sqrt(pow((point2x - pos1), 2) + pow((point2y - pos2), 2));
	double L = (pow(l2, 2) - pow(l3, 2) + pow(d, 2)) / (2 * d);
	double h = sqrt(pow(l2, 2) - pow(L, 2));
	double p3_x = (L / d) * (pos1 - point2x) + (h / d) * (pos2 - point2y) + point2x;
	double p3_y = (L / d) * (pos2 - point2y) - (h / d) * (pos1 - point2x) + point2y;
	double servoangle = (atan((p3_x - pos1) / (pos2 - p3_y))) * (180 / pi);
	return servoangle;
}

void processAccelGyro(double OuterGymbal,double InnerGymbal)
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



	servo_x.write(-Innergymbal + 90);
	servo_y.write(OuterGymbal + 90);
	delay(100);
}


/* 

    INITIALIZATION

*/

void main() {
	
    //GPS
    pinMode(13, OUTPUT); // Sets the digital pin 13 as output
    
    //PID
    double xdot = 0; // Velocity in the x axis (In practice theses are the velocities (either x or y) that the system derives from sensor data)
	double ydot = 0; // Velocity in the y axis
	double target = 0; // Desired velocity value, should always be zero
	double Kp = 1.1; // Proportional gain
	double Ki = 0.1; // Integral gain
	double Kd = 0.5; // Derivative gain
	double gRoll = 0; // Yaw gymbal angle (In practice these gymbal angle values should be fed into Brandon's conversion code to get a servo angle and then the servos should be driven to that angle)
	double gPitch = 0; // Pitch gymbal angle
	
	PID myPIDr(xdot,gRoll,target,Kp,Ki,Kd,DIRECT); // Initialize roll PID
	myPIDr.SetMode(AUTOMATIC); // Start roll PID
	PID myPIDp(ydot,gPitch,target,Kp,Ki,Kd,DIRECT); // Initialize pitch PID
	myPIDp.SetMode(AUTOMATIC); // Start pitch PID

    //SERVO
    using namespace std;
    double pi = 3.1415926535;

    double InnerGimbal = gimbal2servo(27.5, 37.9, 11.9, -37.65, -35, 10);
	// Inner Gimbal Deflection of 10 degrees
	cout << InnerGimbal << endl; // Inner Gimbal is Pitch

	double OuterGimbal = gimbal2servo(25.75, 36.3, 11.9, 36.2, -11, 10);
	// Outer Gimbal Deflection of 10 degrees
	cout << OuterGimbal << endl; // Outer Gimbal is Roll

    servo_x.attach(9); // attach the signal pin of servo to pin9 of arduino
	servo_y.attach(10);
	
    /* 

        LOOP

    */

	void loop() { // This code repeats until the arduino shuts off or explodes or something
		
        //PID
        gRoll = colaPIDr(); // Update the roll gymbal angle
		gPitch = colaPIDp(); // Update the pitch gymbal angle

        //GPS
        long altitude = gps_location(long altitude);
        int armed = gps_location(int armed);
        while (ignition_condition == 0 && armed == 1)
        {
            if (altitude <= 11000) // When COLA is 11 meters off the ground
            {
                digitalWrite(13, HIGH); // Sets the digital pin 13 on (Sends high Voltage to the igniter to light it)
                delay(1000); // Waits 2 seconds after the high voltage is on
                digitalWrite(13, LOW);  // Sets the digital pin 13 off (Turns off the high Voltage)
                ignition_condition = 1;
            }
        }

        //SERVO
        InnerGimbal = gimbal2servo(27.5, 37.9, 11.9, -37.65, -35, gPitch);
        OuterGimbal = gimbal2servo(25.75, 36.3, 11.9, 36.2, -11, gRoll);
        processAccelGyro(OuterGymbal,InnerGymbal);
	}
}