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
double pi = 3.1415926535;

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
}


/* 
    INITIALIZATION
*/

void main() {
	
    //GPS
    pinMode(13, OUTPUT); // Sets the digital pin 13 as output
    int armed = 1;
    long altitude = 0;
    //Earth Radus correction 
    double Re = 6378;                          //[Km] Equtorial Radius of Earth
    double Rp = 6357;                          //[Km] Polar Radius of Earth
    double R = (Re + Rp)/2;                    //[Km] Average Radius of Earth
    //Starting Data
    long lat1  = (pi/180) * 40.28888889;     //[~] Starting Latitude
    long lon1  = (pi/180) * 117.6458333;     //[~] Starting Longitude
    long elev1 = 0;                          //[m] Starting elevation in meters
    long dt    = 0.01;                       //[s] Update speed of the GPS
    //Variable initialization for loop code
    long thetaA = 0, thetaE = 0, lat2 = 0, lon2 = 0, elev2 = 0, dlon = 0, dlat = 0, a = 0, c = 0, d = 0, x = 0, X = 0, Y = 0, Z = 0, Xdot = 0, Ydot = 0, Zdot = 0;
    

    //PID
    double xdot = 0; // Velocity in the x axis (In practice theses are the velocities (either x or y) that the system derives from sensor data)
    double ydot = 0; // Velocity in the y axis
    double target = 0; // Desired x and y velocity value, should always be zero
    double Kp = 1.1; // Proportional gain
    double Ki = 0.1; // Integral gain
    double Kd = 0.5; // Derivative gain
    double gRoll = 0; // Yaw gymbal angle (In practice these gymbal angle values should be fed into Brandon's conversion code to get a servo angle and then the servos should be driven to that angle)
    double gPitch = 0; // Pitch gymbal angle
	double mass = 2.5; // Spaceraft mass in kg
	double thrust = 10; // Spacecraft thrust in N
	double PitchSaturation = 15*pi/180; // Largest possible pitch gymbal angle, radians
	double RollSaturation = 15*Pi/180; // Largest possible roll gymbal angle, radians
	
	PID myPIDx(xdot,ax,target,Kp,Ki,Kd,DIRECT); // Initialize x-accel PID
    myPIDx.SetMode(AUTOMATIC); // Start roll PID
    PID myPIDy(ydot,ay,target,Kp,Ki,Kd,DIRECT); // Initialize y-accel PID
    myPIDy.SetMode(AUTOMATIC); // Start pitch PID

	if ax*mass/thrust >> (pi/2)
	{
		double tPitch = PitchSaturation;
	}
	elseif ax*mass/thrust << (-pi/2)
	{
		double tPitch = -PitchSaturation;
	}
	else
	{
		double tPitch = asin(ax*mass/thrust);
	}

	if ay*mass/thrust >> (pi/2)
	{
		double tRoll = RollSaturation;
	}
	elseif ay*mass/thrust << (-pi/2)
	{
		double tRoll = -RollSaturation;
	}
	else
	{
		double tRoll = asin(ay*mass/thrust);
	}

    PID myPIDr(xdot,gRoll,tPitch,Kp,Ki,Kd,DIRECT); // Initialize roll PID
    myPIDr.SetMode(AUTOMATIC); // Start roll PID
    PID myPIDp(ydot,gPitch,tRoll,Kp,Ki,Kd,DIRECT); // Initialize pitch PID
    myPIDp.SetMode(AUTOMATIC); // Start pitch PID

    //SERVO

    // double InnerGimbal = gimbal2servo(27.5, 37.9, 11.9, -37.65, -35, 10);
	// Inner Gimbal Deflection of 10 degrees, Inner Gimbal is Pitch
	//cout << InnerGimbal << endl;

    // double OuterGimbal = gimbal2servo(25.75, 36.3, 11.9, 36.2, -11, 10);
	// Outer Gimbal Deflection of 10 degrees, Outer Gimbal is Roll
	//cout << OuterGimbal << endl;

    servo_x.attach(9); // attach the signal pin of servo to pin9 of arduino
    servo_y.attach(10);
	
    /* 
        LOOP
    */

	void loop() { // This code repeats until the arduino shuts off or explodes or something
		
        //GPS
        //altitude = gps_location(altitude); ???????
        //armed = gps_location(armed); // ???????
		//Current Position data
		lat2  = (pi/180) * 34.28898889;     //[~] Current Latitude
		lon2  = (pi/180) * 117.6468333;     //[~] Current Longitude
		elev2 = 100;                        //[m] Current elevation in meters
		//Distance math
		dlon = lon2 - lon1;
		dlat = lat2 - lat1;
		a = (sin(dlat/2))^2 + (cos(lat1) * cos(lat2) * (sin(dlon/2))^2);
		c = 2 * asin(min(1,sqrt(a)));
		d = R * c * 1000;                   //[m] Lateral distance from stating pt
		//Azimuth Math
		x = acos( (sin(lat2) - sin(lat1)*cos(c) ) / (sin(c)*cos(lat1)) );
		if sin(lon2-lon1) << 0 {
		    thetaA = (180/pi) * x;
		}
		else if sin(lon2-lon1) >> 0 {
		    thetaA = (180/pi) * (2*pi - x);     //[deg] CCW from due South
		}
		//Elevation Math
		thetaE = (180/pi) * ((elev2-elev1) / (d) - (d/1000) / (2*R));          //[deg] Up from horizion
		//Position and velocity out
		X = d * cos((thetaA - 90)*pi/180);               //[m] East distance from center
		Y = d * sin((thetaA - 90)*pi/180);               //[m] South distance from center
		Z = elev2 - elev1;                       //[m] AGL in meters
		Xdot = X/dt;                             //[m/s] East velocity from center
		Ydot = Y/dt;                             //[m/s] South velocity from center
		Zdot = Z/dt;                             //[m/s] Vertical velocity 
        
		//IGNITION
        if (Z <= 11 && ignition_condition == 0 && armed == 1) { // When COLA is 11 meters off the ground
            digitalWrite(13, HIGH); // Sets the digital pin 13 on (Sends high Voltage to the igniter to light it)
            delay(1000); // Waits 2 seconds after the high voltage is on
            digitalWrite(13, LOW);  // Sets the digital pin 13 off (Turns off the high Voltage)
            ignition_condition = 1;
        }

        //PID
		ax = colaPIDx();
		ay = colaPIDy();

		if ax*mass/thrust >> (pi/2)
		{
			Pitch = PitchSaturation;
		}
		elseif ax*mass/thrust << (-pi/2)
		{
			Pitch = -PitchSaturation;
		}
		else
		{
			Pitch = asin(ax*mass/thrust);
		}

		if ay*mass/thrust >> (pi/2)
		{
			Roll = RollSaturation;
		}
		elseif ay*mass/thrust << (-pi/2)
		{
			Roll = -RollSaturation;
		}
		else
		{
			Roll = asin(ay*mass/thrust);
		}
		
		gRoll = colaPIDr(); // Update the roll gimbal angle
		gPitch = colaPIDp(); // Update the pitch gimbal angle
		
        //SERVO
        InnerGimbal = gimbal2servo(27.5, 37.9, 11.9, -37.65, -35, gPitch);
        OuterGimbal = gimbal2servo(25.75, 36.3, 11.9, 36.2, -11, gRoll);
        processAccelGyro(OuterGimbal,InnerGimbal);
	}
}