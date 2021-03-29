// Gimbal2Servo.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <iostream>
#include <cmath>

// Inner Gimbal is Pitch
// Outer Gimbal is Roll

using namespace std;

double pi = 3.1415926535;

// Inner Gimbal Dimensions (mm)
// l1 = 27.5;% distance from gimbal point to connecting rod mount
// l2 = 37.9;% connecting rod length
// l3 = 11.9;% servo control horn radius
// pos1 = -37.65;% x component of gimbal pivit point position
// pos2 = -35;% y component of gimbal pivit point position

// Outer Gimbal Dimensions (mm)
// l1 = 25.75;% distance from gimbal point to connecting rod mount
// l2 = 36.3;% connecting rod length
// l3 = 11.9;% servo control horn radius
// pos1 = 36.2;% x component of gimbal pivit point position
// pos2 = -11;% y component of gimbal pivit point position

// g_theta = Wanted Gimbal Deflection (degrees)

double gimbal2servo(double l1, double l2, double l3, double pos1, double pos2, double g_theta)
{
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

int main()
{
	double InnerGimbal = gimbal2servo(27.5, 37.9, 11.9, -37.65, -35, 10);
	// Inner Gimbal Deflection of 10 degrees
	cout << InnerGimbal << endl; // Inner Gimbal is Pitch

	double OuterGimbal = gimbal2servo(25.75, 36.3, 11.9, 36.2, -11, 10);
	// Outer Gimbal Deflection of 10 degrees
	cout << OuterGimbal << endl; // Outer Gimbal is Roll
}
