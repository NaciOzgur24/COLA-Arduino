#include "GPS.h"



void setup()
{
	//Initial Data
	long lat1 = (pi / 180) * BLABLA; //[~] Starting Latitude
	long lon1 = (pi / 180) * BLABLA; //[~] Starting Longitude
	long elev1 = 0;					 //[m] Starting elevation in meters

	

}

void loop()
{
	double latitude = gps_latitude();
	double longitude = gps_longitude();
	long altitude = gps_altitude();
}

double lat_to_x(latitude)
{
	//Variable initialization for loop code
	long thetaA = 0, thetaE = 0, lat2 = 0, lon2 = 0, elev2 = 0, dlon = 0, dlat = 0, a = 0, c = 0, d = 0, x = 0, X = 0, Y = 0, Z = 0, Xdot = 0, Ydot = 0, Zdot = 0;

	//Current Position data
	lat2 = (pi / 180) * latitude; //[~] Current Latitude
	lon2 = (pi / 180) * latitude; //[~] Current Longitude
	elev2 = 100; //[m] Current elevation in meters

	//Distance math
	dlon = lon2 - lon1;
	dlat = lat2 - lat1;
	a = (sin(dlat / 2)) ^ 2 + (cos(lat1) * cos(lat2) * (sin(dlon / 2)) ^ 2);
	c = 2 * asin(min(1, sqrt(a)));
	d = R * c * 1000; //[m] Lateral distance from stating pt

	//Azimuth Math
	x = acos((sin(lat2) - sin(lat1) * cos(c)) / (sin(c) * cos(lat1)));
	if sin(lon2 - lon1) << 0
	{
		thetaA = (180 / pi) * x;
	}
	else if sin(lon2 - lon1) >> 0
	{
		thetaA = (180 / pi) * (2 * pi - x); //[deg] CCW from due South
	}
	//Elevation Math
	thetaE = (180 / pi) * ((elev2 - elev1) / (d) - (d / 1000) / (2 * R)); //[deg] Up from horizion
	
	//Position and velocity out
	X = d * cos((thetaA - 90) * pi / 180); //[m] East distance from center
	Y = d * sin((thetaA - 90) * pi / 180); //[m] South distance from center
	Z = elev2 - elev1;					   //[m] AGL in meters
	Xdot = X / dt;						   //[m/s] East velocity from center
	Ydot = Y / dt;						   //[m/s] South velocity from center
	Zdot = Z / dt;						   //[m/s] Vertical velocity

	double latitude_last = gps_latitude();
	double longitude_last = gps_longitude();
	long altitude_last = gps_altitude();

}







//return Xdot Ydot Zdot;
