#ifndef _VELOCITY_H
#define _VELOCITY_H

#include "GPS.h"



void setup()
{
	double pi = 3.14159265;
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







// INPUT EXAMPLE   -   alt_to_z([Launch Site Ground Elevation (m)], [GPS Altitude OUTPUT])

double alt_to_z(double elevi, double altitude)
{
	double Z = (altitude / 1000) - elevi;
	return Z;
}


// INPUT EXAMPLE   -   lat_to_x([Reference Latitude]x10^7, [Reference Longitude]x10^7, [GPS Latitude OUTPUT], [GPS Longitude OUTPUT])
// First two inputs will remain constant. They will be the position of the launch site.
// OUTPUTS X distance relative to reference position (meters)

double lat_to_x(double lati, double loni, double latitude, double longitude)
{
	double pi = 3.14159265;
	double R = (6378 + 6357) / 2;

	//Initial Data
	double lat1 = (pi / 180) * lati / 10000000.0; //[~] Starting Latitude
	double lon1 = (pi / 180) * loni / 10000000.0; //[~] Starting Longitude

	//Current Position data
	double lat2 = (pi / 180) * latitude / 10000000.0; //[~] Current Latitude
	double lon2 = (pi / 180) * longitude / 10000000.0; //[~] Current Longitude

	//Distance math
	double dlon = (lon2 - lon1);
	double dlat = (lat2 - lat1);
	double a = pow((sin(dlat / 2)),2) + (cos(lat1) * cos(lat2) * pow((sin(dlon / 2)),2));
	double c = 2 * asin(sqrt(a));
	double d = R * c * 1000; //[m] Lateral distance from stating pt

	//Azimuth Math
	double thetaA = 0;
	double x = acos((sin(lat2) - sin(lat1) * cos(c)) / (sin(c) * cos(lat1)));
	if ((sin(lon2 - lon1)) < 0.0)
	{
		thetaA = (180 / pi) * x;
	}
	else
	{
		thetaA = (180 / pi) * (2 * pi - x); //[deg] CCW from due South
	}

	//Position out
	double X = d * cos((thetaA - 90) * pi / 180); //[m] East distance from center
	return X;
}



// INPUT EXAMPLE   -   long_to_y([Reference Latitude], [Reference Longitude], [GPS Latitude OUTPUT], [GPS Longitude OUTPUT])
// First two inputs will remain constant. They will be the position of the launch site.
// OUTPUTS Y distance relative to reference position (meters)

double long_to_y(double lati, double loni, double latitude, double longitude)
{
	double pi = 3.14159265;
	double R = (6378 + 6357) / 2;

	//Initial Data
	double lat1 = (pi / 180) * lati / 10000000.0; //[~] Starting Latitude
	double lon1 = (pi / 180) * loni / 10000000.0; //[~] Starting Longitude

	//Current Position data
	double lat2 = (pi / 180) * latitude / 10000000.0; //[~] Current Latitude
	double lon2 = (pi / 180) * longitude / 10000000.0; //[~] Current Longitude

	//Distance math
	double dlon = (lon2 - lon1);
	double dlat = (lat2 - lat1);
	double a = pow((sin(dlat / 2)), 2) + (cos(lat1) * cos(lat2) * pow((sin(dlon / 2)), 2));
	double c = 2 * asin(sqrt(a));
	double d = R * c * 1000; //[m] Lateral distance from stating pt

	//Azimuth Math
	double thetaA = 0;
	double x = acos((sin(lat2) - sin(lat1) * cos(c)) / (sin(c) * cos(lat1)));
	if ((sin(lon2 - lon1)) < 0.0)
	{
		thetaA = (180 / pi) * x;
	}
	else
	{
		thetaA = (180 / pi) * (2 * pi - x); //[deg] CCW from due South
	}
	//Position out
	double Y = d * sin((thetaA - 90) * pi / 180); //[m] South distance from center
	return Y;
}



#endif // _VELOCITY_H
