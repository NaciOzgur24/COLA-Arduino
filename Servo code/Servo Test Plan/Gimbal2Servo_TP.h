/*
   Gimbal to Servo header code
   Inner Gimbal is Roll
   Outer Gimbal is Pitch
*/

#ifndef _GIMBALTOSERVO_H
#define _GIMBALTOSERVO_H

#include <math.h>

double gimbal2servo(double l1, double l2, double l3, double pos1, double pos2, double g_theta)
{
  double pi = 3.1415926535;
  double point2x = cos((g_theta * (pi / 180)) - pi / 2) * l1;
  double point2y = sin((g_theta * (pi / 180)) - pi / 2) * l1;
  double d = sqrt(pow((point2x - pos1), 2) + pow((point2y - pos2), 2));
  double L = (pow(l2, 2) - pow(l3, 2) + pow(d, 2)) / (2 * d);
  double h = sqrt(pow(l2, 2) - pow(L, 2));
  double p3_x = (L / d) * (pos1 - point2x) + (h / d) * (pos2 - point2y) + point2x;
  double p3_y = (L / d) * (pos2 - point2y) - (h / d) * (pos1 - point2x) + point2y;
  double servoangle = (atan((p3_x - pos1) / (pos2 - p3_y))) * (180 / pi);
  return servoangle += 90;
}

#endif // _GIMBALTOSERVO_H
