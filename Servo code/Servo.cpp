/*
COLA Arduino 
Servo Code
*/

#include <Servo.h>

Servo servo_x; // Connect to the servo in x-axis
Servo servo_y; // y-axis 

int angle = 0;

void setup()
{
  servo_x.attach(9); // attach the signal pin of servo to pin9 of arduino
  servo_y.attach(10); // attach the signal pin of servo to pin10 of arduino
}

void loop()
{
  for(angle = 0; angle < 120; angle += 1)    // command to move from 0 degrees to 180 degrees 
  {
    servo_x.write(angle); // command to rotate the servo to the specified angle x-axis
    servo_y.write(angle); // y-axis
    delay(15);
  }
  delay(1000);
  for(angle = 120; angle >= 1; angle -= 5) // command to move from 180 degrees to 0 degrees 
  {
    servo_test.write(angle); //command to rotate the servo to the specified angle
    delay(5);
  }
    delay(1000);
}
