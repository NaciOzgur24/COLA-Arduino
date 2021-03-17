/*
COLA Arduino 
Servo code reading output data from the PID controller
*/

#include "Wire.h"
#include <Servo.h>
#include "PID.h" // The PID controller code from Simulink

Servo servo_x; // Connect to the servo in x-axis
Servo servo_y; // y-axis 

int angle = 0;

float PIDYaw = 0;
float PIDPitch = 0;
float PIDRoll = 0;

float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// relative ypr[x] usage based on sensor orientation when mounted, e.g. ypr[PITCH]
#define YAW   0     // defines the position within ypr[x] variable for YAW; may vary due to sensor orientation when mounted
#define PITCH   1     // defines the position within ypr[x] variable for PITCH; may vary due to sensor orientation when mounted
#define ROLL  2     // defines the position within ypr[x] variable for ROLL; may vary due to sensor orientation when mounted

void setup()
{
  servo_x.attach(9); // attach the signal pin of servo to pin9 of arduino
  servo_y.attach(10); // attach the signal pin of servo to pin10 of arduino
  for(angle = 0; angle < 120; angle += 1)    // command to move from 0 degrees to 120 degrees 
  {
    servo_x.write(angle); // command to rotate the servo to the specified angle x-axis
    servo_y.write(angle); // y-axis
    delay(500);
  }
  delay(1000);
  for(angle = 120; angle >= 1; angle -= 1) // command to move from 120 degrees to 0 degrees 
  {
    servo_x.write(angle); //command to rotate the servo to the specified angle
    servo_y.write(angle); //command to rotate the servo to the specified angle
    delay(500);
  }
  delay(1000);
}

void loop()
{
  processAccelGyro();
}

void processAccelGyro()
{

  /*/ Get INT_STATUS byte
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
    return;
  }

  if (mpuIntStatus & 0x02)  // otherwise continue processing
  {
    // check for correct available data length
    if (fifoCount < packetSize)
      return; //  fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    fifoCount -= packetSize;

    // flush buffer to prevent overflow
    mpu.resetFIFO();

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    */

    PIDYaw  = ypr[YAW] * 180 / M_PI;
    PIDPitch = ypr[PITCH] * 180 / M_PI;
    PIDRoll = ypr[ROLL] * 180 / M_PI;
    
    /*/ flush buffer to prevent overflow
    mpu.resetFIFO();

    */

    servo_x.write(-PIDPitch + 90);
    servo_y.write(PIDRoll + 90);
    //delay(10);

  } // if (mpuIntStatus & 0x02)
}  // processAccelGyro()
