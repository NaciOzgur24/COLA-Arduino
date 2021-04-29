/********************************************************
 * PID Basic Example
 * Reading analog Input 0 to control analog PWM output 3
 ********************************************************/

#include "PID_v2.h"


//Hereafter the code for the IMU+servo
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Servo.h"

MPU6050 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz;

Servo myservo;

int Val, Target;
//int prevVal;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,1,0,0, DIRECT);

void setup()
{
  //initialize the variables we're linked to

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Input = map(ay, -17000, 17000, 0, 255);   
  Setpoint = 160;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  
  //Hereafter the code for the IMU+servo
  Wire.begin();
  Serial.begin(38400);

  Serial.println("Initialize MPU");
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "Connected" : "Connection failed");
  myservo.attach(9);
  
  Serial.begin(9600); //initialize serial monitor
  //myPID.SetOutputLimits(0, 180);
}

void loop()
{
  //Hereafter the code for the IMU+servo 
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  Val = map(ay, -17000, 17000, 1, 180);
  Input = (double)Val;
  myPID.Compute();
  myservo.write(Output);
  Serial.print("Setpoint: ");
  Serial.print(Setpoint);
  Serial.print("ay: ");
  Serial.print(ay);
  Serial.print("Val: ");
  Serial.print(Val);
  Serial.print(" Intput: ");
  Serial.print(Input);
  Serial.print(" Output: ");
  Serial.println(Output);

  delay(100);  
}