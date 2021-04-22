#include <SD.h>
#include <SPI.h>
#include <Servo.h>
#include<Wire.h>
//#include <KalmanFilter.h>
const int MPU=0x68; 
int16_t AcX,AcY,AcZ,Tmp,GyX,GyroY,GyroZ;
int pitch;
int accAngleX;
int accAngleY;
int yaw;
int GyroX;
int gyroAngleX;
int gyroAngleY;
int ledBLU = 14;    // LED connected to digital pin 9
int ledGRN = 16;    // LED connected to digital pin 9
int ledRED = 15;    // LED connected to digital pin 9
int mosfet = 24;
int valueX = 110;
int valueY = 150;
Servo servoX;
Servo servoY;


float elapsedTime, currentTime, previousTime;
float kp = 1;
float ki = 1;
float kd = 1;
int pos;
//KalmanFilter kalman(0.001, 0.003, 0.03);

void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission(true);
  Serial.begin(9600);
  servoX.attach(5);
  servoY.attach(6);
  pinMode(mosfet, OUTPUT);
   pinMode(ledBLU, OUTPUT);
  pinMode(ledGRN, OUTPUT);
  pinMode(ledRED, OUTPUT);
  
  digitalWrite(ledBLU, HIGH);
  delay(500);
  digitalWrite(ledBLU, LOW);
   
  digitalWrite(ledGRN, HIGH);
  delay(500);
  digitalWrite(ledGRN, LOW);
     
  digitalWrite(ledRED, HIGH);
  delay(500);
  digitalWrite(ledRED, LOW);



   digitalWrite(ledBLU, HIGH);
  delay(500);
  digitalWrite(ledBLU, LOW);
   
  digitalWrite(ledGRN, HIGH);
  delay(500);
  digitalWrite(ledGRN, LOW);
     
  digitalWrite(ledRED, HIGH);
  delay(500);
  digitalWrite(ledRED, LOW);
  Serial.begin(9600);
  //120 minutes
  //delay(7200000);
  //digitalWrite(ledBLU, HIGH);
  //delay(5000);
  //digitalWrite(ledBLU, LOW);
}
void loop() {
  
  filter();
  map();
  previousTime = currentTime;        
  currentTime = millis();            
  elapsedTime = (currentTime - previousTime) / 1000; 
  accAngleX = (atan(AcY / sqrt(pow(AcX, 2) + pow(AcZ, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AcX / sqrt(pow(AcY, 2) + pow(AcZ, 2))) * 180 / PI) + 1.58; // 
 
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,12,true);
  AcX=Wire.read()<<8|Wire.read();    
  AcY=Wire.read()<<8|Wire.read();  
  AcZ=Wire.read()<<8|Wire.read();  
  GyroX=Wire.read()<<8|Wire.read();  
  GyroY=Wire.read()<<8|Wire.read();  
  GyroZ=Wire.read()<<8|Wire.read();  
  
  gyroAngleX = gyroAngleX + GyroZ * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  //Serial.print(valueX);
  
  
  //Serial.print("Accelerometer: ");
  //Serial.print("X = "); Serial.print(AcX);
  //Serial.print(" | Y = "); Serial.print(AcY);
 // Serial.print(" | Z = "); Serial.println(AcZ);
  
 //Serial.print("Gyroscope: ");
 //Serial.print("X = "); Serial.print(GyX);
 // Serial.print(" | Y = "); Serial.print(GyroY);
  Serial.print(" | Z = "); Serial.println(GyroZ);
  //ignition();
   delay(5);

}
void filter () {
  Serial.print(GyroX);
  pitch = 0.9 * gyroAngleX + 0.1 * accAngleX;
 yaw = 0.9 * gyroAngleY + 0.1 * accAngleY;
}

void map () {
  
 
 //TVC Startup//
 digitalWrite(ledBLU, HIGH);
 valueY = map(yaw, -2000, 2000, 90, 130);
 valueX = map(pitch, -2000, 2000, 105, 145);
 servoX.write(valueX);
 servoY.write(valueY); 
 
  
 

}
void ignition () {
  
 
 digitalWrite(mosfet, HIGH);
  Serial.println("mosfetHIGH");
  delay(3000);
 
  
 

}
