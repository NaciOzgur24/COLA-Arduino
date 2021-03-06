/*
    COLA IMU CALIBRATION
*/

#ifndef _BNO055_CALIBRATION_H
#define _BNO055_CALIBRATION_H

#include "BNO055_support.h"    //Contains the bridge code between the API and Arduino
#include <Wire.h>

//The device address is set to BNO055_I2C_ADDR2 in this example. You can change this in the BNO055.h file in the code segment shown below.
// /* BNO055 I2C Address */
// #define BNO055_I2C_ADDR1                0x28
// #define BNO055_I2C_ADDR2                0x29
// #define BNO055_I2C_ADDR                 BNO055_I2C_ADDR2

//Pin assignments as tested on the Arduino Due.
//Vdd,Vddio : 3.3V
//GND : GND
//SDA/SCL : SDA/SCL
//PSO/PS1 : GND/GND (I2C mode)

//This structure contains the details of the BNO055 device that is connected. (Updated after initialization)
struct bno055_t myBNO;
unsigned char accelCalibStatus = 0;   //Variable to hold the calibration status of the Accelerometer
unsigned char magCalibStatus = 0;   //Variable to hold the calibration status of the Magnetometer
unsigned char gyroCalibStatus = 0;    //Variable to hold the calibration status of the Gyroscope
unsigned char sysCalibStatus = 0;   //Variable to hold the calibration status of the System (BNO055's MCU)

long lastTime_imu1 = 0;


void imu_setup() //This code is executed once
{
  //Initialize I2C communication
  Wire.begin();

  //Initialization of the BNO055
  BNO_Init(&myBNO); //Assigning the structure to hold information about the device

  //Configuration to NDoF mode
  bno055_set_operation_mode(OPERATION_MODE_NDOF);

  delay(1);

  //Initialize the Serial Port to view information on the Serial Monitor
  Serial.begin(115200);
}

void imu_calibration_loop() //This code is looped forever
{
  if((millis()-lastTime_imu1) >= 200) //To read calibration status at 5 Hz without using additional timers
  {
    lastTime_imu1 = millis();
    
    Serial.print("Time Stamp: ");     //To read out the Time Stamp
    Serial.println(lastTime_imu1);
    
    bno055_get_accelcalib_status(&accelCalibStatus);
    Serial.print("Accelerometer Calibration Status: ");   //To read out the Accelerometer Calibration Status (0-3)
    Serial.println(accelCalibStatus);
    
    bno055_get_magcalib_status(&magCalibStatus);
    Serial.print("Magnetometer Calibration Status: ");    //To read out the Magnetometer Calibration Status (0-3)
    Serial.println(magCalibStatus);
    
    bno055_get_magcalib_status(&gyroCalibStatus);
    Serial.print("Gyroscope Calibration Status: ");     //To read out the Gyroscope Calibration Status (0-3)
    Serial.println(gyroCalibStatus);
    
    bno055_get_syscalib_status(&sysCalibStatus);
    Serial.print("System Calibration Status: ");      //To read out the Magnetometer Calibration Status (0-3)
    Serial.println(sysCalibStatus);
    
    Serial.println();                   //To separate between packets
  }
}

#endif // _BNO055_CALIBRATION_H
