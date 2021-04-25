/*
    COLA IMU gets Euler Angles

    //Euler data registers
        #define BNO055_EUL_HEADING_LSB_ADDR			0X1A
        #define BNO055_EUL_HEADING_MSB_ADDR			0X1B

        #define BNO055_EUL_ROLL_LSB_ADDR			0X1C
        #define BNO055_EUL_ROLL_MSB_ADDR			0X1D

        #define BNO055_EUL_PITCH_LSB_ADDR			0X1E
        #define BNO055_EUL_PITCH_MSB_ADDR			0X1F

    //Quaternion data registers
        #define BNO055_QUA_DATA_W_LSB_ADDR			0X20
        #define BNO055_QUA_DATA_W_MSB_ADDR			0X21
        #define BNO055_QUA_DATA_X_LSB_ADDR			0X22
        #define BNO055_QUA_DATA_X_MSB_ADDR			0X23
        #define BNO055_QUA_DATA_Y_LSB_ADDR			0X24
        #define BNO055_QUA_DATA_Y_MSB_ADDR			0X25
        #define BNO055_QUA_DATA_Z_LSB_ADDR			0X26
        #define BNO055_QUA_DATA_Z_MSB_ADDR			0X27
*/
//Adafruit_BNO055 bno = Adafruit_BNO055(55,0x29);

#ifndef _BNO055_EULER_ANGLES_H
#define _BNO055_EULER_ANGLES_H

#include <Wire.h>
#include "BNO055_support.h"		//Contains the bridge code between the API and Arduino

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

struct bno055_t myBNO; //This structure contains the details of the BNO055 device that is connected. (Updated after initialization)
struct bno055_euler myEulerData; //Structure to hold the Euler data

long lastTime_imu2 = 0;

void setup()
{
  Wire.begin(); //I2C communication

  BNO_Init(&myBNO); //(Initialization of the BNO055) Assigning the structure to hold information about the device
  
  bno055_set_operation_mode(OPERATION_MODE_NDOF); //Configuration to NDoF mode
  delay(1);

  Serial.begin(115200);
}

void imu_Euler_Angles_loop()
{
  if ((millis() - lastTime_imu2) >= 100) //To stream at 10 Hz without using additional timers
  {
    lastTime_imu2 = millis();

    bno055_read_euler_hrp(&myEulerData);			//Update Euler data into the structure

    Serial.print("Time Stamp: ");				//To read out the Time Stamp
    Serial.println(lastTime_imu2);

    Serial.print("Heading(Yaw): ");				//To read out the Heading (Yaw)
    Serial.println(float(myEulerData.h) / 16.00);		//Convert to degrees

    Serial.print("Roll: ");					//To read out the Roll
    Serial.println(float(myEulerData.r) / 16.00);		//Convert to degrees

    Serial.print("Pitch: ");				//To read out the Pitch
    Serial.println(float(myEulerData.p) / 16.00);		//Convert to degrees

    Serial.println();
  }
}

#endif // _BNO055_EULER_ANGLES_H
