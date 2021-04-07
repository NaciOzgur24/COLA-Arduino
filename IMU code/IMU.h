/*
COLA Arduino
IMU Code (Asynchronys Protocol)
*/

// 116(0x74) Sets the axis directions for the IMU
// 231(0xe7) Set UART baud rate
// 232(0xe8) Get UART baud rate
// 233(0xe9) Set USB Mode
// 234(0xea) Get USB Mode
// 237(0xed) Get serial number


#ifndef IMU_h
#define IMU_h

#include <SoftwareSerial.h>


void setup()
{
  //pinMode(0, INPUT); // set push button pin as input
  //pinMode(1, OUTPUT); // set LED pin as output
  SoftwareSerial mySerial(0, 1); // Rx, Tx
  //digitalWrite(13, LOW); // Low Voltage on Pin 13
  mySerial.begin(115200); // Initialize UART with baud rate of 115200 bps
  
  //pinMode(15, INPUT); // set push button pin as input
  //pinMode(14, OUTPUT); // set LED pin as output
  SoftwareSerial mySerial1(15, 14); // Rx, Tx
  //digitalWrite(12, LOW); // Low Voltage on Pin 12
  mySerial1.begin(115200); // Initialize UART with baud rate of 115200 bps
}

void loop()
{
  if (mySerial.available())
  {
    byte data_rcvd = mySerial.read(); // Read one byte from serial buffer and save to data_rcvd
    imu_data();
  }

}

byte imu_data(byte clear_buffer, byte send_packet, byte Tared_Quaternion, byte status)
{
  // Clear the internal data buffer on the IMU
  byte clear_buffer = transferByte(0x01);
  mySerial.print("Cleared internal buffer. Result: "), mySerial.println(clear_buffer);

  // Send start of packet:
  byte send_packet = transferByte(0xF6);
  mySerial.print("Send start of packet. Result: "), mySerial.println(send_packet);

  // Send command (Tared Quaternion)
  byte Tared_Quaternion = transferByte(0x00);
  mySerial.print("Send commmand 0x00. Result: "), mySerial.println(Tared_Quaternion);

  // Get status of device:
  byte status = transferByte(0xFF);
  mySerial.print("Status of device. Result: "), mySerial.println(status);

  while (status != 0x01)
  { // Repeat until device is Ready
    status = transferByte(0xFF);
    mySerial.print("Status of device. Result: "), mySerial.println(status);
  }
}

#endif