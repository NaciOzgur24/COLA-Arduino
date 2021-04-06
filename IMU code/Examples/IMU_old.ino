/* Still have to change from SPI protocol to USB 2.0
COLA Arduino
IMU Code (USB 2.0 Protocol)
*/

#include <SPI.h>

//Set Slave Select Pin
//MOSI, MISO, CLK are handeled automatically

const int CSN = 2; //4
const int SO = 3; //OG const int SO = 74;
const int SI = 4; //75
const int CLK = 5; //76

// Needed to convert the bytes from SPI to float (***NEED TO UPDATE THIS BECAUSE WE'RE GETTING QUATTERNION INSTEAD OF EULER ANGLES***)
union u_types {
    byte b[4];
    float fval;
} data[3]; // Create 3 unions, one for each Euler Angle (It was the inputs for the PID but we can get Quatternion if we want)

void setup()
{
  //Seting Pin Modes
  pinMode(CSN, OUTPUT);
  pinMode(SI, OUTPUT); // Input
  pinMode(SO, INPUT); // Output
  pinMode(CLK, OUTPUT); // Clock

  //Set Slave Select High to Start i.e disable chip
  digitalWrite(CSN, HIGH);

  //Initialize SPI and set SPI mode/bit order
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  Serial.begin(115200); //OG 9600
}

//function to transfer commands through SPI
byte transferByte(byte byteToWrite)
{
  byte Result = 0x00;
  digitalWrite(CSN,LOW);
  delay(1);

  Result = SPI.transfer(byteToWrite);
  delay(1);

  digitalWrite(CSN,HIGH);

  return Result;

//function to swap endian
void endianSwap(byte temp[4])
{
  byte myTemp = temp[0];
  temp[0] = temp[3];
  temp[3] = myTemp;

  myTemp = temp[1];
  temp[1] = temp[2];
  temp[2] = myTemp;
}

void loop()
{
  // Clear the internal data buffer on the IMU
  byte result = transferByte(0x01);
  Serial.print("Cleared internal buffer. Result: "),Serial.println(result);

  // Send start of packet:
  result = transferByte(0xF6);
  Serial.print("Send start of packet. Result: "),Serial.println(result);

  // Send command (Tared Quaternion)
  result = transferByte(0x00);
  Serial.print("Send commmand 0x00. Result: "),Serial.println(result);

  // Get status of device:
  result = transferByte(0xFF);
  Serial.print("Status of device. Result: "),Serial.println(result);

  while (result != 0x01)
  { // Repeat until device is Ready
    result = transferByte(0xFF);
    Serial.print("Status of device. Result: "),Serial.println(result);
  }
}
