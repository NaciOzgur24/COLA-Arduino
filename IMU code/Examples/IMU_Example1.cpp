/*
COLA Arduino (Manufacterer Arduino example)
IMU example code
*/

#include <Wire.h> // If we're using I2C protocol
#define DUE_ADDRESS 0x46
// Extra pins for use with the level shifter
#define Vout 9
#define OE 7

void setup()
{
  Serial.begin(115200); // Baud rate of the Serial
  pinMode(OE, OUTPUT);
  digitalWrite(OE, HIGH);
  pinMode(Vout, OUTPUT);
  digitalWrite(Vout, HIGH);
  Wire.begin();
  Serial.println("Setup complete");

  Wire.beginTransmission(DUE_ADDRESS);
  Wire.write(0x42); // Prepares the sensor to receive a TSS command (Read raw accelerometer vector)
  Wire.write(0xED); // Get the sensors serial number
  Wire.endTransmission();
  // Prepares the sensor to send data out
  Wire.beginTransmission(DUE_ADDRESS);
  Wire.write(0x00); // (0x00) Reading Quaternion
  // Wire.write(0x01); // (0x01) Reading tared orientation as Euler Angles
  Wire.endTransmission();

  // Request 4 bytes of data from the sensor
  Wire.requestFrom(DUE_ADDRESS, 4);

  // Read the data we requested
  if (4 <= Wire.available())
  { // if four bytes were received
    unsigned char result[4];
    for (int i = 0; i < 4; i++)
    {
      result[i] = Wire.read();
    }
    char format[32];
    sprintf(format, "Serial Number: %02x%02x%02x%02x\n", result[0], result[1],
            result[2], result[3]);
    Serial.print(format);
  }
}

void loop()
{

}
