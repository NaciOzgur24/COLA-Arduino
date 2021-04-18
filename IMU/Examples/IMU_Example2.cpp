/*
COLA Arduino
IMU Code
*/

#include<SPI.h>
#include<stdlib.h>
#define TRANSACTION_SETTINGS SPISettings(1000000,MSBFIRST,SPI_MODE0)
#define SS 10
// Extra pins for use with the level shifter
#define OE 7
#define Vout 9
//these are the most reliable delays tested
#define MicroDelay 6
#define MiliDelay 3
/*Spi Command List:
0xE9: This command prepares the sensor to receive a TSS command
0xD9: This command will clear the command stored in the sensor
0x69: This command tells the sensor to return the command data
0x81: This command prepares the sensor to send the status of the slave device
more information in data sheet page 23 */
void setup() {
 Serial.begin(19200);
 SPI.begin();
 pinMode(OE, OUTPUT);
 digitalWrite(OE, HIGH);
 pinMode(Vout, OUTPUT);
 digitalWrite(Vout, HIGH);
 pinMode(SS,OUTPUT);
 digitalWrite(SS,HIGH);
 SPI.beginTransaction(TRANSACTION_SETTINGS);
 ReadSerialNumber();
}
void loop() {
 //print out tared orientation every half second
 delay(500);
 ReadOrientationQuat();
}
void ReadSerialNumber(){
 char buff[32];

 digitalWrite(SS,LOW);
 SPI.transfer(0xe9); //tell the sensor the next byte will be a command
 delayMicroseconds(MicroDelay);
 SPI.transfer(0xed); //command to get Serial Number of sensor
 digitalWrite(SS,HIGH);
 delay(MiliDelay);
 unsigned char in_byte[5];
 unsigned char polling_byte = 0x00;
 //Poll the sensor until it returns the return data length in this case it will be
4
 while(polling_byte != 4){
 digitalWrite(SS,LOW);
 polling_byte = SPI.transfer(0x69);
 delayMicroseconds(MicroDelay);
 digitalWrite(SS,HIGH);
 delay(MiliDelay);
 }
 //Read out requested command data
 digitalWrite(SS,LOW);
 for(unsigned int i = 0; i < polling_byte; i++){
 in_byte[i]=SPI.transfer(0xff);
 delayMicroseconds(MicroDelay);
 }
 digitalWrite(SS,HIGH);
 sprintf(buff,"%02x%02x%02x%02x",in_byte[0],in_byte[1],in_byte[2],in_byte[3]);
 Serial.println(buff);
 Serial.println("done");

}
void ReadOrientationQuat(){
 char buff[32];
 byte in_byte[16];
 unsigned char polling_byte = 0x00;
 digitalWrite(SS,LOW);
 SPI.transfer(0xe9); //tell the sensor the next byte will be a command
 delayMicroseconds(MicroDelay);
 SPI.transfer(0x00); //command to get tared orientation in quaternion form
 digitalWrite(SS,HIGH);
 delay(MiliDelay);
 //Poll the sensor until it returns the return data length
 while(polling_byte != 16){
 digitalWrite(SS,LOW);
 polling_byte = SPI.transfer(0x69);
 delayMicroseconds(MicroDelay);
 digitalWrite(SS,HIGH);
 delay(MiliDelay);
 }
 for(unsigned int i = 0; i < polling_byte; ++i){
 //Read out requested command data
 digitalWrite(SS,LOW);
 in_byte[i]=SPI.transfer(0xff);
 delayMicroseconds(MicroDelay);
 }
 for(unsigned int i = 0; i < 4; ++i){
 //reverse endianess and append to string
 ((float*)in_byte)[i] = ReverseFloat(((float*)in_byte)[i]);
 dtostrf(((float*)in_byte)[i],8,5,(buff +(i*8 +i)));
 if(i*8 + i-1 < 32 && i){
 buff[i*8 + i-1] = ',';
 }
 }
 Serial.println(buff);
}
float ReverseFloat( const float inFloat )
{
 float retVal;
 char *floatToConvert = ( char* ) & inFloat;
 char *returnFloat = ( char* ) & retVal;
 // swap the bytes into a temporary buffer
 returnFloat[0] = floatToConvert[3];
 returnFloat[1] = floatToConvert[2];
 returnFloat[2] = floatToConvert[1];
 returnFloat[3] = floatToConvert[0];
 return retVal;
}
