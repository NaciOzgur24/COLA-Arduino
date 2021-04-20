// Get tared orientation as Quaternion with the programmable response header
//
// The 3-Space Sensor is capable of returning additional data that can be prepended to all command responses. 
//
// This example code is compatible with all 3-Space Sensors connected via USB, serial, or Bluetooth
// Requires 2.0+ firmware features
#include "yei_threespace_basic_utils.h"
#include <Windows.h>
#include <stdio.h>

// For a full list of commands refer to the 3-Space Manual of your sensor
#define TSS_GET_TARED_ORIENTATION_AS_QUATERNION 0x00
#define TSS_SET_WIRED_RESPONSE_HEADER_BITFIELD  0xdd

// This structure must match the options in response_header_setup
#pragma pack(push,1)
typedef struct Response_Header{
    unsigned char success_failure;
    unsigned int timestamp;
    unsigned char command_echo;
	// unsigned char checksum;	//(Commented out as it is not being used)
	// unsigned char logical_id;	//(Commented out as it is not being used)
	// unsigned int serial_number;	//(Commented out as it is not being used)
    unsigned char data_length;
} Response_Header;
#pragma pack(pop)

int main()
{
	HANDLE com_handle;
	DWORD bytes_written;
	DWORD bytes_read;
	unsigned int response_header_setup=0;
	unsigned char write_rh_bytes[7];   // start byte, command, data(4), checksum
	unsigned char write_quat_bytes[3]; // start byte, command, checksum
	Response_Header response_header;
	float quat[4]; // quaternion xyzw
	int i;


	com_handle = openAndSetupComPort(TEXT("\\\\.\\COM93"));
	if(com_handle == INVALID_HANDLE_VALUE){
		printf("comport open failed\n");
		return 1;
	}
	printf("TSS_SET_WIRED_RESPONSE_HEADER_BITFIELD\n");
	// Setting up the response header, note the size is a 32 bit int even tho only 7 bits are used right now
	response_header_setup =TSS_RH_SUCCESS_FAILURE;
	response_header_setup+=TSS_RH_TIMESTAMP;
	response_header_setup+=TSS_RH_COMMAND_ECHO;
	response_header_setup+=TSS_RH_DATA_LENGTH;

	// The data must be flipped to big endian before sending to sensor
	endian_swap_32((unsigned int *)&response_header_setup);

	write_rh_bytes[0]= TSS_START_BYTE;
	write_rh_bytes[1]= TSS_SET_WIRED_RESPONSE_HEADER_BITFIELD;
	memcpy(&write_rh_bytes[2], &response_header_setup, sizeof(response_header_setup)); 
	write_rh_bytes[sizeof(write_rh_bytes)-1]= createChecksum(&write_rh_bytes[1], sizeof(response_header_setup)+1);
	// Write the bytes to the serial
	if(!WriteFile(com_handle, write_rh_bytes, sizeof(write_rh_bytes), &bytes_written, 0)){
		printf("Error writing to port\n");
		return 2;
	}

	printf("TSS_GET_TARED_ORIENTATION_AS_QUATERNION\n");
	// With parameterless wired commands the command byte will be the same as the checksum
	// When this startbyte is used commands will return with the reponse header
	write_quat_bytes[0]= TSS_RESPONSE_HEADER_START_BYTE;  
	write_quat_bytes[1]= TSS_GET_TARED_ORIENTATION_AS_QUATERNION;
	write_quat_bytes[2]= TSS_GET_TARED_ORIENTATION_AS_QUATERNION;
	// Write the bytes to the serial
	if(!WriteFile(com_handle, write_quat_bytes, sizeof(write_quat_bytes), &bytes_written, 0)){
		printf("Error writing to port\n");
		return 3;
	}

	// Read the response header from the serial
	if(!ReadFile(com_handle, &response_header, sizeof(response_header), &bytes_read, 0)){
		printf("Error reading from port\n");
		return 3;
	}
	// The data must be flipped to little endian to be read correctly
	endian_swap_32((unsigned int *)&response_header.timestamp);
	printf("Response Header:\n\tsuccess_failure:%u\n\ttimestamp:%u\n\tcommand_echo:%u\n\tdata_length:%u\n",
		response_header.success_failure,
		response_header.timestamp, // timestamp is in microseconds
		response_header.command_echo,
		response_header.data_length);

	// Read the bytes returned from the serial
	if(!ReadFile(com_handle, quat, sizeof(quat), &bytes_read, 0)){
		printf("Error reading from port\n");
		return 3;
	}
	// The data must be fliped to little endian to be read correctly
	for(i=0; i< sizeof(quat)/sizeof(float); i++){
		endian_swap_32((unsigned int *)&quat[i]);
	}

	printf("Quaternion: % 8.5f, % 8.5f, % 8.5f, % 8.5f\n",	quat[0], 
															quat[1], 
															quat[2],
															quat[3]);
	// Close the serial
	CloseHandle(com_handle);
	printf("Finished press Enter to continue");
	getchar();
	return 0;
}