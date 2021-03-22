#include <threespace_api_export.h>

// A basic get tared orientation as Quaternion example
// This example code is compatible with all 3-Space Sensors connected via USB, serial, or Bluetooth
#include "yei_threespace_basic_utils.h"
#include <Windows.h>
#include <stdio.h>
// For a full list of commands refer to the 3-Space Manual for your sensor
#define TSS_GET_TARED_ORIENTATION_AS_QUATERNION 0x00

int main()
{
	HANDLE com_handle;
	DWORD bytes_written;
	DWORD bytes_read;
	unsigned char write_bytes[3]; // start byte, command, checksum
	float quat[4]; // quaternion xyzw
	int i;

	com_handle = openAndSetupComPort(TEXT("\\\\.\\COM93"));
	if(com_handle == INVALID_HANDLE_VALUE){
		printf("comport open failed\n");
		return 1;
	}
	printf("TSS_GET_TARED_ORIENTATION_AS_QUATERNION\n");
	
	// With parameterless wired commands the command byte will be the same as the checksum
	write_bytes[0]= TSS_START_BYTE;
	write_bytes[1]= TSS_GET_TARED_ORIENTATION_AS_QUATERNION;
	write_bytes[2]= TSS_GET_TARED_ORIENTATION_AS_QUATERNION;
	// Write the bytes to the serial
	if(!WriteFile(com_handle, write_bytes, sizeof(write_bytes), &bytes_written, 0)){
		printf("Error writing to port\n");
		return 2;
	}
	// Read the bytes returned from the serial
	if(!ReadFile(com_handle, quat, sizeof(quat), &bytes_read, 0)){
		printf("Error reading from port\n");
		return 3;
	}
	// The data must be flipped to little endian to be read correctly
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
