// A basic get all corrected component sensor data
// This example code is compatible with all 3-Space Sensors connected via USB, serial, or Bluetooth
#include "yei_threespace_basic_utils.h"
#include <Windows.h>
#include <stdio.h>
// For a full list of commands refer to the 3-Space Manual of your sensor
#define TSS_GET_ALL_CORRECTED_COMPONENT_SENSOR_DATA 0x25

int main()
{
	HANDLE com_handle;
	DWORD bytes_written;
	DWORD bytes_read;
	unsigned char write_bytes[3]; // start byte, command, checksum
	float data[9]; // gyro xyz, accel xyz, compass xyz
	int i;

	com_handle = openAndSetupComPort(TEXT("\\\\.\\COM93"));
	if(com_handle == INVALID_HANDLE_VALUE){
		printf("comport open failed\n");
		return 1;
	}
	printf("TSS_GET_ALL_CORRECTED_COMPONENT_SENSOR_DATA\n");
	
	// With parameterless wired commands the command byte will be the same as the checksum
	write_bytes[0]= TSS_START_BYTE;
	write_bytes[1]= TSS_GET_ALL_CORRECTED_COMPONENT_SENSOR_DATA;
	write_bytes[2]= TSS_GET_ALL_CORRECTED_COMPONENT_SENSOR_DATA;
	// Write the bytes to the serial
	if(!WriteFile(com_handle, write_bytes, sizeof(write_bytes), &bytes_written, 0)){
		printf("Error writing to port\n");
		return 2;
	}
	// Read the bytes returned from the serial
	if(!ReadFile(com_handle, data, sizeof(data), &bytes_read, 0)){
		printf("Error reading from port\n");
		return 3;
	}
	// The data must be fliped to little endian to be read correctly
	for(i=0; i< sizeof(data)/sizeof(float); i++){
		endian_swap_32((unsigned int *)&data[i]);
	}

	printf("Gyro in radians/sec:  % 8.5f, % 8.5f, % 8.5f\n",	data[0],
																data[1],
																data[2]);
	printf("Acceleration in G:    % 8.5f, % 8.5f, % 8.5f\n",	data[3],
																data[4],
																data[5]);
	printf("Compass in gauss:     % 8.5f, % 8.5f, % 8.5f\n",	data[6],
																data[7],
																data[8]);
	// Close the serial
	CloseHandle(com_handle);
	printf("Finished press Enter to continue");
	getchar();
	return 0;
}