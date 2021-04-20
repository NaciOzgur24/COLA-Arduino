// A basic get tared orientation as Quaternion example over wireless
// This example code is for 3-Space Wireless sensors and dongles, not for Bluetooth
// The 3-Space Wireless sensor must be paired with a dongle prior to running this code
#include "yei_threespace_basic_utils.h"
#include <Windows.h>
#include <stdio.h>
// For a full list of commands refer to the 3-Space Manual of your sensor
#define TSS_GET_TARED_ORIENTATION_AS_QUATERNION 0x00

int main()
{
	HANDLE com_handle;
	DWORD bytes_written;
	DWORD bytes_read;
	unsigned char write_bytes[4]; // start byte, logical id, command, checksum
	unsigned char response_header[2]; // failbyte, logical id
	unsigned char response_data_size; // return data size, only returned with sucsessful responces
	float quat[4]; // quaternion xyzw
	int logical_id= 0; // valid range is 0-14, 0 is the first sensor in the logical id table
	int i;

	com_handle = openAndSetupComPort(TEXT("\\\\.\\COM15"));
	if(com_handle == INVALID_HANDLE_VALUE){
		printf("comport open failed\n");
		return 1;
	}
	printf("TSS_GET_TARED_ORIENTATION_AS_QUATERNION\n");
	
	// Wireless commands take a logical id in addition to the usual wired commands
	write_bytes[0]= TSS_WIRELESS_START_BYTE;
	write_bytes[1]= logical_id; 
	write_bytes[2]= TSS_GET_TARED_ORIENTATION_AS_QUATERNION;
	write_bytes[sizeof(write_bytes)-1]= createChecksum(&write_bytes[1], 2); // logical id + command
	// Write the bytes to the serial
	if(!WriteFile(com_handle, write_bytes, sizeof(write_bytes), &bytes_written, 0)){
		printf("Error writing to port\n");
		return 2;
	}

	// Read the bytes returned from the serial
	// Response header contains the fail byte and the logical id
	// If success, an additional size byte is also returned followed by the data.
	if(!ReadFile(com_handle, response_header, sizeof(response_header), &bytes_read, 0)){
		printf("Error reading from port\n");
		return 3;
	}
	if(response_header[0] == 0){ //successful send and receive 
		if(!ReadFile(com_handle, &response_data_size, 1, &bytes_read, 0)){
			printf("Error reading from port\n");
			return 4;
		}
		// Reading the data, response_data_size should match size of the data requested
		if(!ReadFile(com_handle, quat, sizeof(quat), &bytes_read, 0)){
			printf("Error reading from port\n");
			return 5;
		}

		// The data must be flipped to little endian to be read correctly
		for(i=0; i< sizeof(quat)/sizeof(float); i++){
			endian_swap_32((unsigned int *)&quat[i]);
		}

		printf("Quaternion: % 8.5f, % 8.5f, % 8.5f, % 8.5f\n",	quat[0], 
																quat[1], 
																quat[2],
																quat[3]);
	}
	else{
		printf("Command was lost\n");
	}
	// Close the serial
	CloseHandle(com_handle);
	printf("Finished press Enter to continue");
	getchar();
	return 0;
}