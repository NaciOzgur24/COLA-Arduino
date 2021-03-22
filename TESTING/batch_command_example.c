// A basic batch command example
//
// Get streaming batch allows you to get up to 8 commands in one request from the sensor
// This is very useful for Bluetooth and wireless sensors when requesting the same data frequently
//
// This example code is compatible with all 3-Space Sensors connected via USB, serial, or Bluetooth
// Requires 2.0+ firmware features
#include "yei_threespace_basic_utils.h"
#include <Windows.h>
#include <stdio.h>

// For a full list of streamable commands refer to the "Wired Streaming Mode" section in the 
// 3-Space Manual of your sensor
#define TSS_GET_TARED_ORIENTATION_AS_QUATERNION 0x00
#define TSS_GET_CORRECTED_LINEAR_ACCELERATION_IN_GLOBAL_SPACE 0x29
#define TSS_GET_BUTTON_STATE 0xfa
#define TSS_NULL 0xff // No command use to fill the empty slots in "set stream slots"
// For a full list of commands refer to the 3-Space Manual of your sensor
#define TSS_SET_STREAMING_SLOTS 0x50
#define TSS_GET_STREAMING_BATCH 0x54

// Stream data stuctures must be packed else they will not properly work
#pragma pack(push,1)
typedef struct Batch_Data{
    float quaternion[4]; // xyzw
    float linear_acceleration[3]; // xyz
	unsigned char button_state; // bit 0 - button 0, bit 1 - button 1
} Batch_Data;
#pragma pack(pop)

int main()
{
	HANDLE com_handle;
	DWORD bytes_written;
	DWORD bytes_read;
	unsigned char write_slot_bytes[11]; // start byte, command, data(8), checksum
	unsigned char write_batch_bytes[3]; // start byte, command, checksum
	Batch_Data batch_data={0};
	int i;

	com_handle = openAndSetupComPort(TEXT("\\\\.\\COM93"));
	if(com_handle == INVALID_HANDLE_VALUE){
		printf("comport open failed\n");
		return 1;
	}

	printf("TSS_SET_STREAMING_SLOTS\n");
	// There are 8 streaming slots available for use, and each one can hold one of the streamable commands.
	// Unused slots should be filled with 0xff so that they will output nothing.
	write_slot_bytes[0]= TSS_START_BYTE;
	write_slot_bytes[1]= TSS_SET_STREAMING_SLOTS;
	write_slot_bytes[2]= TSS_GET_TARED_ORIENTATION_AS_QUATERNION; // stream slot0
	write_slot_bytes[3]= TSS_GET_CORRECTED_LINEAR_ACCELERATION_IN_GLOBAL_SPACE; // stream slot1
	write_slot_bytes[4]= TSS_GET_BUTTON_STATE; // stream slot2
	write_slot_bytes[5]= TSS_NULL; // stream slot3
	write_slot_bytes[6]= TSS_NULL; // stream slot4
	write_slot_bytes[7]= TSS_NULL; // stream slot5
	write_slot_bytes[8]= TSS_NULL; // stream slot6
	write_slot_bytes[9]= TSS_NULL; // stream slot7
	write_slot_bytes[10]= createChecksum(&write_slot_bytes[1], 8+1);
	// Write the bytes to the serial
	if(!WriteFile(com_handle, write_slot_bytes, sizeof(write_slot_bytes), &bytes_written, 0)){
		printf("Error writing to port\n");
		return 2;
	}

	printf("TSS_GET_STREAMING_BATCH\n");
	// With parameterless wired commands the command byte will be the same as the checksum
	write_batch_bytes[0]= TSS_START_BYTE;
	write_batch_bytes[1]= TSS_GET_STREAMING_BATCH;
	write_batch_bytes[2]= TSS_GET_STREAMING_BATCH;
	// Write the bytes to the serial
	if(!WriteFile(com_handle, write_batch_bytes, sizeof(write_batch_bytes), &bytes_written, 0)){
		printf("Error writing to port\n");
		return 2;
	}

	// Read the bytes returned from the serial
	if(!ReadFile(com_handle, &batch_data, sizeof(batch_data), &bytes_read, 0)){
		printf("Error reading from port\n");
		return 3;
	}

	// The data must be fliped to little endian to be read correctly
	for(i=0; i< sizeof(batch_data.quaternion)/sizeof(float); i++){
		endian_swap_32((unsigned int *)&batch_data.quaternion[i]);
	}
	for(i=0; i< sizeof(batch_data.linear_acceleration)/sizeof(float); i++){
		endian_swap_32((unsigned int *)&batch_data.linear_acceleration[i]);
	}

	printf("Quaternion:   % 8.5f, % 8.5f, % 8.5f, % 8.5f\n",	batch_data.quaternion[0], 
																batch_data.quaternion[1], 
																batch_data.quaternion[2],
																batch_data.quaternion[3]);

	printf("Linear Accel: % 8.5f, % 8.5f, % 8.5f\n",	batch_data.linear_acceleration[0], 
														batch_data.linear_acceleration[1], 
														batch_data.linear_acceleration[2]);
	printf("Button State:  %u\n", batch_data.button_state);
	// Close the serial
	CloseHandle(com_handle);
	printf("Finished press Enter to continue");
	getchar();
	return 0;
}