// A basic set LED color example
// This example code is compatible with all 3-Space Sensors connected via USB, serial, or Bluetooth
#include "yei_threespace_basic_utils.h"
#include <Windows.h>
#include <stdio.h>
// For a full list of commands refer to the 3-Space Manual of your sensor
#define TSS_SET_LED_COLOR 0xee

int main()
{
	HANDLE com_handle;
	DWORD bytes_written;
	unsigned char write_bytes[15]; // start byte, command, data(12), checksum
	float red_color[]  = {1.0f, 0.0f, 0.0f}; // rgb
	float blue_color[] = {0.0f, 0.0f, 1.0f}; // rgb

	com_handle = openAndSetupComPort(TEXT("\\\\.\\COM93"));
	if(com_handle == INVALID_HANDLE_VALUE){
		printf("comport open failed\n");
		return 1;
	}
	printf("TSS_SET_LED_COLOR\n");

	// The data must be flipped to big endian before sending to sensor
	endian_swap_32((unsigned int *)&red_color[0]);
	endian_swap_32((unsigned int *)&red_color[1]);
	endian_swap_32((unsigned int *)&red_color[2]);
	
	write_bytes[0]= TSS_START_BYTE;
	write_bytes[1]= TSS_SET_LED_COLOR;
	memcpy(&write_bytes[2], red_color, sizeof(red_color)); // copy color data into write packet
	write_bytes[sizeof(write_bytes)-1]= createChecksum(&write_bytes[1], sizeof(red_color)+1);
	// Write the bytes to the serial
	if(!WriteFile(com_handle, write_bytes, sizeof(write_bytes), &bytes_written, 0)){
		printf("Error writing to port\n");
		return 2;
	}

	printf("The LED should be RED\n");
	Sleep(2000); // Wait period for the user to confirm LED color change

	// The data must be flipped to big endian before sending to sensor
	endian_swap_32((unsigned int *)&blue_color[0]);
	endian_swap_32((unsigned int *)&blue_color[1]);
	endian_swap_32((unsigned int *)&blue_color[2]);

	write_bytes[0]= TSS_START_BYTE;
	write_bytes[1]= TSS_SET_LED_COLOR;
	memcpy(&write_bytes[2], blue_color, sizeof(blue_color)); // copy color data into write packet
	write_bytes[sizeof(write_bytes)-1]= createChecksum(&write_bytes[1], sizeof(blue_color)+1);
	// Write the bytes to the serial
	if(!WriteFile(com_handle, write_bytes, sizeof(write_bytes), &bytes_written, 0)){
		printf("Error writing to port\n");
		return 3;
	}
	printf("The LED should be BLUE\n");
	// Close the serial
	CloseHandle(com_handle);
	printf("Finished press Enter to continue");
	getchar();
	return 0;
}