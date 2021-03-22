// A basic set LED color example over wireless
//
// This example code is for 3-Space Wireless sensors and dongles, not for Bluetooth
// The 3-Space Wireless sensor must be paired with a dongle prior to running this code
#include "yei_threespace_basic_utils.h"
#include <Windows.h>
#include <stdio.h>
// For a full list of commands refer to the 3-Space Manual of your sensor
#define TSS_SET_LED_COLOR 0xee

int main()
{
	HANDLE com_handle;
	DWORD bytes_written;
	DWORD bytes_read;
	unsigned char write_bytes[16]; // start byte, logical id, command, data(12), checksum
	unsigned char response_header[2];// failbyte, logical id
	unsigned char response_data_size;// return data size, only returned with sucsessful responces
	float red_color[]  = {1.0f, 0.0f, 0.0f};// rgb
	float blue_color[] = {0.0f, 0.0f, 1.0f};// rgb
	int logical_id = 0; // valid range is 0-14, 0 is the first sensor in the logical id table

	com_handle = openAndSetupComPort(TEXT("\\\\.\\COM15"));
	if(com_handle == INVALID_HANDLE_VALUE){
		printf("comport open failed\n");
		return 1;
	}
	printf("TSS_SET_LED_COLOR\n");

	// The data must be flipped to big endian before sending to sensor
	endian_swap_32((unsigned int *)&red_color[0]);
	endian_swap_32((unsigned int *)&red_color[1]);
	endian_swap_32((unsigned int *)&red_color[2]);
	
	// Wireless commands take a logical id in addition to the usual wired protocol
	write_bytes[0]= TSS_WIRELESS_START_BYTE;
	write_bytes[1]= logical_id; 
	write_bytes[2]= TSS_SET_LED_COLOR;
	memcpy(&write_bytes[3], red_color, sizeof(red_color)); // copy color data into write packet
	write_bytes[sizeof(write_bytes)-1]= createChecksum(&write_bytes[1], sizeof(red_color)+2);
	// Write the bytes to the serial
	if(!WriteFile(com_handle, write_bytes, sizeof(write_bytes), &bytes_written, 0)){
		printf("Error writing to port\n");
		return 2;
	}

	// Read the bytes returned from the serial
	// Response header contains the failure byte and the logical id.
	// If success, an additional size byte is also returned followed by the data.
	if(!ReadFile(com_handle, response_header, sizeof(response_header), &bytes_read, 0)){
		printf("Error reading from port\n");
		return 3;
	}
	if(response_header[0] == 0){ // successful send and receive 
		if(!ReadFile(com_handle, &response_data_size, 1, &bytes_read, 0)){
			printf("Error reading from port\n");
			return 4;
		}
		printf("The LED should be RED\n");
		Sleep(2000); // wait period for the user to confirm LED color change
	}
	else{
		printf("Failed to confirm LED being set to RED\n");
	}

	// The data must be flipped to big endian before sending to sensor
	endian_swap_32((unsigned int *)&blue_color[0]);
	endian_swap_32((unsigned int *)&blue_color[1]);
	endian_swap_32((unsigned int *)&blue_color[2]);
	
	// Wireless commands take a logical id in addition to the usual wired commands
	write_bytes[0]= TSS_WIRELESS_START_BYTE;
	write_bytes[1]= logical_id; 
	write_bytes[2]= TSS_SET_LED_COLOR;
	memcpy(&write_bytes[3], blue_color, sizeof(blue_color)); //copy color data into write packet
	write_bytes[sizeof(write_bytes)-1]= createChecksum(&write_bytes[1], sizeof(blue_color)+2);
	// Write the bytes to the serial
	if(!WriteFile(com_handle, write_bytes, sizeof(write_bytes), &bytes_written, 0)){
		printf("Error writing to port\n");
		return 5;
	}

	// Read the bytes returned from the serial
	// Response header contains the failure byte and the logical id.
	// If success, an additional size byte is also returned followed by the data.
	if(!ReadFile(com_handle, response_header, sizeof(response_header), &bytes_read, 0)){
		printf("Error reading from port\n");
		return 6;
	}
	if(response_header[0] == 0){ // successful send and receive 
		if(!ReadFile(com_handle, &response_data_size, 1, &bytes_read, 0)){
			printf("Error reading from port\n");
			return 7;
		}
		printf("The LED should be BLUE\n");
	}
	else{
		printf("Failed to confirm LED being set to BLUE\n");
	}
	// Close the serial
	CloseHandle(com_handle);
	printf("Finished press Enter to continue");
	getchar();
	return 0;
}