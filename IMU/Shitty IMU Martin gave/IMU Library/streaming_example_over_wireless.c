// A basic wireless streaming mode example
//
// The default mode of communication for a 3-Space Sensor is a call and response paradigm wherein you send a
// command and then receive a response. The sensor also features a streaming mode where it can be instructed to
// periodically send back the response from a command automatically, without any further communication from the host.
//
// This example code is for 3-Space Wireless sensors and dongles, not for Bluetooth
// The 3-Space Wireless sensor must be paired with a dongle prior to running this code
// Requires 2.0+ firmware features
#include "yei_threespace_basic_utils.h"
#include <Windows.h>
#include <stdio.h>

#define STREAM_DURATION 10 // Time to stream in seconds

// For a full list of streamable commands refer to "Wired Streaming Mode" section in the
// 3-Space Manual of your sensor
#define TSS_GET_TARED_ORIENTATION_AS_QUATERNION 0x00
#define TSS_GET_CORRECTED_LINEAR_ACCELERATION_IN_GLOBAL_SPACE 0x29
#define TSS_GET_BUTTON_STATE 0xfa
#define TSS_NULL 0xff
// For a full list of commands refer to the 3-Space Manual of your sensor
#define TSS_SET_WIRELESS_RESPONSE_HEADER_BITFIELD 0xdb
#define TSS_SET_STREAMING_SLOTS 0x50
#define TSS_SET_STREAMING_TIMING 0x52
#define TSS_START_STREAMING 0x55
#define TSS_STOP_STREAMING 0x56


// This structure must match the options in response_header_setup
#pragma pack(push,1)
typedef struct Response_Header{
    unsigned char success_failure;
    // unsigned int timestamp;	(Commented out as it is not being used)
    // unsigned char command_echo;	(Commented out as it is not being used)
	// unsigned char checksum;	(Commented out as it is not being used)
	unsigned char logical_id;
	// unsigned int serial_number;	(Commented out as it is not being used)
    unsigned char data_length;
} Response_Header;
#pragma pack(pop)

// Stream data stuctures must be packed else they will not properly work
#pragma pack(push,1)
typedef struct Batch_Data{
    float quaternion[4]; // xyzw
    float linear_acceleration[3]; // xyz
	unsigned char button_state; // bit 0 - button 0, bit 1 - button 1
} Batch_Data;
#pragma pack(pop)

// Streaming mode require the streaming slots and streaming timing being setup prior to start streaming
int setupStreamingWireless(HANDLE com_handle, int logical_id)
{
	DWORD bytes_written;
	DWORD bytes_read;
	Response_Header response_header;
	unsigned char write_slot_bytes[12]; // start byte, logical id, command, data(8), checksum
	unsigned char write_timing_bytes[16]; // start byte, logical id, command, data(12), checksum
	unsigned int interval;
	unsigned int duration;
	unsigned int delay;

	printf("TSS_SET_STREAMING_SLOTS\n");
	//There are 8 streaming slots available for use, and each one can hold one of these commands.
	//Unused slots should be filled with 0xff so that they will output nothing. 
	write_slot_bytes[0]= TSS_WIRELESS_RESPONSE_HEADER_START_BYTE;
	write_slot_bytes[1]= logical_id; 
	write_slot_bytes[2]= TSS_SET_STREAMING_SLOTS;
	write_slot_bytes[3]= TSS_GET_TARED_ORIENTATION_AS_QUATERNION; // stream slot0
	write_slot_bytes[4]= TSS_GET_CORRECTED_LINEAR_ACCELERATION_IN_GLOBAL_SPACE; // stream slot1
	write_slot_bytes[5]= TSS_GET_BUTTON_STATE; // stream slot2
	write_slot_bytes[6]= TSS_NULL; // stream slot3
	write_slot_bytes[7]= TSS_NULL; // stream slot4
	write_slot_bytes[8]= TSS_NULL; // stream slot5
	write_slot_bytes[9]= TSS_NULL; // stream slot6
	write_slot_bytes[10]=TSS_NULL; // stream slot7
	write_slot_bytes[11]= createChecksum(&write_slot_bytes[1], 8+2);
	// Write the bytes to the serial
	if(!WriteFile(com_handle, write_slot_bytes, sizeof(write_slot_bytes), &bytes_written, 0)){
		printf("Error writing to port\n");
		return 1;
	}

	// Read the bytes returned from the serial
	// This uses the programmable response header as setup in main
	if(!ReadFile(com_handle, &response_header, sizeof(response_header), &bytes_read, 0)){
		printf("Error reading from port\n");
		return 3;
	}
	if(response_header.success_failure != 0){
		printf("Failed to confirm TSS_SET_STREAMING_SLOTS set\n");
	}

	printf("TSS_SET_STREAMING_TIMING\n");
	// Interval determines how often the streaming session will output data from the requested commands
	// An interval of 0 will output data at the max filter rate
	interval= 0; // microseconds
	// Duration determines how long the streaming session will run for
	// A duration of 0xffffffff will have the streaming session run till the stop stream command is called
	duration= STREAM_DURATION*1000000; // microseconds
	// Delay determines how long the sensor should wait after a start command is issued to actually begin
    // streaming
	delay= 1000; //microseconds

	// The data must be flipped to big endian before sending to sensor
	endian_swap_32((unsigned int *)&interval);
	endian_swap_32((unsigned int *)&duration);
	endian_swap_32((unsigned int *)&delay);


	write_timing_bytes[0]= TSS_WIRELESS_RESPONSE_HEADER_START_BYTE;
	write_timing_bytes[1]= logical_id; 
	write_timing_bytes[2]= TSS_SET_STREAMING_TIMING;
	memcpy(&write_timing_bytes[3], &interval, sizeof(interval));
	memcpy(&write_timing_bytes[7], &duration, sizeof(duration));
	memcpy(&write_timing_bytes[11], &delay, sizeof(delay));
	write_timing_bytes[sizeof(write_timing_bytes)-1]= createChecksum(&write_timing_bytes[1], 12+2);
	// Write the bytes to the serial
	if(!WriteFile(com_handle, write_timing_bytes, sizeof(write_timing_bytes), &bytes_written, 0)){
		printf("Error writing to port\n");
		return 2;
	}
	// Read the bytes returned from the serial
	// This uses the programmable response header as setup in main
	if(!ReadFile(com_handle, &response_header, sizeof(response_header), &bytes_read, 0)){
		printf("Error reading from port\n");
		return 3;
	}
	if(response_header.success_failure != 0){
		printf("Failed to confirm TSS_SET_STREAMING_TIMING set\n");
	}

	return 0;
}

int main()
{
	HANDLE com_handle;
	DWORD bytes_written;
	DWORD bytes_read;
	unsigned int response_header_setup=0;
	unsigned char write_rh_bytes[7];
	Response_Header response_header;
	int logical_id = 0; // valid range is 0-14, 0 is the first sensor in the logical id table
	unsigned char write_stream_bytes[4];
	Batch_Data batch_data={0};
	int i;
	unsigned int sample_count=0;

	// High-resolution performance counter varibles, this is just a timer
	LARGE_INTEGER frequency; // counts per second
    LARGE_INTEGER start_time, current_time; // counts
    QueryPerformanceFrequency(&frequency); // gets performance-counter frequency, in counts per second

	com_handle = openAndSetupComPort(TEXT("\\\\.\\COM15"));
	if(com_handle == INVALID_HANDLE_VALUE){
		printf("comport open failed\n");
		return 1;
	}

	printf("TSS_SET_WIRED_RESPONSE_HEADER_BITFIELD\n");
	// Setting up the response header, note the size is a 32 bit int even tho only 3 bits are used right now
	response_header_setup =TSS_RH_SUCCESS_FAILURE;
	response_header_setup+=TSS_RH_LOGICAL_ID;
	response_header_setup+=TSS_RH_DATA_LENGTH;

	// The data must be flipped to big endian before sending to sensor
	endian_swap_32((unsigned int *)&response_header_setup);

	write_rh_bytes[0]= TSS_START_BYTE;
	write_rh_bytes[1]= TSS_SET_WIRELESS_RESPONSE_HEADER_BITFIELD;
	memcpy(&write_rh_bytes[2], &response_header_setup, sizeof(response_header_setup)); 
	write_rh_bytes[sizeof(write_rh_bytes)-1]= createChecksum(&write_rh_bytes[1], sizeof(response_header_setup)+1);
	// Write the bytes to the serial
	if(!WriteFile(com_handle, write_rh_bytes, sizeof(write_rh_bytes), &bytes_written, 0)){
		printf("Error writing to port\n");
		return 2;
	}

	if(setupStreamingWireless(com_handle, logical_id)){
		printf("setup streaming failed\n");
		return 2;
	}

	printf("TSS_START_STREAMING\n");

	// Wireless commands take a logical id in addition to the usual wired commands
	write_stream_bytes[0]= TSS_WIRELESS_RESPONSE_HEADER_START_BYTE;
	write_stream_bytes[1]= logical_id;
	write_stream_bytes[2]= TSS_START_STREAMING;
	write_stream_bytes[3]= createChecksum(&write_stream_bytes[1], 2);
	// Write the bytes to the serial
	if(!WriteFile(com_handle, write_stream_bytes, sizeof(write_stream_bytes), &bytes_written, 0)){
		printf("Error writing to port\n");
		return 3;
	}
	// Read the bytes returned from the serial
	// This uses the programmable response header as setup in main
	if(!ReadFile(com_handle, &response_header, sizeof(response_header), &bytes_read, 0)){
		printf("Error reading from port\n");
		return 3;
	}
	if(response_header.success_failure != 0){
		printf("Failed to confirm TSS_START_STREAMING set\n");
	}
	// If TSS_START_STREAMING suceeds and the sensor starts blinking but there is no data being received,
	// the dongle may have pause streaming enabled
	// Write TSS_START_STREAMING to the dongle using the wired start byte like in the "streaming example"

	QueryPerformanceCounter(&start_time); // Retrieves the current value of the high-resolution performance counter.
	QueryPerformanceCounter(&current_time);  // Retrieves the current value of the high-resolution performance counter.
	// Loop runs the length of STREAM_DURATION
	while((float)STREAM_DURATION > ((current_time.QuadPart-start_time.QuadPart)*1.0f/frequency.QuadPart)){
		QueryPerformanceCounter(&current_time);  // Retrieves the current value of the high-resolution performance counter.
		// Read the bytes returned from the serial
		if(!ReadFile(com_handle, &response_header, sizeof(response_header), &bytes_read, 0)){
			printf("Error reading from port\n");
			return 3;
		}
		if(bytes_read !=sizeof(response_header)){ // lost stream packets or end of stream
			continue;
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
		printf("========================================================\n");
		printf("Quaternion:   % 8.5f, % 8.5f, % 8.5f, % 8.5f\n",	batch_data.quaternion[0], 
																batch_data.quaternion[1], 
																batch_data.quaternion[2],
																batch_data.quaternion[3]);

		printf("Linear Accel: % 8.5f, % 8.5f, % 8.5f\n",	batch_data.linear_acceleration[0], 
															batch_data.linear_acceleration[1], 
															batch_data.linear_acceleration[2]);
		printf("Button State:  %u\n", batch_data.button_state);
		sample_count++;
	}
	printf("Sample Count=%u\n", sample_count);
    // Close the serial
	CloseHandle(com_handle);
	printf("Finished press Enter to continue");
	getchar();
	return 0;
}