// An example script to demonstrate streaming from multiple wireless sensors and synchronizing their timestamps
//
// This example code is for 3-Space Wireless sensors and dongles, not for Bluetooth
// The 3-Space Wireless sensors must be paired with a dongle prior to running this code
// Requires 2.0+ firmware features
#include "yei_threespace_basic_utils.h"
#include <Windows.h>
#include <stdio.h>

#define STREAM_DURATION 10 // Time to stream in seconds

// For a full list of streamable commands refer to the "Wired Streaming Mode" section in the 
// 3-Space Manual of your sensor
#define TSS_GET_TARED_ORIENTATION_AS_QUATERNION 0x00
#define TSS_GET_CORRECTED_LINEAR_ACCELERATION_IN_GLOBAL_SPACE 0x29
#define TSS_GET_BUTTON_STATE 0xfa
#define TSS_NULL 0xff // No command, use to fill the empty slots in "set stream slots"
// For a full list of commands refer to the 3-Space Manual of your sensor
#define TSS_SET_WIRELESS_RESPONSE_HEADER_BITFIELD 0xdb
#define TSS_SET_STREAMING_SLOTS 0x50
#define TSS_SET_STREAMING_TIMING 0x52
#define TSS_START_STREAMING 0x55
#define TSS_STOP_STREAMING 0x56
#define TSS_BROADCAST_SYNCHRONIZATION_PULSE 0xb6


// This structure must match the options in response_header_setup
#pragma pack(push,1)
typedef struct Response_Header{
    unsigned char success_failure;
    unsigned int timestamp;
    // unsigned char command_echo;	//(Commented out as it is not being used)
	// unsigned char checksum;	//(Commented out as it is not being used)
	unsigned char logical_id;
	// unsigned int serial_number;	//(Commented out as it is not being used)
    unsigned char data_length;
} Response_Header;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct Batch_Data{
    float quaternion[4];
} Batch_Data;
#pragma pack(pop)

// Streaming mode requires the streaming slots and streaming timing to be setup prior to starting streaming
int setupStreamingWireless(HANDLE com_handle, int logical_id)
{
	DWORD bytes_written;
	DWORD bytes_read;
	Response_Header response_header;
	unsigned char write_slot_bytes[12];   // start byte, logical id, command, data(8), checksum
	unsigned char write_timing_bytes[16]; // start byte, logical id, command, data(12), checksum
	unsigned int interval;
	unsigned int duration;
	unsigned int delay;

	printf("TSS_SET_STREAMING_SLOTS\n");
	// There are 8 streaming slots available for use, and each one can hold one of the streamable commands.
	// Unused slots should be filled with 0xff so that they will output nothing. 
	write_slot_bytes[0]= TSS_WIRELESS_RESPONSE_HEADER_START_BYTE;
	write_slot_bytes[1]= logical_id; 
	write_slot_bytes[2]= TSS_SET_STREAMING_SLOTS;
	write_slot_bytes[3]= TSS_GET_TARED_ORIENTATION_AS_QUATERNION; // stream slot0
	write_slot_bytes[4]= TSS_NULL; // stream slot1
	write_slot_bytes[5]= TSS_NULL; // stream slot2
	write_slot_bytes[6]= TSS_NULL; // stream slot3
	write_slot_bytes[7]= TSS_NULL; // stream slot4
	write_slot_bytes[8]= TSS_NULL; // stream slot5
	write_slot_bytes[9]= TSS_NULL; // stream slot6
	write_slot_bytes[10]= TSS_NULL; // stream slot7
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
	// When starting stream on multiple sensors adding a delay gives you enough time to send the start
	// command to all the sensors before streaming begins
	delay= 50000; // microseconds

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
		printf("Failed to confirm TSS_SET_STREAMING_SLOTS set\n");
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
	unsigned char write_pulse_bytes[3];
	Response_Header response_header;
	int logical_id00 = 0; // valid range is 0-14, 0 is the first sensor in the logical id table
	int logical_id01 = 1; // valid range is 0-14, 1 is the second sensor in the logical id table
	unsigned char write_stream_bytes[4];
	Batch_Data batch_data={0};
	int i;
	unsigned int sample_count00 =0; // the amount of packets received 
	unsigned int sample_count01 =0; // the amount of packets received 

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
	// Setting up the response header, note the size is a 32 bit integer even though only 7 bits are used right now
	response_header_setup =TSS_RH_SUCCESS_FAILURE;
	response_header_setup+=TSS_RH_TIMESTAMP;
	response_header_setup+=TSS_RH_LOGICAL_ID;
	response_header_setup+=TSS_RH_DATA_LENGTH;

	// The data must be flipped to big endian before sending to sensor
	endian_swap_32((unsigned int *)&response_header_setup);

	write_rh_bytes[0]= TSS_START_BYTE;
	write_rh_bytes[1]= TSS_SET_WIRELESS_RESPONSE_HEADER_BITFIELD;
	memcpy(&write_rh_bytes[2], &response_header_setup, sizeof(response_header_setup)); 
	write_rh_bytes[sizeof(write_rh_bytes)-1]= createChecksum(&write_rh_bytes[1], sizeof(response_header_setup)+1);
	// Write the bytes to the serial
	if(!WriteFile(com_handle, write_rh_bytes, sizeof(write_rh_bytes), &bytes_written,0)){
		printf("Error writing to port\n");
		return 2;
	}

	// Set up streaming for both sensors
	if(setupStreamingWireless(com_handle, logical_id00)){
		printf("setup streaming failed to logical_id %u\n", logical_id00);
		return 3;
	}

	if(setupStreamingWireless(com_handle, logical_id01)){
		printf("setup streaming failed to logical_id %u\n", logical_id01);
		return 4;
	}

	printf("TSS_BROADCAST_SYNCHRONIZATION_PULSE\n");
	// Sends out a timestamp synchronization broadcast message to all wireless sensors that are listening on the same Channel and PanID as the dongle
	write_pulse_bytes[0]= TSS_START_BYTE;
	write_pulse_bytes[1]= TSS_BROADCAST_SYNCHRONIZATION_PULSE;
	write_pulse_bytes[2]= TSS_BROADCAST_SYNCHRONIZATION_PULSE;
	// Write the bytes to the serial
	// This command will pulse a timestamp a few times but there is no return to guarantee that all the sensors received the pulse
	if(!WriteFile(com_handle, write_pulse_bytes, sizeof(write_pulse_bytes), &bytes_written, 0)){
		printf("Error writing to port\n");
		return 5;
	}
	// To ensure all the sensors received the timestamp, send the command a second time
	if(!WriteFile(com_handle, write_pulse_bytes, sizeof(write_pulse_bytes), &bytes_written, 0)){
		printf("Error writing to port\n");
		return 5;
	}
	// A brief wait to make sure the timestamp has been applied
	Sleep(30);

	printf("TSS_START_STREAMING\n");
	// Wireless commands take a logical id in addition to the usual wired commands
	write_stream_bytes[0]= TSS_WIRELESS_RESPONSE_HEADER_START_BYTE;
	write_stream_bytes[1]= logical_id00;
	write_stream_bytes[2]= TSS_START_STREAMING;
	write_stream_bytes[3]= createChecksum(&write_stream_bytes[1], 2);
	// Write the bytes to the serial
	if(!WriteFile(com_handle, write_stream_bytes, sizeof(write_stream_bytes), &bytes_written,0)){
		printf("Error writing to port\n");
		return 3;
	}

	// Wireless commands take a logical id in addition to the usual wired commands
	write_stream_bytes[0]= TSS_WIRELESS_RESPONSE_HEADER_START_BYTE;
	write_stream_bytes[1]= logical_id01;
	write_stream_bytes[2]= TSS_START_STREAMING;
	write_stream_bytes[3]= createChecksum(&write_stream_bytes[1], 2);
	// Write the bytes to the serial
	if(!WriteFile(com_handle, write_stream_bytes, sizeof(write_stream_bytes), &bytes_written,0)){
		printf("Error writing to port\n");
		return 3;
	}
	// Read the bytes returned from the serial for logical_id00
	if(!ReadFile(com_handle, &response_header, sizeof(response_header), &bytes_read, 0)){
		printf("Error reading from port\n");
		return 3;
	}
	if(response_header.success_failure != 0){
		printf("Failed to confirm TSS_START_STREAMING set\n");
	}
	// Read the bytes returned from the serial for logical_id01
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
		QueryPerformanceCounter(&current_time); // Retrieves the current value of the high-resolution performance counter.
		// Read the bytes returned from the serial
		if(!ReadFile(com_handle, &response_header, sizeof(response_header), &bytes_read, 0)){
			printf("Error reading from port\n");
			return 3;
		}
		if(bytes_read !=sizeof(response_header)){ // lost stream packets or end of stream
			continue;
		}

		if(!ReadFile(com_handle, &batch_data, sizeof(batch_data), &bytes_read, 0)){
			printf("Error reading from port\n");
			return 3;
		}

		printf("========================================================\n");
		// Based on the response header the data can be sorted 
		if(response_header.logical_id == logical_id00){
			sample_count00++;
		}
		else if(response_header.logical_id == logical_id01){
			sample_count01++;
		}
		// The data must be fliped to little endian to be read correctly
		endian_swap_32((unsigned int *)&response_header.timestamp);
		printf("Sensor:%u T:%u ",
			response_header.logical_id,
			response_header.timestamp);

		// The data must be fliped to little endian to be read correctly
		for( i=0; i< sizeof(batch_data.quaternion)/sizeof(float); i++){
			endian_swap_32((unsigned int *)&batch_data.quaternion[i]);
		}
		printf("Q:  % 8.5f, % 8.5f, % 8.5f, % 8.5f\n",	batch_data.quaternion[0], 
														batch_data.quaternion[1], 
														batch_data.quaternion[2],
														batch_data.quaternion[3]);
	}
	printf("Sample Count 0=%u\n", sample_count00);
	printf("Sample Count 1=%u\n", sample_count01);
	// Close the serial
	CloseHandle(com_handle);
	printf("Finished press Enter to continue");
	getchar();
	return 0;
}