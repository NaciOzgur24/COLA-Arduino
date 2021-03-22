// A basic example demonstrating the different filter modes (QGrad, Kalman, IMU)
// To show the sensors running at full speed, streaming is used to reduce the overhead of polling
//
// This example code is compatible with all 3-Space Sensors connected via USB, serial, or Bluetooth
// Requires 2.0+ firmware features
#include "yei_threespace_basic_utils.h"
#include <Windows.h>
#include <stdio.h>

#define STREAM_DURATION 1 // Time to stream in seconds
// For a full list of streamable commands refer to the "Wired Streaming Mode" section in the 
// 3-Space Manual of your sensor
#define TSS_GET_TARED_ORIENTATION_AS_QUATERNION 0x00
#define TSS_GET_ALL_CORRECTED_COMPONENT_SENSOR_DATA 0x25
#define TSS_NULL 0xff // No command use to fill the empty slots in "set stream slots"
// For a full list of commands refer to the 3-Space Manual of your sensor
#define TSS_SET_WIRELESS_RESPONSE_HEADER_BITFIELD 0xdb
#define TSS_SET_STREAMING_SLOTS 0x50
#define TSS_SET_STREAMING_TIMING 0x52
#define TSS_START_STREAMING 0x55
#define TSS_STOP_STREAMING 0x56
#define TSS_SET_FILTER_MODE 0x7b

// The filter modes the current firmware supports
typedef enum TSS_Filter_Mode
{
    TSS_FILTER_IMU, // orientations are not updated in this mode
    TSS_FILTER_KALMAN,
    TSS_FILTER_ALTERNATING_KALMAN,
    TSS_FILTER_COMPLEMENTARY,
    TSS_FILTER_QUATERNION_GRADIENT_DECENT,
}TSS_Filter_Mode;

// Streaming mode requires the streaming slots and streaming timing to be setup prior to starting streaming
// This function also exposes the first slot in stream slots to swap the stream command
int setupStreamingAdvanced(HANDLE com_handle, unsigned char stream_cmd)
{
	DWORD bytes_written;
	unsigned char write_slot_bytes[11];   // start byte, command, data(8), checksum
	unsigned char write_timing_bytes[15]; // start byte, command, data(12), checksum
	unsigned int interval;
	unsigned int duration;
	unsigned int delay;

	printf("TSS_SET_STREAMING_SLOTS\n");
	// There are 8 streaming slots available for use, and each one can hold one of the streamable commands.
	// Unused slots should be filled with 0xff so that they will output nothing. 
	write_slot_bytes[0]= TSS_START_BYTE;
	write_slot_bytes[1]= TSS_SET_STREAMING_SLOTS;
	write_slot_bytes[2]= stream_cmd; //stream slot0
	write_slot_bytes[3]= TSS_NULL; //stream slot1
	write_slot_bytes[4]= TSS_NULL; //stream slot2
	write_slot_bytes[5]= TSS_NULL; //stream slot3
	write_slot_bytes[6]= TSS_NULL; //stream slot4
	write_slot_bytes[7]= TSS_NULL; //stream slot5
	write_slot_bytes[8]= TSS_NULL; //stream slot6
	write_slot_bytes[9]= TSS_NULL; //stream slot7
	write_slot_bytes[10]= createChecksum(&write_slot_bytes[1], 8+1);
	// Write the bytes to the serial
	if(!WriteFile(com_handle, write_slot_bytes, sizeof(write_slot_bytes), &bytes_written, 0)){
		printf("Error writing to port\n");
		return 1;
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
	delay= 0; // microseconds

	// The data must be flipped to big endian before sending to sensor
	endian_swap_32((unsigned int *)&interval);
	endian_swap_32((unsigned int *)&duration);
	endian_swap_32((unsigned int *)&delay);

	write_timing_bytes[0]= TSS_START_BYTE;
	write_timing_bytes[1]= TSS_SET_STREAMING_TIMING;
	memcpy(&write_timing_bytes[2], &interval, sizeof(interval));
	memcpy(&write_timing_bytes[6], &duration, sizeof(duration));
	memcpy(&write_timing_bytes[10], &delay, sizeof(delay));
	write_timing_bytes[sizeof(write_timing_bytes)-1]= createChecksum(&write_timing_bytes[1], 12+1);
	// Write the bytes to the serial
	if(!WriteFile(com_handle, write_timing_bytes, sizeof(write_timing_bytes), &bytes_written, 0)){
		printf("Error writing to port\n");
		return 2;
	}
	return 0;
}

int main()
{
	HANDLE com_handle;
	DWORD bytes_written;
	DWORD bytes_read;
	unsigned char write_filter_bytes[4]; // start byte, command, data, checksum
	unsigned char write_stream_bytes[3]; // start byte, command, checksum
	float quat[4]; // quaternion xyzw
	float data[9]; // gyro xyz, accel xyz, compass xyz
	int i;
	unsigned int sample_count_qgrad,sample_count_kalman, sample_count_imu =0;
	LARGE_INTEGER frequency; // ticks per second
    LARGE_INTEGER start_time, current_time; // ticks

    QueryPerformanceFrequency(&frequency);

	com_handle = openAndSetupComPort(TEXT("\\\\.\\COM93"));
	if(com_handle == INVALID_HANDLE_VALUE){
		printf("comport open failed\n");
		return 1;
	}

	if(setupStreamingAdvanced(com_handle, TSS_GET_TARED_ORIENTATION_AS_QUATERNION)){
		printf("setup streaming failed\n");
		return 2;
	}

	write_filter_bytes[0]= TSS_START_BYTE;
	write_filter_bytes[1]= TSS_SET_FILTER_MODE;
	write_filter_bytes[2]= TSS_FILTER_QUATERNION_GRADIENT_DECENT;
	write_filter_bytes[3]= createChecksum(&write_filter_bytes[1], 2);

	if(!WriteFile(com_handle, write_filter_bytes, sizeof(write_filter_bytes), &bytes_written, 0)){
		printf("Error writing to port\n");
		return 3;
	}

	Sleep(10); // A brief wait to ensure the filter has been changed before streaming

	printf("TSS_START_STREAMING\n");
	// With parameterless wired commands the command byte will be the same as the checksum
	// Wireless commands take a logical id in addition to the usual wired commands
	write_stream_bytes[0]= TSS_START_BYTE;
	write_stream_bytes[1]= TSS_START_STREAMING;
	write_stream_bytes[2]= TSS_START_STREAMING;

	if(!WriteFile(com_handle, write_stream_bytes, sizeof(write_stream_bytes), &bytes_written, 0)){
		printf("Error writing to port\n");
		return 3;
	}

	sample_count_qgrad=0;
	QueryPerformanceCounter(&start_time);
	QueryPerformanceCounter(&current_time);
	while((float)STREAM_DURATION > ((current_time.QuadPart-start_time.QuadPart)*1.0f/frequency.QuadPart)){
		QueryPerformanceCounter(&current_time);
		if(!ReadFile(com_handle, quat, sizeof(quat), &bytes_read, 0)){
			printf("Error reading from port\n");
			return 3;
		}
		if(bytes_read !=sizeof(quat)){ // lost stream packets or end of stream
			printf("FIN\n");
			continue;
		}

		// The data must be fliped to little endian to be read correctly
		for(i=0; i< sizeof(quat)/sizeof(float); i++){
			endian_swap_32((unsigned int *)&quat[i]);
		}
		printf("========================================================\n");
		printf("Quaternion:  % 8.5f, % 8.5f, % 8.5f, % 8.5f\n",	quat[0], 
																quat[1], 
																quat[2],
																quat[3]);
		sample_count_qgrad++;
	}
	printf("Sample Count=%u\n", sample_count_qgrad);

	// Flush out any data that was on the line from the previous session that wasn't processed
	Sleep(100);
	PurgeComm(com_handle,PURGE_RXCLEAR|PURGE_TXCLEAR);

	///////////////////////////////////////////////////////////////////////////

	write_filter_bytes[0]= TSS_START_BYTE;
	write_filter_bytes[1]= TSS_SET_FILTER_MODE;
	write_filter_bytes[2]= TSS_FILTER_KALMAN;
	write_filter_bytes[3]= createChecksum(&write_filter_bytes[1], 2);

	if(!WriteFile(com_handle, write_filter_bytes, sizeof(write_filter_bytes), &bytes_written, 0)){
		printf("Error writing to port\n");
		return 3;
	}

	Sleep(10); // A brief wait to ensure the filter has been changed before streaming

	printf("TSS_START_STREAMING\n");
	// With parameterless wired commands the command byte will be the same as the checksum
	// Wireless commands take a logical id in addition to the usual wired commands
	write_stream_bytes[0]= TSS_START_BYTE;
	write_stream_bytes[1]= TSS_START_STREAMING;
	write_stream_bytes[2]= TSS_START_STREAMING;

	if(!WriteFile(com_handle, write_stream_bytes, sizeof(write_stream_bytes), &bytes_written, 0)){
		printf("Error writing to port\n");
		return 3;
	}

	sample_count_kalman=0;
	QueryPerformanceCounter(&start_time);
	QueryPerformanceCounter(&current_time);
	while((float)STREAM_DURATION > ((current_time.QuadPart-start_time.QuadPart)*1.0f/frequency.QuadPart)){
		QueryPerformanceCounter(&current_time);
		if(!ReadFile(com_handle, quat, sizeof(quat), &bytes_read, 0)){
			printf("Error reading from port\n");
			return 3;
		}
		if(bytes_read !=sizeof(quat)){ // lost stream packets or end of stream
			continue;
		}

		// The data must be fliped to little endian to be read correctly
		for(i=0; i< sizeof(quat)/sizeof(float); i++){
			endian_swap_32((unsigned int *)&quat[i]);
		}
		printf("========================================================\n");
		printf("Quaternion:  % 8.5f, % 8.5f, % 8.5f, % 8.5f\n",	quat[0], 
																quat[1], 
																quat[2],
																quat[3]);
		sample_count_kalman++;
	}
	printf("Sample Count=%u\n", sample_count_kalman);

	// Flush out any data that was on the line from the previous session that wasn't processed
	Sleep(100);
	PurgeComm(com_handle,PURGE_RXCLEAR|PURGE_TXCLEAR);

	/////////////////////////////////////////////////////////////////////////////////

	if(setupStreamingAdvanced(com_handle, TSS_GET_ALL_CORRECTED_COMPONENT_SENSOR_DATA)){
		printf("setup streaming failed\n");
		return 2;
	}

	write_filter_bytes[0]= TSS_START_BYTE;
	write_filter_bytes[1]= TSS_SET_FILTER_MODE;
	write_filter_bytes[2]= TSS_FILTER_IMU;
	write_filter_bytes[3]= createChecksum(&write_filter_bytes[1], 2);

	if(!WriteFile(com_handle, write_filter_bytes, sizeof(write_filter_bytes), &bytes_written, 0)){
		printf("Error writing to port\n");
		return 3;
	}

	Sleep(10); // A brief wait to ensure the filter has been changed before streaming

	printf("TSS_START_STREAMING\n");
	// With parameterless wired commands the command byte will be the same as the checksum
	// Wireless commands take a logical id in addition to the usual wired commands
	write_stream_bytes[0]= TSS_START_BYTE;
	write_stream_bytes[1]= TSS_START_STREAMING;
	write_stream_bytes[2]= TSS_START_STREAMING;

	if(!WriteFile(com_handle, write_stream_bytes, sizeof(write_stream_bytes), &bytes_written, 0)){
		printf("Error writing to port\n");
		return 3;
	}

	sample_count_imu=0;
	QueryPerformanceCounter(&start_time);
	QueryPerformanceCounter(&current_time);
	while((float)STREAM_DURATION > ((current_time.QuadPart-start_time.QuadPart)*1.0f/frequency.QuadPart)){
		QueryPerformanceCounter(&current_time);
		if(!ReadFile(com_handle, data, sizeof(data), &bytes_read, 0)){
			printf("Error reading from port\n");
			return 3;
		}
		if(bytes_read !=sizeof(data)){ // lost stream packets or end of stream
			continue;
		}

		// The data must be fliped to little endian to be read correctly
		for(i=0; i< sizeof(data)/sizeof(float); i++){
			endian_swap_32((unsigned int *)&data[i]);
		}
		printf("========================================================\n");
		printf("Gyro:  % 8.5f, % 8.5f, % 8.5f\n",	data[0],
												data[1],
												data[2]);
		printf("Accel: % 8.5f, % 8.5f, % 8.5f\n",	data[3],
												data[4],
												data[5]);
		printf("Comp:  % 8.5f, % 8.5f, % 8.5f\n",	data[6],
												data[7],
												data[8]);
		sample_count_imu++;
	}
	printf("Sample Count Qgrad=%u\n", sample_count_qgrad);
	printf("Sample Count Kalman=%u\n", sample_count_kalman);
	printf("Sample Count IMU=%u\n", sample_count_imu);

	// Close the serial
	CloseHandle(com_handle);
	printf("Finished press Enter to continue");
	getchar();
	return 0;
}