// This is a helpful paper for starting out with serial on Windows
// http://www.robbayer.com/files/serial-win.pdf
// The next one is a bit more complicated and not used in the examples uses overlapped io
// http://msdn.microsoft.com/en-us/library/ms810467.aspx
#include <Windows.h>
#include <stdio.h>

// Start Bytes, indicate the start of a command packet
#define TSS_START_BYTE 0xf7
#define TSS_WIRELESS_START_BYTE 0xf8
#define TSS_RESPONSE_HEADER_START_BYTE 0xf9
#define TSS_WIRELESS_RESPONSE_HEADER_START_BYTE 0xfa


// Response Header bit options 
// Details and descriptions are in the YEI 3-Space Manual under "Wired Response Header"
#define TSS_RH_SUCCESS_FAILURE 0x01
#define TSS_RH_TIMESTAMP 0x02
#define TSS_RH_COMMAND_ECHO 0x04
#define TSS_RH_CHECKSUM 0x08
#define TSS_RH_LOGICAL_ID 0x10
#define TSS_RH_SERIAL_NUMBER 0x20
#define TSS_RH_DATA_LENGTH 0x40


// The 3-Space sensors are Big Endian and x86 is Little Endian
// So the bytes need be swapped around, this function can convert from and
// to big endian
void endian_swap_16(unsigned short * x)
{
    *x = (*x>>8) |
        (*x<<8);
}

void endian_swap_32(unsigned int * x)
{
    *x = (*x>>24) |
        ((*x<<8) & 0x00FF0000) |
        ((*x>>8) & 0x0000FF00) |
         (*x<<24);
}

// This is a convenience function to calculate the last byte in the packet
// Commands without parameters can use the same number as the command
// \param command_bytes The address of the array to sum, this does not include the start byte
// \param num_bytes The number of bytes in the array Data + 1 for wired. Data + 2 for wireless
// \return checksum
unsigned char createChecksum(unsigned char * command_bytes, const unsigned int num_bytes)
{
	unsigned int chkSum = 0;
	unsigned int i;
	for (i = 0; i < num_bytes; i++){
		chkSum += command_bytes[i];
	}
	return (unsigned char)(chkSum % 256);
}

// This creates the comport handle and does initial setup like baudrates and timeouts
// \param comport The path to the port, comports higher than 8 require \\\\.\\ prepended to open
// \return com_handle If the open or configuaration fails, INVALID_HANDLE_VALUE is returned
HANDLE openAndSetupComPort(const TCHAR* comport)
{
	HANDLE com_handle;
	DCB dcb_serial_params = {0};
	COMMTIMEOUTS timeouts = {0};
	DWORD com_error;
	COMSTAT comstat;

	com_handle = CreateFile(comport, 
							GENERIC_READ | GENERIC_WRITE, 
							0, 
							0, 
							OPEN_EXISTING,
							FILE_ATTRIBUTE_NORMAL,
							0);
	if (com_handle == INVALID_HANDLE_VALUE){
		printf ("Error opening port\n");
		return INVALID_HANDLE_VALUE;
	}

	// Setting up the baud rate
	dcb_serial_params.DCBlength = sizeof(dcb_serial_params);
	if (!GetCommState(com_handle, &dcb_serial_params)){
		printf ("Failed to get the previous state of the serial port\n");
		CloseHandle(com_handle);
		return INVALID_HANDLE_VALUE;
	}
	dcb_serial_params.BaudRate = 115200; // default baud rate for 3-Space Sensors
	dcb_serial_params.ByteSize = 8;
	dcb_serial_params.StopBits = ONESTOPBIT;
	dcb_serial_params.Parity = NOPARITY;
	if(!SetCommState(com_handle, &dcb_serial_params)){
		printf("Failed to set the serial port's state\n");
	}
	// Setting the timeouts, tweaking these can improve reliability
	// Bluetooth may need longer timeouts with some adapters
    timeouts.ReadIntervalTimeout = -1;
    timeouts.ReadTotalTimeoutConstant = 100;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 100;
    timeouts.WriteTotalTimeoutMultiplier = 10;
	if(!SetCommTimeouts(com_handle, &timeouts)){
		printf( "Failed to set the timeouts\n" );
		CloseHandle(com_handle);
		return INVALID_HANDLE_VALUE;
	}

	// Flush out any data that was on the line from a previous session
	Sleep(300);
	ClearCommError(com_handle, &com_error, &comstat);
	if(comstat.cbInQue != 0){
		PurgeComm(com_handle,PURGE_RXCLEAR|PURGE_TXCLEAR);
	}

	return com_handle;
}