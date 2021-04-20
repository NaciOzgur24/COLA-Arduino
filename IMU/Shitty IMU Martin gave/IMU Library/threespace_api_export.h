/********************************************//**
* \ingroup threespace_api
* \file threespace_api_export.h
* \section Details
* \author Chris George
* \author Steve Landers
* \author Travis Lynn
* \author Nick Leyder
* \author Keith Webb
* \version YOST 3-Space API 1.0.0
* \copyright Copyright 1998-2017, Yost Labs Corporation
*
* \brief This file is a collection of convenience functions for using the Yost 3-Space devices
* for use in a program written in C/C++ or any language that can import a compiled library (.dll, .so, etc).
*
* The Yost 3-Space API is released under the Yost 3-Space Open Source License, which allows for both
* non-commercial use and commercial use with certain restrictions.
*
* For Non-Commercial Use, your use of Covered Works is governed by the GNU GPL v.3, subject to the Yost 3-Space Open
* Source Licensing Overview and Definitions.
*
* For Commercial Use, a Yost Commercial/Redistribution License is required, pursuant to the Yost 3-Space Open Source
* Licensing Overview and Definitions. Commercial Use, for the purposes of this License, means the use, reproduction
* and/or Distribution, either directly or indirectly, of the Covered Works or any portion thereof, or a Compilation,
* Improvement, or Modification, for Pecuniary Gain. A Yost Commercial/Redistribution License may or may not require
* payment, depending upon the intended use.
*
* Full details of the Yost 3-Space Open Source License can be found in license.txt
* License also available online at http://www.yeitechnology.com/yei-3-space-open-source-license
***********************************************/
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif
	//useful types
	typedef uint8_t  U8;
	typedef uint16_t U16;
	typedef uint32_t U32;
	typedef int32_t  I32;

#ifndef __cplusplus
	typedef U8 bool;
	#define true 1
	#define false 0
#endif

	/********************************************//**
	* \ingroup tss_api
	* Yost 3-Space API device identifier, a common parameter needed for all 3-Space API calls.
	***********************************************/
	typedef U32 tss_device_id;

	/********************************************//**
	* \ingroup tss_api
	* An enum expressing the different types of errors a 3-Space API call can return.
	***********************************************/
	enum TSS_ERROR
	{
		TSS_NO_ERROR = 0,									/**< The API call successfully executed */
		TSS_ERROR_NOT_ENOUGH_DATA = 1,						/**< There wasn't enough data necessary to execute the command to the intended device */
		TSS_ERROR_CANT_OPEN_PORT = 2,						/**< The API cannot open the communication port */
		TSS_ERROR_NOT_AVAILABLE_WIRELESS = 3,				/**< The command is does not work over wireless */
		TSS_ERROR_ALREADY_STREAMING = 4,					/**< The device is already streaming */
		TSS_ERROR_NOT_STREAMING = 5,						/**< The device is not streaming */
		TSS_ERROR_LOGICAL_ID_OUT_OF_RANGE = 6,				/**< The logical id parameter passed in to an API call is not allowed */
		TSS_ERROR_CHILD_EXISTS = 7,							/**< The child of the device already exist */
		TSS_ERROR_COMMAND_FAILURE = 8,						/**< The command returned a failed response */
		TSS_ERROR_CANT_STOP_STREAMING = 9,					/**< The API failed to stop streaming */
		TSS_ERROR_NO_WIRELESS_STREAMING_FROM_SENSOR = 10,	/**< The device is currently wireless and cannot stream over wired connection */
		TSS_ERROR_COMMAND_CALLED_DURING_STREAMING = 11,		/**< The device is currently streaming */
		TSS_ERROR_NOT_CONNECTED = 12,						/**< The device is not connected */
		TSS_ERROR_CANT_CLOSE_WIRELESS_PORT = 13,			/**< The API cannot close the communication port */
		TSS_ERROR_ALREADY_CONNECTED = 14,					/**< The device is already connected */
		TSS_ERROR_INVALID_ID = 15,							/**< The tss_device_id parameter passed in to an API call is not associated with a connected 3-Space device */
		TSS_ERROR_CANT_READ_INIT_DATA = 16,					/**< The device could not read data during the initialization process */
		TSS_ERROR_WIRELESS_ONLY = 17,						/**< The command is only works over wireless */
		TSS_ERROR_SENSOR_TYPE_MISMATCH = 18,				/**< The command does not work for the device */
		TSS_ERROR_CHILD_DOESNT_EXIST = 19,					/**< The child of the device does not exist */
		TSS_ERROR_BOOTLOADER_MODE = 20,                     /**< The command cannot be sent to a device in bootloader mode */
		TSS_ERROR_BOOTLOADER_ONLY = 21                      /**< The command only works with devices in bootloader mode */
	};

	/********************************************//**
	* \ingroup tss_api
	* A c_string array to help express the different types of errors a 3-Space API call can return.
	***********************************************/
	static const char* tss_error_string[] = {
		"TSS_NO_ERROR",
		"TSS_ERROR_NOT_ENOUGH_DATA",
		"TSS_ERROR_CANT_OPEN_PORT",
		"TSS_ERROR_NOT_AVAILABLE_WIRELESS",
		"TSS_ERROR_ALREADY_STREAMING",
		"TSS_ERROR_NOT_STREAMING",
		"TSS_ERROR_LOGICAL_ID_OUT_OF_RANGE",
		"TSS_ERROR_CHILD_EXISTS",
		"TSS_ERROR_COMMAND_FAILURE",
		"TSS_ERROR_CANT_STOP_STREAMING",
		"TSS_ERROR_NO_WIRELESS_STREAMING_FROM_SENSOR",
		"TSS_ERROR_COMMAND_CALLED_DURING_STREAMING",
		"TSS_ERROR_NOT_CONNECTED",
		"TSS_ERROR_CANT_CLOSE_WIRELESS_PORT",
		"TSS_ERROR_ALREADY_CONNECTED",
		"TSS_ERROR_INVALID_ID",
		"TSS_ERROR_CANT_READ_INIT_DATA",
		"TSS_ERROR_WIRELESS_ONLY",
		"TSS_ERROR_SENSOR_TYPE_MISMATCH",
		"TSS_ERROR_CHILD_DOESNT_EXIST",
		"TSS_ERROR_BOOTLOADER_MODE",
		"TSS_ERROR_BOOTLOADER_ONLY"
	};

#define TSS_STREAMING_PACKET_SIZE 128
#define TSS_PORT_NAME_SIZE 128
#define TSS_STREAM_DURATION_INFINITE 0xffffffff

	/********************************************//**
	* \ingroup tss_api
	* An enum expressing the command list of Streamable Commands.
	***********************************************/
	enum TSS_STREAM
	{
		TSS_STREAM_TARED_ORIENTATION_AS_QUATERNION = 1,             /**< Stream command to get the tared orientation as a quaternion */
		TSS_STREAM_TARED_ORIENTATION_AS_EULER_ANGLES = 2,           /**< Stream command to get the tared orientation as a euler angle */
		TSS_STREAM_TARED_ORIENTATION_AS_ROTATION_MATRIX = 4,        /**< Stream command to get the tared orientation as a rotation matrix */
		TSS_STREAM_TARED_ORIENTATION_AS_AXIS_ANGLE = 8,             /**< Stream command to get the tared orientation as a axis angle */
		TSS_STREAM_TARED_ORIENTATION_AS_TWO_VECTOR = 16,            /**< Stream command to get the tared orientation as two vectors */
		TSS_STREAM_UNTARED_ORIENTATION_AS_QUATERNION = 32,          /**< Stream command to get the untared orientation as a quaternion */
		TSS_STREAM_UNTARED_ORIENTATION_AS_EULER_ANGLES = 64,        /**< Stream command to get the untared orientation as a euler angle */
		TSS_STREAM_UNTARED_ORIENTATION_AS_ROTATION_MATRIX = 128,    /**< Stream command to get the untared orientation as a rotation matrix */
		TSS_STREAM_UNTARED_ORIENTATION_AS_AXIS_ANGLE = 256,         /**< Stream command to get the untared orientation as a axis angle */
		TSS_STREAM_UNTARED_ORIENTATION_AS_TWO_VECTOR = 512,         /**< Stream command to get the untared orientation as two vectors */
		TSS_STREAM_RAW_SENSOR_DATA = 1024,                          /**< Stream command to get the raw gyroscope/accelerometer/magnetometer data */
		TSS_STREAM_RAW_GYROSCOPE_DATA = 2048,                       /**< Stream command to get the raw gyroscope data */
		TSS_STREAM_RAW_ACCELEROMETER_DATA = 4096,                   /**< Stream command to get the raw accelerometer data */
		TSS_STREAM_RAW_MAGNETOMETER_DATA = 8192,                    /**< Stream command to get the raw magnetometer data */
		TSS_STREAM_NORMALIZED_SENSOR_DATA = 16384,                  /**< Stream command to get the normalized gyroscope/accelerometer/magnetometer data */
		TSS_STREAM_NORMALIZED_GYROSCOPE_DATA = 32768,               /**< Stream command to get the normalized gyroscope data */
		TSS_STREAM_NORMALIZED_ACCELEROMETER_DATA = 65536,           /**< Stream command to get the normalized accelerometer data */
		TSS_STREAM_NORMALIZED_MAGNETOMETER_DATA = 131072,           /**< Stream command to get the normalized magnetometer data */
		TSS_STREAM_CORRECTED_SENSOR_DATA = 262144,                  /**< Stream command to get the corrected gyroscope/accelerometer/magnetometer data */
		TSS_STREAM_CORRECTED_GYROSCOPE_DATA = 524288,               /**< Stream command to get the corrected gyroscope data */
		TSS_STREAM_CORRECTED_ACCELEROMETER_DATA = 1048576,          /**< Stream command to get the corrected accelerometer data */
		TSS_STREAM_CORRECTED_MAGNETOMETER_DATA = 4194304,           /**< Stream command to get the corrected magnetometer data */
		TSS_STREAM_LINEAR_ACCELERATION = 8388608,                   /**< Stream command to get the linear acceleration */
		TSS_STREAM_BATTERY_VOLTAGE = 16777216,                      /**< Stream command to get the battery voltage */
		TSS_STREAM_BATTERY_LEVEL = 33554432,                        /**< Stream command to get the battery level */
		TSS_STREAM_BATTERY_STATUS = 67108864,                       /**< Stream command to get the battery status */
		TSS_STREAM_CELSIUS_TEMPERATURE = 134217728,                 /**< Stream command to get the temperature in degrees celsius */
		TSS_STREAM_FAHRENHEIT_TEMPERATURE = 268435456,              /**< Stream command to get the temperature in degrees fahrenheit */
		TSS_STREAM_BUTTON_STATE = 536870912                         /**< Stream command to get the button state */
    };

	enum TSS_RESPONSE_HEADER
	{
		TSS_RESPONSE_HEADER_SUCCESS = 1,
		TSS_RESPONSE_HEADER_TIMESTAMP = 2,
		TSS_RESPONSE_HEADER_COMMAND_ECHO = 4,
		TSS_RESPONSE_HEADER_CHECKSUM = 8,
		TSS_RESPONSE_HEADER_LOGICAL_ID = 16,
		TSS_RESPONSE_HEADER_SERIAL_NUMBER = 32,
		TSS_RESPONSE_HEADER_DATA_LENGTH = 64
	};

	/********************************************//**
	* \ingroup tss_api
	* An enum expressing the find flags for the tss_getComPorts filter parameter.
	***********************************************/
	enum TSS_TYPE
	{
		TSS_UNKNOWN = 1,			/**< Device type was not able to be determined */
        TSS_BOOTLOADER = 2,			/**< 3-Space Bootloader */
		TSS_USB = 4,				/**< 3-Space USB */
		TSS_EMBEDDED = 8,			/**< 3-Space Embedded */
		TSS_WIRELESS = 16,			/**< 3-Space Wireless */
		TSS_DONGLE = 32,			/**< 3-Space Dongle */
		TSS_DATALOGGER = 64,		/**< 3-Space Data Logger */
		TSS_BLUETOOTH = 128,		/**< 3-Space Bluetooth */
		TSS_LE = 256,				/**< 3-Space LE */
		TSS_LX = 512,				/**< 3-Space LX */
        TSS_FIND_ALL_KNOWN = 1023	/**< Find all serial ports including "unknown" serial ports that may have 3-Space devices connected */
	};

	/********************************************//**
	* \ingroup tss_api
	* An enum expressing the types of possible connections a computer may have to a device
	***********************************************/
	enum TSS_CONNECTION_TYPE
	{
		TSS_USB_CT = 0,             /**< 3-Space Device connected over USB connection */
		TSS_BT_CT = 1               /**< 3-Space Device connected over Bluetooth connection */
	};

	/********************************************//**
	* \ingroup tss_api
	* \brief A structure that contains basic information about a serial port.
	***********************************************/
	typedef struct TSS_ComPort
	{
		char* port_name;      /**< The serial port string. */
		U16 device_type;       /**< The type of PrioVR device connected through the serial port. */
		U8 connection_Type;
	} TSS_ComPort;

	/********************************************//**
	* \ingroup tss_api
	* \brief A structure that contains the entire packet header data. NOTE: Some options may not be initialized.
	***********************************************/
	typedef struct TSS_Header
	{
		U8 Flags;
		U8 Success;
		U32 SensorTimestamp;
		float SystemTimestamp;
		U8 CommandEcho;
		U8 Checksum;
		U8 LogicalId;
		U32 SerialNumber;
		U8 DataLength;
	} TSS_Header;

	/********************************************//**
	* \ingroup tss_api
	* \brief A structure that contains the entire packet's data. NOTE: Some fields may not be initialized.
	***********************************************/
	typedef struct TSS_Stream_Packet
	{
		struct TSS_Header header;
		U8 streaming_slots[8];

		float taredOrientQuat[4];
		float taredOrientEuler[3];
		float taredOrientMatrix[9];
		float taredOrientAxis[3];
		float taredOrientAngle;
		float taredOrientForward[3];
		float taredOrientDown[3];
		float untaredOrientQuat[4];
		float untaredOrientEuler[3];
		float untaredOrientMatrix[9];
		float untaredOrientAxis[3];
		float untaredOrientAngle;
		float untaredOrientNorth[3];
		float untaredOrientGravity[3];

		float rawSensorData[9];
		float rawGyroscopeData[3];
		float rawAccelerometerData[3];
		float rawMagnetometerData[3];
		float normalizedSensorData[9];
		float normalizedGyroscopeData[3];
		float normalizedAccelerometerData[3];
		float normalizedMagnetometerData[3];
		float correctedSensorData[9];
		float correctedGyroscopeData[3];
		float correctedAccelerometerData[3];
		float correctedMagnetometerData[3];
		float linearAcceleration[3];
		float batteryVoltage;
		U8 batteryLevel;
		U8 batteryStatus;
		float temperatureF;
		float temperatureC;
		U8 buttonState;

		U8 rawData[TSS_STREAMING_PACKET_SIZE];
		U32 rawDataSize;
	} TSS_Stream_Packet;

#define TSS_WIRED (TSS_USB | TSS_EMBEDDED | TSS_WIRELESS | TSS_DATALOGGER | TSS_BLUETOOTH | TSS_LE | TSS_LX)

#if defined(_MSC_VER)
#define TSS_EXPORT __declspec(dllexport)
#elif defined(__GNUC__) && !defined(__APPLE__)
#define TSS_EXPORT __attribute__ ((dllexport))
#elif defined(__GNUC__) && defined(__APPLE__)
#define TSS_EXPORT
#endif

    /********************************************//**
    * \ingroup tss_api_methods
    * \brief Initalize the 3-Space API.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
	TSS_EXPORT TSS_ERROR tss_initAPI();
    /********************************************//**
    * \ingroup tss_api_methods
    * \brief Deconstruct the 3-Space API.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
	TSS_EXPORT TSS_ERROR tss_deinitAPI();

	////Sensor functions
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Creates a 3-Space Sensor ID.
    *
    * This creates and performs initial setup on the connected 3-Space Sensor.
    * The returned 3-Space Sensor ID is used to interact with the 3-Space Sensor.
    * When a 3-Space Sensor ID is created, other programs cannot use that serial port until the port is closed.
    * \param[in] port_name The serial port string for the serial port to be queried.
    * \param[out] out_id A tss_device_id will be returned to represent the 3-Space Sensor.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
	TSS_EXPORT TSS_ERROR tss_createSensor(const char* port_name, tss_device_id* out_id);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief This removes the connected 3-Space Sensor.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_removeSensor(tss_device_id sensor_id);

	//Standard device functions
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Checks to see if the sensor is connected in bootloader mode.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] in_bootloader The flag which states the mode, 1 for bootloader mode, 0 for firmware mode.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_inBootloader(tss_device_id sensor_id, U8* in_bootloader);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Retrieves the serial number of the 3-Space Sensor.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] serial_number The serial number.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getSerialNumber(tss_device_id sensor_id, U32* serial_number, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Retrieves the hardware version string of the 3-Space Sensor.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] version_string The version string.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getHardwareVersionString(tss_device_id sensor_id, char* hw_version_string);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Retrieves the firmware version string of the 3-Space Sensor.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] version_string The version string.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getFirmwareVersionString(tss_device_id sensor_id, char* fw_version_string);

	/********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Checks to see if the specified 3-Space Sensor has a wireless connection.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] is_wireless Will have the value 1 if wireless, 0 otherwise.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_isWireless(tss_device_id sensor_id, U8* is_wireless);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Checks to see if the specified 3-Space Sensor is connected.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_isConnected(tss_device_id sensor_id);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Retrieves the last timestamp logged by the 3-Space Sensor.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[out] timestamp the returned timestamp in milliseconds.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getLastSensorTimestamp(tss_device_id sensor_id, U32* timestamp);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Retrieves the last timestamp logged by the System.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[out] timestamp the returned timestamp in milliseconds.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_sensor_getLastSystemTimestamp(tss_device_id sensor_id, float* timestamp);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Retrieves the last timestamp logged by the 3-Space Sensor.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[out] sensor_type The type of the 3-Space Sensor.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_sensor_getSensorType(tss_device_id sensor_id, U16* sensor_type);
	
	//Port control functions
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Open a port to connect to 3-Space Sensor.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[in] port_name The name of the communication port
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_openPort(tss_device_id sensor_id, const char* port_name);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Close the connected port
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_sensor_closePort(tss_device_id sensor_id);

	//Header control functions
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Enables timestamps for wired connection
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_sensor_enableTimestampsWired(tss_device_id sensor_id);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Disables timestamps for wired connection
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_sensor_disableTimestampsWired(tss_device_id sensor_id);

	//Streaming functions
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Enable wireless streaming with the given flags.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[in] data_flags The data flags reprsenting the data to stream. The flags are TSS_STREAM_TARED_ORIENTATION_AS_QUATERNION, TSS_STREAM_TARED_ORIENTATION_AS_EULER_ANGLES,
    * TSS_STREAM_TARED_ORIENTATION_AS_ROTATION_MATRIX, TSS_STREAM_TARED_ORIENTATION_AS_AXIS_ANGLE, TSS_STREAM_TARED_ORIENTATION_AS_TWO_VECTOR, TSS_STREAM_UNTARED_ORIENTATION_AS_QUATERNION,
	* TSS_STREAM_UNTARED_ORIENTATION_AS_EULER_ANGLES, TSS_STREAM_UNTARED_ORIENTATION_AS_ROTATION_MATRIX, TSS_STREAM_UNTARED_ORIENTATION_AS_AXIS_ANGLE, TSS_STREAM_UNTARED_ORIENTATION_AS_TWO_VECTOR, and more.
    * \param[in] interval The interval to stream data.
    * \param[in] duration The duration of the stream.
    * \param[in] delay A delay before streaming.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_sensor_enableStreamingWireless(tss_device_id sensor_id, U32 data_flags, U32 interval, U32 duration, U32 delay);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Disables wireless streaming with the given flags.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_sensor_disableStreamingWireless(tss_device_id sensor_id);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Enables wired streaming with the given flags.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[in] data_flags The data flags reprsenting the data to stream. The flags are TSS_STREAM_TARED_ORIENTATION_AS_QUATERNION, TSS_STREAM_TARED_ORIENTATION_AS_EULER_ANGLES,
    * TSS_STREAM_TARED_ORIENTATION_AS_ROTATION_MATRIX, TSS_STREAM_TARED_ORIENTATION_AS_AXIS_ANGLE, TSS_STREAM_TARED_ORIENTATION_AS_TWO_VECTOR, TSS_STREAM_UNTARED_ORIENTATION_AS_QUATERNION,
    * TSS_STREAM_UNTARED_ORIENTATION_AS_EULER_ANGLES, TSS_STREAM_UNTARED_ORIENTATION_AS_ROTATION_MATRIX, TSS_STREAM_UNTARED_ORIENTATION_AS_AXIS_ANGLE, TSS_STREAM_UNTARED_ORIENTATION_AS_TWO_VECTOR, and more.
    * \param[in] interval The interval to stream data.
    * \param[in] duration The duration of the stream.
    * \param[in] delay A delay before streaming.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_sensor_startStreamingWired(tss_device_id sensor_id, U32 data_flags, U32 interval, U32 duration, U32 delay=0);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Return the oldest received packet.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[out] stream_packet The first packet in the buffer.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/    
    TSS_EXPORT TSS_ERROR tss_sensor_getFirstStreamingPacket(tss_device_id sensor_id, struct TSS_Stream_Packet* stream_packet);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Return the newest received packet.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[out] stream_packet The last packet in the buffer.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_sensor_getLastStreamingPacket(tss_device_id sensor_id, struct TSS_Stream_Packet* stream_packet);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Disables wired streaming with the given flags.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_sensor_stopStreamingWired(tss_device_id sensor_id);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Get the number of unread stream packets.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[out] in_waiting The number of unread packets.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_sensor_getStreamingPacketsInWaiting(tss_device_id sensor_id, U32* in_waiting);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Whether or not the 3-Space Sensor as overloaded and wrapped the buffer.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[out] overflow The U8 representing whether of not the buffer has overloaded.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_sensor_didStreamingOverflow(tss_device_id sensor_id, U8* overflow);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Gets the data flags reprsenting the data to stream. The possible flags are TSS_STREAM_TARED_ORIENTATION_AS_QUATERNION, TSS_STREAM_TARED_ORIENTATION_AS_EULER_ANGLES,
    * TSS_STREAM_TARED_ORIENTATION_AS_ROTATION_MATRIX, TSS_STREAM_TARED_ORIENTATION_AS_AXIS_ANGLE, TSS_STREAM_TARED_ORIENTATION_AS_TWO_VECTOR, TSS_STREAM_UNTARED_ORIENTATION_AS_QUATERNION,
    * TSS_STREAM_UNTARED_ORIENTATION_AS_EULER_ANGLES, TSS_STREAM_UNTARED_ORIENTATION_AS_ROTATION_MATRIX, TSS_STREAM_UNTARED_ORIENTATION_AS_AXIS_ANGLE, TSS_STREAM_UNTARED_ORIENTATION_AS_TWO_VECTOR, and more.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] slot1 A byte representing what data is streamed from the sensor for this slot.
	* \param[out] slot2 A byte representing what data is streamed from the sensor for this slot.
	* \param[out] slot3 A byte representing what data is streamed from the sensor for this slot.
	* \param[out] slot4 A byte representing what data is streamed from the sensor for this slot.
	* \param[out] slot5 A byte representing what data is streamed from the sensor for this slot.
	* \param[out] slot6 A byte representing what data is streamed from the sensor for this slot.
	* \param[out] slot7 A byte representing what data is streamed from the sensor for this slot.
	* \param[out] slot8 A byte representing what data is streamed from the sensor for this slot.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getStreamingSlots(tss_device_id sensor_id, U8* slot1, U8* slot2, U8* slot3, U8* slot4, U8* slot5, U8* slot6, U8* slot7, U8* slot8, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets the data flags reprsenting the data to stream. The possible flags are TSS_STREAM_TARED_ORIENTATION_AS_QUATERNION, TSS_STREAM_TARED_ORIENTATION_AS_EULER_ANGLES,
    * TSS_STREAM_TARED_ORIENTATION_AS_ROTATION_MATRIX, TSS_STREAM_TARED_ORIENTATION_AS_AXIS_ANGLE, TSS_STREAM_TARED_ORIENTATION_AS_TWO_VECTOR, TSS_STREAM_UNTARED_ORIENTATION_AS_QUATERNION,
    * TSS_STREAM_UNTARED_ORIENTATION_AS_EULER_ANGLES, TSS_STREAM_UNTARED_ORIENTATION_AS_ROTATION_MATRIX, TSS_STREAM_UNTARED_ORIENTATION_AS_AXIS_ANGLE, TSS_STREAM_UNTARED_ORIENTATION_AS_TWO_VECTOR, and more.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] slot1 A byte representing what data is streamed from the sensor for this slot.
	* \param[in] slot2 A byte representing what data is streamed from the sensor for this slot.
	* \param[in] slot3 A byte representing what data is streamed from the sensor for this slot.
	* \param[in] slot4 A byte representing what data is streamed from the sensor for this slot.
	* \param[in] slot5 A byte representing what data is streamed from the sensor for this slot.
	* \param[in] slot6 A byte representing what data is streamed from the sensor for this slot.
	* \param[in] slot7 A byte representing what data is streamed from the sensor for this slot.
	* \param[in] slot8 A byte representing what data is streamed from the sensor for this slot.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setStreamingSlots(tss_device_id sensor_id, U8 slot1, U8 slot2, U8 slot3, U8 slot4, U8 slot5, U8 slot6, U8 slot7, U8 slot8, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Gets the streaming timing information from the 3-Space Sensor.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] interval The interval to stream data.
    * \param[out] duration The duration of the stream.
    * \param[out] delay A delay before streaming.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getStreamingTiming(tss_device_id sensor_id, U32* interval, U32* duration, U32* delay, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets the streaming timing information from the 3-Space Sensor.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] interval The interval to stream data.
	* \param[in] duration The duration of the stream.
	* \param[in] delay A delay before streaming.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setStreamingTiming(tss_device_id sensor_id, U32 interval, U32 duration, U32 delay, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Gets the wired response header bitfield from the sensor.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] bitfield The bitfield returned, see commmand charts for additional info.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getWiredResponseHeaderBitfield(tss_device_id sensor_id, U32* bitfield, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets the wired response header bitfield from the sensor.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] bitfield The bitfield, see commmand charts for additional info.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setWiredResponseHeaderBitfield(tss_device_id sensor_id, U32 bitfield, U32* timestamp);

	//Call and response functions
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Sets the total number of times to retry a command before failing.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[in] retries The number of times to retry a command before failing.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setCommandRetries(tss_device_id sensor_id, U8 retries);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Get the tared orientation of a sensor as a Quaternion, it will be returned in a float array in the order (x,y,z,w).
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[out] orient_quat The oreintation of the given sensor as a quaterion.
	* \param[out] timestamp The returned timestamp in milliseconds.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_sensor_getTaredOrientationAsQuaternion(tss_device_id sensor_id, float* orient_quat, U32* timestamp);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Get the tared orientation of a sensor as a euler angle, it will be returned in a float array in the order (x,y,z).
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[out] euler The oreintation of the given sensor as a euler angle.
	* \param[out] timestamp The returned timestamp in milliseconds.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_sensor_getTaredOrientationAsEulerAngles(tss_device_id sensor_id, float* euler, U32* timestamp);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Get the tared orientation of a sensor as a Rotation Matrix, it will be returned in a float array in the order ([0][0],[0][1],[0][2],[1][0],[1][1],[1][2],[2][0],[2][1],[2][2]).
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[out] mat The oreintation of the given sensor as a rotation matrix.
	* \param[out] timestamp The returned timestamp in milliseconds.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_sensor_getTaredOrientationAsRotationMatrix(tss_device_id sensor_id, float* mat, U32* timestamp);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Get the tared orientation of a sensor as an Axis Angle , it will be returned in a float array in the order (x,y,z) and the angle.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[out] vec The axis of the angle.
    * \param[out] angle The angle of the oreintation.
	* \param[out] timestamp The returned timestamp in milliseconds.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_sensor_getTaredOrientationAsAxisAngle(tss_device_id sensor_id, float* vec, float* angle, U32* timestamp);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Get the tared orientation of a sensor as a forward vector and a down vector, they will be returned as two float arrays in the order (x,y,z).
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[out] forward The forward vector.
    * \param[out] down The down vector.
	* \param[out] timestamp The returned timestamp in milliseconds.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_sensor_getTaredOrientationAsTwoVector(tss_device_id sensor_id, float* forward, float* down, U32* timestamp);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Get the untared orientation of a sensor as a Quaternion, it will be returned in a float array in the order (x,y,z,w).
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[out] orient_quat The oreintation of the given sensor as a quaterion.
	* \param[out] timestamp The returned timestamp in milliseconds.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_sensor_getUntaredOrientationAsQuaternion(tss_device_id sensor_id, float* orient_quat, U32* timestamp);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Get the untared orientation of a sensor, it will be returned in a float array in the order (x,y,z).
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[out] euler The oreintation of the given sensor as a euler angle.
	* \param[out] timestamp The returned timestamp in milliseconds.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_sensor_getUntaredOrientationAsEulerAngles(tss_device_id sensor_id, float* euler, U32* timestamp);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Get the untared orientation of a sensor as a Rotation Matrix, it will be returned in a float array in the order ([0][0],[0][1],[0][2],[1][0],[1][1],[1][2],[2][0],[2][1],[2][2]).
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[out] mat The oreintation of the given sensor as a rotation matrix.
	* \param[out] timestamp The returned timestamp in milliseconds.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_sensor_getUntaredOrientationAsRotationMatrix(tss_device_id sensor_id, float* mat, U32* timestamp);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Get the untared orientation of a sensor as an Axis Angle , it will be returned in a float array in the order (x,y,z) and the angle.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[out] vec The axis of the angle.
    * \param[out] angle The angle of the oreintation.
	* \param[out] timestamp The returned timestamp in milliseconds.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_sensor_getUntaredOrientationAsAxisAngle(tss_device_id sensor_id, float* vec, float* angle, U32* timestamp);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Get the untared orientation of a sensor as a north vector and a gravity vector, it will be returned in as two float arrays in the order (x,y,z).
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[out] north The forward vector.
    * \param[out] gravity The down vector.
	* \param[out] timestamp The returned timestamp in milliseconds.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_sensor_getUntaredOrientationAsTwoVector(tss_device_id sensor_id, float* north, float* gravity, U32* timestamp);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Enumerate the 3-Space Sensor
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] timestamp The returned timestamp in milliseconds.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_sensor_tareWithCurrentOrientation(tss_device_id sensor_id, U32* timestamp);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Return the current wireless channel of the 3-Space Sensor. In order for a Dongle to communicate with a 3-Space Sensor, they must both share the same channel.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[out] channel The wireless channel ID.
	* \param[out] timestamp The returned timestamp in milliseconds.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_sensor_getWirelessChannel(tss_device_id sensor_id, U8* channel, U32* timestamp);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Set the wireless channel of the 3-Space Sensor. In order for a Dongle to communicate with a 3-Space Sensor, they must both share the same channel. Note that channel must be in the range 11 � 26 inclusive.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[in] channel The wireless channel ID.
	* \param[out] timestamp The returned timestamp in milliseconds.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_sensor_setWirelessChannel(tss_device_id sensor_id, U8 channel, U32* timestamp);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Return the current wireless panID of the 3-Space Sensor. In order for a Dongle to communicate with a 3-Space Sensor, they must both share the same panID.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[out] pan_id The wireless pan ID.
	* \param[out] timestamp The returned timestamp in milliseconds.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_sensor_getWirelessPanID(tss_device_id sensor_id, U16* pan_id, U32* timestamp);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Set the wireless panID of the 3-Space Sensor. In order for a Dongle to communicate with a 3-Space Sensor, they must both share the same panID. Note that panIDs must be in the range 1 � 65534 inclusive.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[in] pan_id The wireless pan ID.
	* \param[out] timestamp The returned timestamp in milliseconds.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_sensor_setWirelessPanID(tss_device_id sensor_id, U16 pan_id, U32* timestamp);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Commits all committable settings to non-volatile memory so that they will persist beyond a powercycle. For more information on Commitable Settings, please refer to the PrioVR User's Manual.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] timestamp The returned timestamp in milliseconds.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_sensor_commitSettings(tss_device_id sensor_id, U32* timestamp);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Commits all committable wireless settings to non-volatile memory so that they will persist beyond a powercycle. For more information on Commitable Settings, please refer to the PrioVR User's Manual.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] timestamp The returned timestamp in milliseconds.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_sensor_commitWirelessSettings(tss_device_id sensor_id, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Returns the normalized gyro rate vector, accelerometer vector, and compass vector.Note that the gyro vector is in units of radians / sec, while the accelerometer and compass are unit - length vectors indicating the direction of gravity and north, respectively.These two vectors do not have any magnitude data associated with them.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] gyroscope3 The gyroscope data.
	* \param[out] accelerometer3 The accelerometer data.
	* \param[out] magnetometer3 The magnetometer data.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getNormalizedSensorData(tss_device_id sensor_id, float* gyroscope3, float* accelerometer3, float* magnetometer3, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Returns the normalized gyro rate vector, which is in units of radians/sec.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] gyroscope3 The gyroscope data.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getNormalizedGyroscope(tss_device_id sensor_id, float* gyroscope3, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Returns the normalized accelerometer vector. Note that this is a unit-vector indicating the direction of gravity. This vector does not have any magnitude data associated with it.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] accelerometer3 The accelerometer data.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getNormalizedAccelerometer(tss_device_id sensor_id, float* accelerometer3, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Returns the normalized compass vector. Note that this is a unit-vector indicating the direction of gravity. This vector does not have any magnitude data associated with it.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Dongle.
	* \param[out] magnetometer3 The magnetometer data.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getNormalizedMagnetometer(tss_device_id sensor_id, float* magnetometer3, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Returns the corrected gyro rate vector, accelerometer vector, and compass vector. Note that the gyro vector is in units of radians/sec, the accelerometer vector is in units of G, and the compass vector is in units of gauss.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] gyroscope3 The gyroscope data.
	* \param[out] accelerometer3 The accelerometer data.
	* \param[out] magnetometer3 The magnetometer data.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getCorrectedSensorData(tss_device_id sensor_id, float* gyroscope3, float* accelerometer3, float* magnetometer3, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Returns the corrected gyro rate vector, which is in units of radians/sec.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] gyroscope3 The gyroscope data.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getCorrectedGyroscope(tss_device_id sensor_id, float* gyroscope3, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Returns the corrected accelerometer vector. Note that this is a unit-vector indicating the direction of gravity. This vector does not have any magnitude data associated with it.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] accelerometer3 The accelerometer data.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getCorrectedAccelerometer(tss_device_id sensor_id, float* accelerometer3, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Returns the corrected compass vector. Note that this is a unit-vector indicating the direction of gravity. This vector does not have any magnitude data associated with it.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] magnetometer3 The magnetometer data.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getCorrectedMagnetometer(tss_device_id sensor_id, float* magnetometer3, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Returns the linear acceleration of the device, which is the overall acceleration which has been orientation compensated and had the component of acceleration due to gravity removed. Uses the untared orientation.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] accelerometer3 The accelerometer data.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getCorrectedLinearAccelerationInGlobalSpace(tss_device_id sensor_id, float* accelerometer3, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Returns the raw gyro rate vector, accelerometer vector, and compass vector. Note that the gyro vector is in units of radians/sec, the accelerometer vector is in units of G, and the compass vector is in units of gauss.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] gyroscope3 The gyroscope data.
	* \param[out] accelerometer3 The accelerometer data.
	* \param[out] magnetometer3 The magnetometer data.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getRawComponentSensorData(tss_device_id sensor_id, float* gyroscope3, float* accelerometer3, float* magnetometer3, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Returns the raw gyro rate vector, which is in units of radians/sec.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] gyroscope3 The gyroscope data.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getRawGyroscope(tss_device_id sensor_id, float* gyroscope3, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Returns the raw accelerometer vector. Note that this is a unit-vector indicating the direction of gravity. This vector does not have any magnitude data associated with it.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] accelerometer3 The accelerometer data.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getRawAccelerometer(tss_device_id sensor_id, float* accelerometer3, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Returns the raw compass vector. Note that this is a unit-vector indicating the direction of gravity. This vector does not have any magnitude data associated with it.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] magnetometer3 The magnetometer data.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getRawMagnetometer(tss_device_id sensor_id, float* magnetometer3, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Returns the temperature of the sensor in Celsius.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] temp The tempature.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getTemperatureC(tss_device_id sensor_id, float* temp, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Returns the temperature of the sensor in Fahrenheit.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] temp The tempature.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getTemperatureF(tss_device_id sensor_id, float* temp, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Returns a value indicating how much the sensor is being moved at the moment. This value will return 1 if the sensor is completely stationary, and will return 0 if it is in motion. This command can also return values in between indicating how much motion the sensor is experiencing.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] confindence The confindence factor.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getConfidenceFactor(tss_device_id sensor_id, float* confindence, U32* timestamp);

	//Data-Logging Commands
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Enables SD card access, but prevents data logs from being taken.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_turnOnMassStorage(tss_device_id sensor_id, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Erases the contents of the SD card.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_turnOffMassStorage(tss_device_id sensor_id, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Initiates a data logging section with the specified attributes as indicated in the provided data logging configuration file.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_formatAndInitializeSDCard(tss_device_id sensor_id, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Initiates a data logging section with the specified attributes as indicated in the provided data logging configuration file.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_beginDataLoggingSession(tss_device_id sensor_id, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Temin_ratioates the ongoing data logging session.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_endDataLoggingSession(tss_device_id sensor_id, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets the current time on the onboard real-time clock.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] month The month.
	* \param[in] day The date.
	* \param[in] year The year.
	* \param[in] hour The hour.
	* \param[in] minute The minute.
	* \param[in] second The second.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setClockValues(tss_device_id sensor_id, U8 month, U8 day, U8 year, U8 hour, U8 minute, U8 second, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Returns the current time on the onboard real-time clock.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] month The month.
	* \param[out] day The date.
	* \param[out] year The year.
	* \param[out] hour The hour.
	* \param[out] minute The minute.
	* \param[out] second The second.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getClockValues(tss_device_id sensor_id, U8* month, U8* day, U8* year, U8* hour, U8* minute, U8* second, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Update current timestamp.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] set_timestamp The timestamp.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_updateCurrentTimestamp(tss_device_id sensor_id, U32 set_timestamp, U32* timestamp);

	//Configuration Write Commands
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets the current euler angle decomposition order, which detemin_ratioes how the angles returned from command 0x1 are decomposed from the full quaternion orientation. Possible values are 0x0 for XYZ, 0x1 for YZX, 0x2 for ZXY, 0x3 for ZYX, 0x4 for XZY or 0x5 for YXZ (default).
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] order Euler angle decomposition order.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setEulerAngleDecompositionOrder(tss_device_id sensor_id, U8 order, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets required parameters that are necessary to trigger magnetometer resistance mode. First parameter to the command specifies the change in magnetometer field strength that is required to trigger the resistance. Once this field has been detected, the magnetometer will enter a period where it is completely locked out of the orientation calculationthis period will increase while magnetic perturbations are still being detected, but will dissipate as the sensor remains stationary. Once this period is over, the sensor orientation will slowly begin trusting the magnetometer again. The second parameter represents the number of frames that must elapse before the magnetometer is fully trusted again. The third parameter represents a decay value between 0 and 1 that indicates how quickly the outright magnetometer rejection state will fall off. Values closer to 1 result in the magnetometer rejection lasting longer. The final parameter represents how quickly a magnetic perturbation is detected. Values closer to 1 result in the magnetometer rejection occurring more slowly. This setting can be saved to non-volatile flash memory using the Commit Settings command.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] threshold Magnetoresistive threshold in gauss.
	* \param[in] trust_frames Number of magnetometer trust frames.
	* \param[in] lockout_decay Magnetometer lockout decay value.
	* \param[in] perturbation_detection_value Magnetometer perturbation detection value.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setMagnetoresistiveThreshold(tss_device_id sensor_id, float threshold, U32 trust_frames, float lockout_decay, float perturbation_detection_value, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets required parameters that are necessary to trigger accelerometer rejection. During the accelerometer rejection period, the contribution of the accelerometer to the selected orientation estimation algorithm will be zero. The arguments to this command specify the accelerometer threshold and the number of frames that the rejection is active, respectively. This setting can be saved to non-volatile flash memory using the Commit Settings command.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] threshold Accelerometer threshold in g's.
	* \param[in] lockout_frames Number of accelerometer lockout frames.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setAccelerometerResistanceThreshold(tss_device_id sensor_id, float threshold, U32 lockout_frames, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets the offset orientation to be the same as the current filtered orientation.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_offsetWithCurrentOrientation(tss_device_id sensor_id, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets the base offset to an identity quaternion.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_resetBaseOffset(tss_device_id sensor_id, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets the offset orientation to be the same as the supplied orientation, which should be passed as a quaternion.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] quat4 The Quaterion.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_offsetWithQuaternion(tss_device_id sensor_id, const float* quat4, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets the tare orientation to be the same as the supplied orientation, which should be passed as a quaternion.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] quat4 The Quaterion.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_tareWithQuaternion(tss_device_id sensor_id, const float* quat4, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets the tare orientation to be the same as the supplied orientation, which should be passed as a rotation matrix.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] matrix9 The rotation matrix.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_tareWithRotationMatrix(tss_device_id sensor_id, const float* matrix9, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Detemin_ratioes how trusted the accelerometer contribution is to the overall orientation estimation. Trust is 0 to 1, with 1 being fully trusted and 0 being not trusted at all.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] trust_value Accelerometer trust value.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setStaticAccelerometerTrustValue(tss_device_id sensor_id, float trust_value, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Detemin_ratioes how trusted the accelerometer contribution is to the overall orientation estimation. Instead of using a single value, uses a minimum and maximum value. Trust values will be selected from this range depending on the confidence factor. This can have the effect of smoothing out the accelerometer when the sensor is in motion. 
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] min_trust_value Minimum accelerometer trust value.
	* \param[in] max_trust_value Maximum accelerometer trust value.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setConfidenceAccelerometerTrustValues(tss_device_id sensor_id, float min_trust_value, float max_trust_value, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Detemin_ratioes how trusted the accelerometer contribution is to the overall orientation estimation. tribution is to the overall orientation estimation. Trust is 0 to 1, with 1 being fully trusted and 0 being not trusted at all.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] trust_value Compass trust value.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setStaticCompassTrustValue(tss_device_id sensor_id, float trust_value, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Detemin_ratioes how trusted the compass contribution is to the overall orientation estimation. Instead of using a single value, uses a minimum and maximum value. Trust values will be selected from this range depending on the confidence factor. This can have the effect of smoothing out the compass when the sensor is in motion.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] min_trust_value Minimum accelerometer trust value.
	* \param[in] max_trust_value Maximum accelerometer trust value.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setConfidenceCompassTrustValues(tss_device_id sensor_id, float min_trust_value, float max_trust_value, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Causes the processor to wait for the specified number of microseconds at the end of each update loop. Can be useful for bounding the overall update rate of the sensor if necessary.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] update_rate The microsecond update rate.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setDesiredUpdateRate(tss_device_id sensor_id, U32 update_rate, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief  Set the current reference vector mode. Parameter can be 0 for single static mode, which uses a certain reference vector for the compass and another certain vector for the accelerometer at all times, 1 for single auto mode, which uses (0, -1, 0) as the reference vector for the accelerometer at all times and uses the average angle between the accelerometer and compass to calculate the compass reference vector once upon initiation of this mode, 2 for single auto continuous mode, which works similarly to single auto mode, but calculates this continuously, or 3 for multi-reference mode, which uses a collection of reference vectors for the compass and accelerometer both, and selects which ones to use before each step of the filter.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] mode The reference vector mode.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setReferenceVectorMode(tss_device_id sensor_id, U8 mode, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets the number of times to sample each component sensor for each iteration of the filter. This can smooth out readings at the cost of performance. If this value is set to 0 or 1, no oversampling occursotherwise, the number of samples per iteration depends on the specified parameter, up to a maximum of 10. This setting can be saved to non-volatile flash memory using the Commit Settings command.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] gyro_sample_rate The oversample rate of gryoscope.
	* \param[out] accel_sample_rate The oversample rate of accelormeter.
	* \param[out] compass_sample_rate The oversample rate of compass.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setOversampleRate(tss_device_id sensor_id, U16 gyro_sample_rate, U16 accel_sample_rate, U16 compass_sample_rate, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Enable or disable gyroscope readings as inputs to the orientation estimation. Note that updated gyroscope readings are still accessible via commands. This setting can be saved to non-volatile flash memory using the Commit Settings command.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] enabled Whether or not the gryoscope will be enabled.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setGyroscopeEnabled(tss_device_id sensor_id, U8 enabled, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Enable or disable accelerometer readings as inputs to the orientation estimation. Note that updated accelerometer readings are still accessible via commands. This setting can be saved to non-volatile flash memory using the Commit Settings command.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] enabled Whether or not the accelerometer will be enabled.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setAccelerometerEnabled(tss_device_id sensor_id, U8 enabled, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Enable or disable compass readings as inputs to the orientation estimation. Note that updated compass readings are still accessible via commands. This setting can be saved to non-volatile flash memory using the Commit Settings command.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] enabled Whether or not the compass will be enabled.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setCompassEnabled(tss_device_id sensor_id, U8 enabled, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets the smoothing parameters for the compass and accelerometer.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] enabled Whether or not the compass will be enabled.
	* \param[in] compass_max_smooth_factor The maxium smoothing for the compass. 
	* \param[in] compass_min_smooth_factor The minimum smoothing for the compass. 
	* \param[in] accelerometer_max_smooth_factor The maxium smoothing for the accelerometer. 
	* \param[in] accelerometer_min_smooth_factor The minimum smoothing for the accelerometer. 
	* \param[in] smoothing_magnification Scales the effect of the smoothing.
	* \param[in] smoothing_offset Base offset to the smoothing.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setOrientationSmoothing(tss_device_id sensor_id, float enabled, float compass_max_smooth_factor, float compass_min_smooth_factor, float accelerometer_max_smooth_factor, float accelerometer_min_smooth_factor, float smoothing_magnification, float smoothing_offset, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Gets the smoothing parameters for the compass and accelerometer.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] enabled Whether or not the compass will be enabled.
	* \param[out] compass_max_smooth_factor The maxium smoothing for the compass.
	* \param[out] compass_min_smooth_factor The minimum smoothing for the compass.
	* \param[out] accelerometer_max_smooth_factor The maxium smoothing for the accelerometer.
	* \param[out] accelerometer_min_smooth_factor The minimum smoothing for the accelerometer.
	* \param[out] smoothing_magnification Scales the effect of the smoothing.
	* \param[out] smoothing_offset Base offset to the smoothing.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getOrientationSmoothing(tss_device_id sensor_id, U8* enabled, float* compass_max_smooth_factor, float* compass_min_smooth_factor, float* accelerometer_max_smooth_factor, float* accelerometer_min_smooth_factor, float* smoothing_magnification, float* smoothing_offset, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets alternate directions for each of the natural axes of the sensor. The only parameter is a bitfield representing the possible combinations of axis swapping. The lower 3 bits specify where each of the natural axes appears: 000: X: Right, Y: Up, Z: Forward (left-handed system, standard operation) 001: X: Right, Y: Forward, Z: Up (right-handed system) 002: X: Up, Y: Right, Z: Forward (right-handed system) 003: X: Forward, Y: Right, Z: Up (left-handed system) 004: X: Up, Y: Forward, Z: Right (left-handed system) 005: X: Forward, Y: Up, Z: Right (right-handed system) (For example, using X: Right, Y: Forward, Z: Up means that any values that appear on the positive vertical(Up) axis of the sensor will be the third(Z) component of any vectors and will have a positive sign, and any that appear on the negative vertical axis will be the Z component and will have a negative sign.) The 3 bits above those are used to indicate which axes, if any, should be reversed. If it is cleared, the axis will be pointing in the positive direction. Otherwise, the axis will be pointed in the negative direction. (Note: These are applied to the axes after the previous conversion takes place). Bit 4: Positive/Negative Z (Third resulting component) Bit 5: Positive/Negative Y (Second resulting component) Bit 6: Positive/Negative X (First resulting component) Note that for each negation that is applied, the handedness of the system flips. So, if X and Z are negative and you are using a left-handed system, the system will still be left handed, but if only X is negated, the system will become right-handed.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] axis_direction_byte The axis direction bitfield.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setAxisDirections(tss_device_id sensor_id, U8 axis_direction_byte, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets what percentage of running average to use on the sensor's orientation. This is computed as follows:total_orient = total_orient * percenttotal_orient = total_orient + current_orient * (1  percent)current_orient = total_orientIf the percentage is 0, the running average will be shut off completely. Maximum value is 97%. This setting can be saved to non-volatile flash memory using the Commit Settings command.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] gyro_running_average_percent The running average precentage for the gyroscope.
	* \param[in] accel_running_average_percent The running average precentage for the accelerometer.
	* \param[in] mag_running_average_percent The running average precentage for the magnetometer.
	* \param[in] orient_running_average_percent The running average precentage for the orientation.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setRunningAveragePercent(tss_device_id sensor_id, float gyro_running_average_percent, float accel_running_average_percent, float mag_running_average_percent, float orient_running_average_percent, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets the static compass reference vector for Single Reference Mode.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] reference_vector3 Compass reference vector.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setCompassReferenceVector(tss_device_id sensor_id, const float* reference_vector3, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets the static accelerometer reference vector for Single Reference Mode.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] reference_vector3 Accelerometer reference vector
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setAccelerometerReferenceVector(tss_device_id sensor_id, const float* reference_vector3, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Resets the state of the currently selected filter
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_resetKalmanFilter(tss_device_id sensor_id, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Set accelerometer range. Only parameter is the new accelerometer range, which can be 0 for 2g (Default range), which can be 1 for 4g, or 2 for 8g. Higher ranges can detect and report larger accelerations, but are not as accurate for smaller accelerations. This setting can be saved to non-volatile flash memory using the Commit Settings command.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] accelerometer_range_setting Accelerometer range setting.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setAccelerometerRange(tss_device_id sensor_id, U8 accelerometer_range_setting, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Set filter mode. Used to disable the orientation filter or set the orientation filter mode. Changing this parameter can be useful for tuning filter-performance versus orientation-update rates. Passing in a parameter of 0 places the sensor into IMU mode, a 1 places the sensor into Kalman Filtered Mode (Default mode), a 2 places the sensor into Alternating Kalman Filter Mode, a 3 places the sensor into Complementary Filter Mode, a 4 places the sensor into Quaternion Gradient Descent Filter Mode, and a 5 places the sensor into Magnetoresistive Quaternion Gradient Descent Filter Mode. More information can be found in Section 3.1.5. This setting can be saved to non-volatile flash memory using the Commit Settings command.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] mode The filter mode.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setFilterMode(tss_device_id sensor_id, U8 mode, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Set running average mode. Used to further smooth out the orientation at the cost of higher latency. Passing in a parameter of 0 places the sensor into a static running average mode, a 1 places the sensor into a confidence-based running average mode, which changes the running average factor based upon the confidence factor, which is a measure of how 'in motion' the sensor is. This setting can be saved to non-volatile flash memory using the Commit Settings command.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] mode The running average mode.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setRunningAverageMode(tss_device_id sensor_id, U8 mode, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Set gyroscope range. Only parameter is the new gyroscope range, which can be 0 for 250 DPS, 1 for 500 DPS, or 2 for 2000 DPS (Default range). Higher ranges can detect and report larger angular rates, but are not as accurate for smaller angular rates. This setting can be saved to non-volatile flash memory using the Commit Settings command. 
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] gyroscope_range_setting Gyroscope range setting.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setGyroscopeRange(tss_device_id sensor_id, U8 gyroscope_range_setting, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Set compass range. Only parameter is the new compass range, which can be 0 for 0.88G, 1 for 1.3G (Default range), 2 for 1.9G, 3 for 2.5G, 4 for 4.0G, 5 for 4.7G, 6 for 5.6G, or 7 for 8.1G. Higher ranges can detect and report larger magnetic field strengths but are not as accurate for smaller magnetic field strengths. This setting can be saved to non-volatile flash memory using the Commit Settings command.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] compass_range_setting Compass range setting.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setCompassRange(tss_device_id sensor_id, U8 compass_range_setting, U32* timestamp);

	// Configuration Read Commands
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Returns the current tare orientation as a quaternion.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] quat4 The tare orientation.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getTareAsQuaternion(tss_device_id sensor_id, float* quat4, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Returns the current tare orientation as a rotation matrix.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] matrix9 The tare orientation.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getTareAsRotationMatrix(tss_device_id sensor_id, float* matrix9, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Returns the current accelerometer min and max trust values. If static trust values were set, both of these will be the same.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] min_trust_value Minimum accelerometer trust value.
	* \param[out] max_trust_value Maximum accelerometer trust value.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getAccelerometerTrustValues(tss_device_id sensor_id, float* min_trust_value, float* max_trust_value, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Returns the current compass min and max trust values. If static trust values were set, both of these will be the same.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] min_trust_value Minimum compass trust value.
	* \param[out] max_trust_value Maximum compass trust value.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getCompassTrustValues(tss_device_id sensor_id, float* min_trust_value, float* max_trust_value, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief  Reads the amount of time taken by the last filter update step.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] last_update Last update time in microseconds.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getCurrentUpdateRate(tss_device_id sensor_id, U32* last_update, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Reads the current compass reference vector. Note that this is not valid if the sensor is in Multi Reference Vector mode. 
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] reference_vector Compass reference vector.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getCompassReferenceVector(tss_device_id sensor_id, float* reference_vector, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Reads the current accelerometer reference vector. Note that this is not valid if the sensor is in Multi Reference Vector mode. 
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] reference_vector Accelerometer reference vector.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getAccelerometerReferenceVector(tss_device_id sensor_id, float* reference_vector, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Reads the current reference vector mode. Return value can be 0 for single static, 1 for single auto, 2 for single auto continuous or 3 for multi.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] mode The mode.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getReferenceVectorMode(tss_device_id sensor_id, U8* mode, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Returns a value indicating whether the gyroscope contribution is currently part of the orientation estimate: 0 for off, 1 for on. 
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] enabled Gyroscope enabled value.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getGyroscopeEnabledState(tss_device_id sensor_id, U8* enabled, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Returns a value indicating whether the accelerometer contribution is currently part of the orientation estimate: 0 for off, 1 for on. 
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] enabled Accelerometer enabled value.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getAccelerometerEnabledState(tss_device_id sensor_id, U8* enabled, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Returns a value indicating whether the compass contribution is currently part of the orientation estimate: 0 for off, 1 for on. 
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] enabled Compass enabled value.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getCompassEnabledState(tss_device_id sensor_id, U8* enabled, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Returns a value indicating the current axis direction setup. For more information on the meaning of this value, please refer to the Set Axis Direction command (116).
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] axis_directions Axis direction value.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getAxisDirections(tss_device_id sensor_id, U8* axis_directions, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Returns a value indicating how many times each component sensor is sampled before being stored as raw data. A value of 1 indicates that no oversampling is taking place, while a value that is higher indicates the number of samples per component sensor per filter update step.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] gyro_sample_rate The oversample rate of gryoscope.
	* \param[out] accel_sample_rate The oversample rate of accelormeter.
	* \param[out] compass_sample_rate The oversample rate of compass.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getOversampleRate(tss_device_id sensor_id, U16* gyro_sample_rate, U16* accel_sample_rate, U16* compass_sample_rate, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Returns a value indicating how heavily the orientation estimate is based upon the estimate from the previous frame. For more information on the meaning of this value, please refer to the Set Running Average Percent command (117).
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] gyro_running_average_percent The running average precentage for the gyroscope.
	* \param[out] accel_running_average_percent The running average precentage for the accelerometer.
	* \param[out] mag_running_average_percent The running average precentage for the magnetometer.
	* \param[out] orient_running_average_percent The running average precentage for the orientation.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getRunningAveragePercent(tss_device_id sensor_id, float* gyro_running_average_percent, float* accel_running_average_percent, float* mag_running_average_percent, float* orient_running_average_percent, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Returns the current desired update rate. Note that this value does not indicate the actual update rate, but instead indicates the value that should be spent 'idling' in the main loop. Thus, without having set a specified desired update rate, this value should read 0.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] update_rate The update rate.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getDesiredUpdateRate(tss_device_id sensor_id, U32* update_rate, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Return the current accelerometer measurement range, which can be a 0 for 2g, 1 for 4g or a 2 for 8g.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] accelerometer_range_setting Accelerometer range setting.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getAccelerometerRange(tss_device_id sensor_id, U8* accelerometer_range_setting, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Returns the current filter mode, which can be 0 for IMU mode, 1 for Kalman, 2 for Alternating Kalman, 3 for Complementary, or 4 for Quaternion Gradient Descent. For more information, please refer to the Set Filter Mode command (123).
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] mode The filter mode.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getFilterMode(tss_device_id sensor_id, U8* mode, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Reads the selected mode for the running average, which can be 0 for normal or 1 for confidence.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] timestamp Running average mode.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getRunningAverageMode(tss_device_id sensor_id, U8* mode, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Reads the current gyroscope measurement range, which can be 0 for 250 DPS, 1 for 500 DPS or 2 for 2000 DPS.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] gyroscope_range_setting Gyroscope range setting.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getGyroscopeRange(tss_device_id sensor_id, U8* gyroscope_range_setting, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Reads the current compass measurement range, which can be 0 for 0.88G, 1 for 1.3G, 2 for 1.9G, 3 for 2.5G, 4 for 4.0G, 5 for 4.7G, 6 for 5.6G or 7 for 8.1G.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] compass_range_setting Compass range setting.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getCompassRange(tss_device_id sensor_id, U8* compass_range_setting, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Reads the current euler angle decomposition order.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] order Euler angle decomposition order.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getEulerAngleDecompositionOrder(tss_device_id sensor_id, U8* order, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Reads the current magnetoresistive threshold parameters.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] threshold Magnetoresistive threshold in gauss.
	* \param[out] trust_frames Number of magnetometer trust frames.
	* \param[out] lockout_decay Magnetometer lockout decay value.
	* \param[out] perturbation_detection_value Magnetometer perturbation detection value.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getMagnetoresistiveThreshold(tss_device_id sensor_id, float* threshold, U32* trust_frames, float* lockout_decay, float* perturbation_detection_value, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Reads the current accelerometer threshold parameters.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] threshold Accelerometer threshold in g's.
	* \param[out] lockout_frames Number of accelerometer lockout frames.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getAccelerometerResistanceThreshold(tss_device_id sensor_id, float* threshold, U32* lockout_frames, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Returns the current offset orientation as a quaternion.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] quat4 The tare orientation.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getOffsetOrientationAsQuaternion(tss_device_id sensor_id, float* quat4, U32* timestamp);

	//Calibration Commands
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets the current compass calibration parameters to the specified values. These consist of a bias which is added to the raw data vector and a matrix by which the value is multiplied. This setting can be saved to non-volatile flash memory using the Commit Settings command.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] matrix9 The calibration matrix.
	* \param[in] bias3 The calibration bias.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setCompassCalibrationCoefficients(tss_device_id sensor_id, float* matrix9, float* bias3, U32* timestamp);
	/********************************************//**
	*\ingroup tss_sensor_methods
	* \brief Sets the current accelerometer calibration parameters to the specified values. These consist of a bias which is added to the raw data vector and a matrix by which the value is multiplied.This setting can be saved to non - volatile flash memory using the Commit Settings command.
	* \param[in] sensor_id The tss_device_id that represents the 3 - Space Sensor.
	* \param[in] matrix9 The calibration matrix.
	* \param[in] bias3 The calibration bias.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setAccelerometerCalibrationCoefficients(tss_device_id sensor_id, const float* matrix9, const float* bias3, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Return the current compass calibration parameters. These consist of a bias which is added to the raw data vector and a matrix by which the value is multiplied. This setting can be saved to non-volatile flash memory using the Commit Settings command.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] matrix9 The calibration matrix.
	* \param[out] bias3 The calibration bias.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getCompassCalibrationCoefficients(tss_device_id sensor_id, float* matrix9, float* bias3, U32* timestamp);
	/********************************************//**
	*\ingroup tss_sensor_methods
	* \brief Return the current accelerometer calibration parameters. These consist of a bias which is added to the raw data vector and a matrix by which the value is multiplied.This setting can be saved to non - volatile flash memory using the Commit Settings command.
	* \param[in] sensor_id The tss_device_id that represents the 3 - Space Sensor.
	* \param[out] matrix9 The calibration matrix.
	* \param[out] bias3 The calibration bias.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getAccelerometerCalibrationCoefficients(tss_device_id sensor_id, float* matrix9, float* bias3, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Return the current gyroscope calibration parameters.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] matrix9 The calibration matrix.
	* \param[out] bias3 The calibration bias.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getGyroscopeCalibrationCoefficients(tss_device_id sensor_id, float* matrix9, float* bias3, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Performs auto-gyroscope calibration. Sensor should remain still while samples are taken. The gyroscope bias will be automatically placed into the bias part of the gyroscope calibration coefficient list.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_beginGyroscopeAutoCalibration(tss_device_id sensor_id, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets the current gyroscope calibration parameters to the specified values. These consist of a bias which is added to the raw data vector and a matrix by which the value is multiplied. This setting can be saved to non-volatile flash memory using the Commit Settings command.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] matrix9 The calibration matrix.
	* \param[out] bias3 The calibration bias.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setGyroscopeCalibrationCoefficients(tss_device_id sensor_id, const float* matrix9, const float* bias3, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets the current calibration mode, which can be 0 for Bias, 1 for Scale-Bias and 2 for Ortho-Calibration. For more information, refer to section 3.1.3 Additional Calibration. This setting can be saved to non-volatile flash memory using the Commit Settings command.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] mode The calibration mode.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setCalibrationMode(tss_device_id sensor_id, U8 mode, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Reads the current calibration mode, which can be 0 for Bias, 1 for Scale-Bias and 2 for Ortho-Calibration. For more information, refer to section 3.1.3 Additional Calibration. This setting can be saved to non-volatile flash memory using the Commit Settings command.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] mode The calibration mode.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getCalibrationMode(tss_device_id sensor_id, U8* mode, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets what mode the Auto Compass Calibration system is in. Refer to the manual for more information.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] mode The auto compass calibration mode. (0: Manual, 1: Single, 2: Auto, 3: Auto Continuous)
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setAutoCompassCalibrationMode(tss_device_id sensor_id, U8 mode, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Gets what mode the Auto Compass Calibration system is in. Refer to the manual for more information.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] mode The auto compass calibration mode. (0: Manual, 1: Single, 2: Auto, 3: Auto Continuous)
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getAutoCompassCalibrationMode(tss_device_id sensor_id, U8* mode, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets the settings for the Auto Compass Calibration system. Refer to the manual for a more detailed explaination of these setting.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] too_close_angle Compass samples are not taken if the angle between two samples is too close.
	* \param[in] bias_movement_percentage Determines the allowable variance between new calibration biases and the current running average bias. 
	* \param[in] coplanarity_tolerance Determines the threshold for when a set of samples are considered coplanar.	
	* \param[in] max_averages The max allowable number of calibrations for the running average of the bias.
	* \param[in] max_bad_deviations The max allowable number of calibrations that are too variable before the running average bias is reset.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setAutoCompassCalibrationSettings(tss_device_id sensor_id, float too_close_angle, float bias_movement_percentage, float coplanarity_tolerance, U8 max_averages, U8 max_bad_deviations, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Gets the settings for the Auto Compass Calibration system. Refer to the manual for a more detailed explaination of these setting.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] too_close_angle Compass samples are not taken if the angle between two samples is too close.
	* \param[out] bias_movement_percentage Determines the allowable variance between new calibration biases and the current running average bias. 
	* \param[out] coplanarity_tolerance Determines the threshold for when a set of samples are considered coplanar.	
	* \param[out] max_averages The max allowable number of calibrations for the running average of the bias.
	* \param[out] max_bad_deviations The max allowable number of calibrations that are too variable before the running average bias is reset.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getAutoCompassCalibrationSettings(tss_device_id sensor_id, float* too_close_angle, float* bias_movement_percentage, float* coplanarity_tolerance, U8* max_averages, U8* max_bad_deviations, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Gets the current count of the Auto Compass Calibration system. Refer to the manual for more information.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] count The auto compass calibration count.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getAutoCompassCalibrationCount(tss_device_id sensor_id, U8* count, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Read the wireless hardware address for this sensor or dongle.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getWirelessAddress(tss_device_id sensor_id, U16* address, U32* timestamp);

	//Battery Commands
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Read the current battery level in volts. Note that this value will read as slightly higher than it actually is if it is read via a USB connection.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] battery_voltage Battery level in voltage.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getBatteryVoltage(tss_device_id sensor_id, float* battery_voltage, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Read the current battery lifetime as a percentage of the total. Note that this value will read as slightly higher than it actually is if it is read via a USB connection.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] battery_percent Battery level as a percentage.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getBatteryPercentRemaining(tss_device_id sensor_id, U8* battery_percent, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Returns a value indicating the current status of the battery, which can be a 3 to indicate that the battery is currently not charging, a 2 to indicate that the battery is charging and thus plugged in, or a 1 to indicate that the sensor is fully charged. 
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] battery_charge_status Battery charge status.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getBatteryStatus(tss_device_id sensor_id, U8* battery_charge_status, U32* timestamp);

	//General Commands
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Allows finer-grained control over the sensor LED. Accepts a single parameter that can be 0 for standard, which displays all standard LED status indicators or 1 for static, which displays only the LED color as specified by command 238.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] mode The LED mode.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setLEDMode(tss_device_id sensor_id, U8 mode, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Returns the current sensor LED mode, which can be 0 for standard or 1 for static.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] mode The LED mode.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getLEDMode(tss_device_id sensor_id, U8* mode, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Return all non-volatile flash settings to their original, default settings.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_restoreFactorySettings(tss_device_id sensor_id, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Resets the sensor.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.	
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_softwareReset(tss_device_id sensor_id);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets the current sleep mode of the sensor. Supported sleep modes are 0 for NONE and 1 for IDLE. IDLE mode merely skips all filtering steps. NONE is the default state. 
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] The sleep mode.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setSleepMode(tss_device_id sensor_id, U8 mode, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Reads the current sleep mode of the sensor, which can be 0 for NONE or 1 for IDLE. 
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] The sleep mode.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getSleepMode(tss_device_id sensor_id, U8* mode, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Places the sensor into a special mode that allows firmware upgrades. This will case normal operation until the firmware update mode is instructed to return the sensor to normal operation. For more information on upgrading firmware, refer to the 3-Space Sensor Suite Quick Start Guide.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_enterBootloaderMode(tss_device_id sensor_id, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets the baud rate of the physical UART. This setting does not need to be committed, but will not take effect until the sensor is reset. Valid baud rates are 1200, 2400, 4800, 9600, 19200, 28800, 38400, 57600, 115200 (default), 230400, 460800 and 921600. Note that this is only applicable for sensor types that have UART interfaces.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] baud_rate The Baud rate.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setUARTBaudRate(tss_device_id sensor_id, U32 baud_rate, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Returns the baud rate of the physical UART. Note that this is only applicable for sensor types that have UART interfaces.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] baud_rate The Baud rate.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getUARTBaudRate(tss_device_id sensor_id, U32* baud_rate, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets the communication mode for USB. Accepts one value that can be 0 for CDC (default) or 1 for FTDI.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] mode The USB mode.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setUSBMode(tss_device_id sensor_id, U8 mode, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Returns the current USB communication mode.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] mode The USB mode.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getUSBMode(tss_device_id sensor_id, U8* mode, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets the color of the LED on the sensor to the specified RGB color. This setting can be committed to non-volatile flash memory by calling the Commit Wireless Settings command.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] rgb_color3 The RGB color.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setLEDColor(tss_device_id sensor_id, const float* rgb_color3, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Returns the color of the LED on the sensor.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] rgb_color3 The RGB color.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getLEDColor(tss_device_id sensor_id, float* rgb_color3, U32* timestamp);

	//Wired HID Commands
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Enable or disable streaming of joystick HID data for this sensor.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] enabled_state Joystick enabled state.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setJoystickEnabled(tss_device_id sensor_id, U8 enabled_state, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Enable or disable streaming of mouse HID data for this sensor.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] enabled_state Mouse enabled state.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setMouseEnabled(tss_device_id sensor_id, U8 enabled_state, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Read whether the sensor is currently streaming joystick HID data.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] enabled_state Joystick enabled state.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getJoystickEnabled(tss_device_id sensor_id, U8* enabled_state, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Read whether the sensor is currently streaming mouse HID data.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] enabled_state Mouse enabled state.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getMouseEnabled(tss_device_id sensor_id, U8* enabled_state, U32* timestamp);

	//General HID Commands
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets the operation mode for one of the controls. The first parameter is the control class, which can be 0 for Joystick Axis, 1 for Joystick Button, 2 for Mouse Axis or 3 for Mouse Button. There are two axes and eight buttons on the joystick and mouse. The second parameter, the control index, selects which one of these axes or buttons you would like to modify. The third parameter, the handler index, specifies which handler you want to take care of this control. These can be the following:Turn off this control: 255Axes:Global Axis: 0Screen Point: 1Buttons:Hardware Button: 0Orientation Button: 1Shake Button: 2.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] control_class The control class.
	* \param[in] control_index The control index.
	* \param[in] handler_index The handler index.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setControlMode(tss_device_id sensor_id, U8 control_class, U8 control_index, U8 handler_index, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets parameters for the specified control's operation mode. The control classes and indices are the same as described in command 244. Each mode can have up to 10 data points associated with it. How many should be set and what they should be set to is entirely based on which mode is being used. 
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] control_class The control class.
	* \param[in] control_index The control index.
	* \param[in] data_point_index The data point index.
	* \param[in] data_point The data point.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setControlData(tss_device_id sensor_id, U8 control_class, U8 control_index, U8 data_point_index, float data_point, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brie fReads the handler index of this control's mode. The control classes and indices are the same as described in command 244.		
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] control_class The control class.
	* \param[in] control_index The control index.
	* \param[out] handler_index The handler index.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getControlMode(tss_device_id sensor_id, U8 control_class, U8 control_index, U8* handler_index, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Reads the value of a certain parameter of the specified control's operation mode. The control classes and indices are the same as described in command 244.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] control_class The control class.
	* \param[in] control_index The control index.
	* \param[in] data_point_index The data point index.
	* \param[out] data_point The data point.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getControlData(tss_device_id sensor_id, U8 control_class, U8 control_index, U8 data_point_index, float* data_point, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Detemin_ratioes how long, in frames, the gyros should be disabled after one of the physical buttons on the sensor is pressed. A setting of 0 means they won't be disabled at all. This setting helps to alleviate gyro disturbances cause by the buttons causing small shockwaves in the sensor. 
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor
	* \param[in] number_of_frames Number of frames.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setButtonGyroDisableLength(tss_device_id sensor_id, U8 number_of_frames, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Returns the current button gyro disable length.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor
	* \param[out] number_of_frames Number of frames.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getButtonGyroDisableLength(tss_device_id sensor_id, U8* number_of_frames, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Reads the current state of the sensor's physical buttons. This value returns a byte, where each bit represents the state of the sensor's physical buttons.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] button_state Button state.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getButtonState(tss_device_id sensor_id, U8* button_state, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Puts the mode in absolute or relative mode. This change will not take effect immediately and the sensor must be reset before the mouse will enter this mode. The only parameter can be 0 for absolute (default) or 1 for relative
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] mode Absolute or relative mode.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setMouseAbsoluteRelativeMode(tss_device_id sensor_id, U8 mode, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Return the current mouse absolute/relative mode. Note that if the sensor has not been reset since it has been put in this mode, the mouse will not reflect this change yet, even though the command will.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[inout mode Absolute or relative mode.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getMouseAbsoluteRelativeMode(tss_device_id sensor_id, U8* mode, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets whether the joystick and mouse are present or removed. The first parameter is for the joystick, and can be 0 for removed or 1 for present. The second parameter is for the mouse. If removed, they will not show up as devices on the target system at all. For these changes to take effect, the sensor driver may need to be reinstalled. 
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] joystick Joystick present/removed.
	* \param[in] mouse Mouse present/removed.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setJoystickAndMousePresentRemoved(tss_device_id sensor_id, U8 joystick, U8 mouse, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Returns whether the joystick and mouse are present or removed.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] joystick Joystick present/removed.
	* \param[out] mouse Mouse present/removed.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getJoystickAndMousePresentRemoved(tss_device_id sensor_id, U8* joystick, U8* mouse, U32* timestamp);
	
	//Sensor Pedestrian Tracking functions
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Enables the pedestrian tracking systems.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_enablePedestrianTracking(tss_device_id sensor_id, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Disabled the pedestrian tracking systems.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.	
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_disablePedestrianTracking(tss_device_id sensor_id, U32* timestamp);
		/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Gets the enabled state of the pedestrian tracking systems. 0 for off and 1 for on.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] enabled The enabled state.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getPedestrianTrackingEnabledState(tss_device_id sensor_id, U8* enabled, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets the selected step index in sensor memory so that you can get it with tss_sensor_getStepAtSelectedIndex. The index is auto incremented after calling the get function.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] index The step index.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setSelectedStepIndex(tss_device_id sensor_id, U16 index, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Gets the selected step index. This value is an index into the current step buffer.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] index The step index.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getSelectedStepIndex(tss_device_id sensor_id, U16* index, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets the minimum allowable value of the ratio of the positive step area to the negative step area for a valid step. For more information, refer to section 3.2 Pedestrian Tracking. This setting can be saved to non-volatile flash memory using the Commit Settings command.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] min_ratio The minimum ratio.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setStepRatioMinimum(tss_device_id sensor_id, float min_ratio, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Gets the minimum allowable value of the ratio of the positive step area to the negative step area for a valid step. For more information, refer to section 3.2 Pedestrian Tracking.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] min_ratio The minimum ratio.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getStepRatioMinimum(tss_device_id sensor_id, float* min_ratio, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets the maximum allowable value of the ratio of the positive step area to the negative step area for a valid step. For more information, refer to section 3.2 Pedestrian Tracking. This setting can be saved to non-volatile flash memory using the Commit Settings command.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] max_ratio The maximum ratio.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setStepRatioMaximum(tss_device_id sensor_id, float max_ratio, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Gets the maximum allowable value of the ratio of the positive step area to the negative step area for a valid step. For more information, refer to section 3.2 Pedestrian Tracking.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] max_ratio The maximum ratio.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getStepRatioMaximum(tss_device_id sensor_id, float* max_ratio, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets the minimum allowable duty-cycle for a detected step wave-form. For more information, refer to section 3.2 Pedestrian Tracking. This setting can be saved to non-volatile flash memory using the Commit Settings command.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] min_duty_cycle The minimum duty-cycle.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setStepDutyCycleMinimum(tss_device_id sensor_id, float min_duty_cycle, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Gets the minimum allowable duty-cycle for a detected step wave-form For more information, refer to section 3.2 Pedestrian Tracking.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] min_duty_cycle The minimum duty-cycle.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getStepDutyCycleMinimum(tss_device_id sensor_id, float* min_duty_cycle, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets the maximum allowable duty-cycle for a detected step wave-form. For more information, refer to section 3.2 Pedestrian Tracking. This setting can be saved to non-volatile flash memory using the Commit Settings command.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.	
	* \param[in] max_duty_cycle The maximum duty-cycle.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setStepDutyCycleMaximum(tss_device_id sensor_id, float max_duty_cycle, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Gets the maximum allowable duty-cycle for a detected step wave-form For more information, refer to section 3.2 Pedestrian Tracking.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] max_duty_cycle The maximum duty-cycle.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getStepDutyCycleMaximum(tss_device_id sensor_id, float* max_duty_cycle, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets the minimum allowable amplitude for a detected step wave-form. For more information, refer to section 3.2 Pedestrian Tracking. This setting can be saved to non-volatile flash memory using the Commit Settings command.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] min_amplitude The minimum amplitude.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setStepAmplitudeMinimum(tss_device_id sensor_id, float min_amplitude, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Gets the minimum allowable amplitude for a detected step wave-form. For more information, refer to section 3.2 Pedestrian Tracking.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] min_amplitude The minimum amplitude.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getStepAmplitudeMinimum(tss_device_id sensor_id, float* min_amplitude, U32* timestamp);

	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets the maximum allowable amplitude for a detected step wave-form. For more information, refer to section 3.2 Pedestrian Tracking. This setting can be saved to non-volatile flash memory using the Commit Settings command.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] max_amplitude The maximum amplitude.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setStepAmplitudeMaximum(tss_device_id sensor_id, float max_amplitude, U32* timestamp);

	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Gets the maximum allowable amplitude for a detected step wave-form. For more information, refer to section 3.2 Pedestrian Tracking.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] max_amplitude The maximum amplitude.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getStepAmplitudeMaximum(tss_device_id sensor_id, float* max_amplitude, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets the minimum allowable duration for a detected step wave-form. For more information, refer to section 3.2 Pedestrian Tracking. This setting can be saved to non-volatile flash memory using the Commit Settings command.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] min_step_duration The step duration minimum.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setStepDurationMinimum(tss_device_id sensor_id, float min_step_duration, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Gets the minimum allowable duration for a detected step wave-form. For more information, refer to section 3.2 Pedestrian Tracking. This setting can be saved to non-volatile flash memory using the Commit Settings command.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] max_step_duration The step duration maximum.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getStepDurationMinimum(tss_device_id sensor_id, float* min_step_duration, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets the maximum allowable duration for a detected step wave-form. For more information, refer to section 3.2 Pedestrian Tracking. This setting can be saved to non-volatile flash memory using the Commit Settings command.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] max_step_duration The step duration maximum.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setStepDurationMaximum(tss_device_id sensor_id, float max_step_duration, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Gets the maximum allowable duration for a detected step wave-form. For more information, refer to section 3.2 Pedestrian Tracking. This setting can be saved to non-volatile flash memory using the Commit Settings command.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] max_step_duration The step duration maximum.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getStepDurationMaximum(tss_device_id sensor_id, float* max_step_duration, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets the slope of the stride-to-cadence relation used to convert step time to distance. For more information, refer to section 3.2 Pedestrian Tracking. This setting can be saved to non-volatile flash memory using the Commit Settings command.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] slope The stride slope.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setStepStrideSlope(tss_device_id sensor_id, float slope, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Gets the slope of the stride-to-cadence relation used to convert step time to distance. For more information, refer to section 3.2 Pedestrian Tracking. This setting can be saved to non-volatile flash memory using the Commit Settings command.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] slope The stride slope.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getStepStrideSlope(tss_device_id sensor_id, float* slope, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets the offset of the stride-to-cadence relation used to convert step time to distance. For more information, refer to section 3.2 Pedestrian Tracking. This setting can be saved to non-volatile flash memory using the Commit Settings command.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] offset The stride offset.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setStepStrideOffset(tss_device_id sensor_id, float offset, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets the offset of the stride-to-cadence relation used to convert step time to distance. For more information, refer to section 3.2 Pedestrian Tracking. This setting can be saved to non-volatile flash memory using the Commit Settings command.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] offset The stride offset.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getStepStrideOffset(tss_device_id sensor_id, float* offset, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets the units for data returned from the pedestrian tracking subsystem. 0 for meters (metric), and 1 for feet (imperial).
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] unit_select The selected unit.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setPedestrianTrackingUnits(tss_device_id sensor_id, U8 unit_select, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Gets the units for data returned from the pedestrian tracking subsystem. 0 for meters (metric), and 1 for feet (imperial).
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] unit_select The selected units.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getPedestrianTrackingUnits(tss_device_id sensor_id, U8* unit_select, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets the barometer's altitude offset.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] offset The offset to the barometer's reading.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_setAltitudeOffset(tss_device_id sensor_id, float offset, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Sets the barometer's altitude offset by auto calculating the pressure based on the current height.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] known_height The current height of the device. Input is based off the 'selected units' status.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_autoSetAltitudeOffset(tss_device_id sensor_id, float known_height, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Gets the barometer's altitude offset.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] offset The offset to the barometer's reading.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getAltitudeOffset(tss_device_id sensor_id, float* offset, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Gets the current height of the sensor based on the sensor's pressure reading in the selected units.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] altitude The current altitude of the sensor.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getCurrentAltitude(tss_device_id sensor_id, float* altitude, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Gets the first height of the sensor based on the sensor's pressure reading in the selected units. This height is recorded when the subsystem's state is set to on.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] altitude The first altitude recorded by the sensor on subsystem start.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getFirstAltitude(tss_device_id sensor_id, float* altitude, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Computes the difference between the current height and the first altitude.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] difference The difference between the altitudes.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getAltitudeDifference(tss_device_id sensor_id, float* difference, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Gets the current pressure reading from the barometer.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] millibars The current pressure in millibars.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getCurrentPressure(tss_device_id sensor_id, float* pressure, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Gets the current heading of the device.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] heading The current heading of the device in degrees.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getCurrentHeading(tss_device_id sensor_id, float* heading, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Gets the current step buffer length. This can be used to tell how many steps have been taken so far.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] step_buffer_length The current length of the step buffer.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getStepBufferLength(tss_device_id sensor_id, U16* step_buffer_length, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Computes and returns the total distance traveled from all recored steps in the buffer.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] distance The 'unit_select' dependent distance traveled.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getTotalDistanceTraveled(tss_device_id sensor_id, float* distance, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Gets the current confidence for the step session.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] confidence A percentage that represents the confidence of the current step session.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getStepSessionConfidenceValue(tss_device_id sensor_id, float* confidence, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Gets the latest step from the step buffer. Returns 12 floats: (Index, Heading, Step Distance Vector X, Step Distance Vector Y, Step Distance Vector Length, Step Time, Step Amplitude, Step Duty Cycle, Step Ratio, Altitude, Confidence, Altitude Offset).
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] step_float_array_of_size_12 An array that the function will fill with the step data.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getLatestStep(tss_device_id sensor_id, float* step_data, U32* timestamp); //Califaction
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Gets the selected step from the step buffer. You can set the selected step via tss_sensor_setSelectedStepIndex. Returns 12 floats: (Index, Heading, Step Distance Vector X, Step Distance Vector Y, Step Distance Vector Length, Step Time, Step Amplitude, Step Duty Cycle, Step Ratio, Altitude, Confidence, Altitude Offset).
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] step_float_array_of_size_12 An array that the function will fill with the step data.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getStepAtSelectedIndex(tss_device_id sensor_id, float* step_data, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Gets the selected step from the step buffer. You can set the selected step via tss_sensor_setSelectedStepIndex. Returns 12 floats: (Index, Heading, Step Distance Vector X, Step Distance Vector Y, Step Distance Vector Length, Step Time, Step Amplitude, Step Duty Cycle, Step Ratio, Altitude, Confidence, Altitude Offset).
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] step_float_array_of_size_12 An array that the function will fill with the step data.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_resetSteps(tss_device_id sensor_id, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Closes and then reopens the port which the sensor is on.
	* \param[in] device_id The tss_device_id that represents the 3-Space Sensor.	
	* \param[in] port_open A boolean representing whether or not the port opened successfully.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_resetPort(tss_device_id sensor_id, U8* port_open);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Writes a single page of firmware data to the bootloader for storage.
	* \param[in] device_id The tss_device_id that represents the 3-Space Sensor.	
	* \param[in] page_data The page data, which is encoded as a Hex string.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_writePage(tss_device_id sensor_id, const char* page_data);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Tells the bootloader to enter back into firmware mode. Will not have an effect if firmware process is not complete.
	* \param[in] device_id The tss_device_id that represents the 3-Space Sensor.	
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_enterFirmware(tss_device_id sensor_id);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Gets the bootloader page size for this device.
	* \param[in] device_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] page_size The size a single page of data should be for this device.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_getPageSize(tss_device_id sensor_id, U16* page_size);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Starts the firmware update process by writing the location from where the write will start. The update must be completed past this point, firmware mode is unavailable until a successful firmware update has been completed.
	* \param[in] device_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] address The Hex string that represents where the firmware is written.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_startFirmwareUpdate(tss_device_id sensor_id, const char* address);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Stops the firmware update process by asking the bootloader if we can stop writing pages.
	* \param[in] device_id The tss_device_id that represents the 3-Space Sensor.	
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_stopFirmwareUpdate(tss_device_id sensor_id);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Automatically uses other API functions to read a firmware update file, and perform the update process.
	* \param[in] device_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] file_name The path with filename where a firmware update file can be found.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_autoFirmwareUpdateFromFile(tss_device_id sensor_id, const char* file_name);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Automatically uses other API functions to read an update file string, and perform the update process.
	* \param[in] device_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] update_contents The contents of a firmware update file.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_autoFirmwareUpdateFromString(tss_device_id sensor_id, const char* update_contents);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Writes the contents of 'bytes' to the port the sensor is on. Manual function use is not-recommended and can cause mutiple malfuctions in the API if not properly used.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[in] bytes The bytes to write to the port.
	* \param[in] amount How many bytes from 'bytes' to write to the port.	
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_manualWrite(tss_device_id sensor_id, U8* bytes, U16 amount, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Reads from the port the sensor is on. Manual function use is not-recommended and can cause mutiple malfuctions in the API if not properly used.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
	* \param[out] bytes A pointer to a sized array that can handle a max of 'amount' bytes where the data will be stored once read.
	* \param[in] amount The number of bytes to try to read from the port.
	* \param[out] read_count The number of bytes the port was able to read.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_manualRead(tss_device_id sensor_id, U8* bytes, U16 amount, U16* read_count, U32* timestamp);
	/********************************************//**
	* \ingroup tss_sensor_methods
	* \brief Flushes the input buffer for the sensor port. Manual function use is not-recommended and can cause mutiple malfuctions in the API if not properly used.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.	
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_sensor_manualFlush(tss_device_id sensor_id, U32* timestamp);

	////Dongle functions
    /********************************************//**
    * \ingroup tss_dongle_methods
    * This creates and performs initial setup on the connected 3-Space Dongle.
    * The returned Basestation ID is used to interact with the 3-Space Dongle.
    * When a HUB ID is created, other programs cannot use that serial port until the port is closed.
    * \param[in] port_name The serial port string for the serial port to be queried.
    * \param[out] out_id A tss_device_id will be returned to represent the Dongle.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
	TSS_EXPORT TSS_ERROR tss_createDongle(const char* port_name, tss_device_id* out_id);
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \brief This removes the connected 3-Space Dongle.
    * \param[in] dongle_id The tss_device_id that represents the Prio Basestation.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_removeDongle(tss_device_id dongle_id);

	//Standard device functions
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \brief Checks to see if the specified 3-Space Dongle is connected.
    * \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_dongle_isConnected(tss_device_id dongle_id);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Checks to see if the dongle is connected in bootloader mode.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Dongle.
	* \param[out] in_bootloader The flag which states the mode, 1 for bootloader mode, 0 for firmware mode.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_inBootloader(tss_device_id dongle_id, U8* in_bootloader);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Places the dongle into a special mode that allows firmware upgrades. This will cease normal operation until the firmware update mode is instructed to return the dongle to normal operation. For more information on upgrading firmware, refer to the 3-Space Sensor Suite Quick Start Guide.
	* \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_enterBootloaderMode(tss_device_id dongle_id, U32* timestamp);
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \brief Retrieves the last timestamp logged by the System.
    * \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
    * \param[out] timestamp The returned timestamp in milliseconds.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_dongle_getLastSensorTimestamp(tss_device_id dongle_id, U32* timestamp);
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
    * \param[out] timestamp The returned timestamp in milliseconds.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_dongle_getLastSystemTimestamp(tss_device_id dongle_id, float* timestamp);
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \brief Retrieves the serial number of the 3-Space Dongle.
    * \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
    * \param[out] serial_number The serial number.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_dongle_getSerialNumber(tss_device_id dongle_id, U32* serial_number, U32* timestamp);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Retrieves the hardware version string of the 3-Space Dongle.
	* \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
	* \param[out] version_string The version string.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_getHardwareVersionString(tss_device_id dongle_id, char* hw_version_string);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Retrieves the firmware version string of the 3-Space Dongle.
	* \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
	* \param[out] version_string The version string.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_getFirmwareVersionString(tss_device_id dongle_id, char* fw_version_string);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Writes the contents of 'bytes' to the port the dongle is on. Manual function use is not-recommended and can cause mutiple malfuctions in the API if not properly used.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Dongle.
	* \param[in] bytes The bytes to write to the port.
	* \param[in] amount How many bytes from 'bytes' to write to the port.	
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_manualWrite(tss_device_id dongle_id, U8* bytes, U16 amount, U32* timestamp);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Reads from the port the dongle is on. Manual function use is not-recommended and can cause mutiple malfuctions in the API if not properly used.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Dongle.
	* \param[out] bytes A pointer to a sized array that can handle a max of 'amount' bytes where the data will be stored once read.
	* \param[in] amount The number of bytes to try to read from the port.
	* \param[out] read_count The number of bytes the port was able to read.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_manualRead(tss_device_id dongle_id, U8* bytes, U16 amount, U16* read_count, U32* timestamp);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Flushes the input buffer for the dongle port. Manual function use is not-recommended and can cause mutiple malfuctions in the API if not properly used.
	* \param[in] sensor_id The tss_device_id that represents the 3-Space Dongle.	
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_manualFlush(tss_device_id dongle_id, U32* timestamp);	
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Closes and then reopens the port which the dongle is on.
	* \param[in] device_id The tss_device_id that represents the 3-Space Dongle.	
	* \param[in] port_open A boolean representing whether or not the port opened successfully.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_resetPort(tss_device_id dongle_id, U8* port_open);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Writes a single page of firmware data to the bootloader for storage.
	* \param[in] device_id The tss_device_id that represents the 3-Space Dongle.	
	* \param[in] page_data The page data, which is encoded as a Hex string.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_writePage(tss_device_id dongle_id, const char* page_data);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Tells the bootloader to enter back into firmware mode. Will not have an effect if firmware process is not complete.
	* \param[in] device_id The tss_device_id that represents the 3-Space Dongle.	
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_enterFirmware(tss_device_id dongle_id);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Gets the bootloader page size for this device.
	* \param[in] device_id The tss_device_id that represents the 3-Space Dongle.
	* \param[out] page_size The size a single page of data should be for this device.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_getPageSize(tss_device_id dongle_id, U16* page_size);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Starts the firmware update process by writing the location from where the write will start. The update must be completed past this point, firmware mode is unavailable until a successful firmware update has been completed.
	* \param[in] device_id The tss_device_id that represents the 3-Space Dongle.
	* \param[in] address The Hex string that represents where the firmware is written.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_startFirmwareUpdate(tss_device_id dongle_id, const char* address);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Stops the firmware update process by asking the bootloader if we can stop writing pages.
	* \param[in] device_id The tss_device_id that represents the 3-Space Dongle.	
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_stopFirmwareUpdate(tss_device_id dongle_id);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Automatically uses other API functions to read a firmware update file, and perform the update process.
	* \param[in] device_id The tss_device_id that represents the 3-Space Dongle.
	* \param[in] file_name The path with filename where a firmware update file can be found.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_autoFirmwareUpdateFromFile(tss_device_id dongle_id, const char* file_name);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Automatically uses other API functions to read an update file string, and perform the update process.
	* \param[in] device_id The tss_device_id that represents the 3-Space Dongle.
	* \param[in] update_contents The contents of a firmware update file.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_autoFirmwareUpdateFromString(tss_device_id dongle_id, const char* update_contents);

	//Port control functions
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \brief Open a port to connect to 3-Space Dongle.
    * \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
    * \param[in] port_name The name of the communication port
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_dongle_openPort(tss_device_id dongle_id, const char* port_name);
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \brief Close the connected port
    * \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_dongle_closePort(tss_device_id dongle_id);

	//Header control functions
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \brief Enables timestamps over wireless.
    * \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_dongle_enableTimestampsWireless(tss_device_id dongle_id);
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \brief Disables timestamps over wireless.
    * \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_dongle_disableTimestampsWireless(tss_device_id dongle_id);

	//Wireless sensor functions
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \brief Get the paried 3-Space Sensor.
    * \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
    * \param[in] logical_id The logical ID that represents the 3-Space Sensor.
    * \param[out] out_id The tss_device_id that represents the connected 3-Space Sensor.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_dongle_getWirelessSensor(tss_device_id dongle_id, U8 logical_id, tss_device_id* out_id);
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \brief Remove the paried 3-Space Sensor.
    * \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
    * \param[in] logical_id The logical ID that represents the 3-Space Sensor.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_dongle_removeWirelessSensor(tss_device_id dongle_id, U8 logical_id);
	
	//Streaming functions
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Enables wireless streaming with the given flags.
    * \param[in] dongle_id The tss_device_id that represents the 3-Space Sensor.
    * \param[in] data_flags The data flags reprsenting the data to stream. The flags are TSS_STREAM_TARED_ORIENTATION_AS_QUATERNION, TSS_STREAM_TARED_ORIENTATION_AS_EULER_ANGLES,
    * TSS_STREAM_TARED_ORIENTATION_AS_ROTATION_MATRIX, TSS_STREAM_TARED_ORIENTATION_AS_AXIS_ANGLE, TSS_STREAM_TARED_ORIENTATION_AS_TWO_VECTOR, TSS_STREAM_UNTARED_ORIENTATION_AS_QUATERNION,
    * TSS_STREAM_UNTARED_ORIENTATION_AS_EULER_ANGLES, TSS_STREAM_UNTARED_ORIENTATION_AS_ROTATION_MATRIX, TSS_STREAM_UNTARED_ORIENTATION_AS_AXIS_ANGLE, TSS_STREAM_UNTARED_ORIENTATION_AS_TWO_VECTOR, and more.
    * \param[in] interval The interval to stream data.
    * \param[in] duration The duration of the stream.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_dongle_enableAllSensorsAndStartStreaming(tss_device_id dongle_id, U32 data_flags, U32 interval, U32 duration);
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \brief Start the streaming with the given flags.
    * \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_dongle_startStreaming(tss_device_id dongle_id);
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \brief Stops the streaming
    * \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_dongle_stopStreaming(tss_device_id dongle_id);

	//Call and response functions
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Resets the dongle.
	* \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_softwareReset(tss_device_id dongle_id);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Allows finer-grained control over the dongle LED. Accepts a single parameter that can be 0 for standard, which displays all standard LED status indicators or 1 for static, which displays only the LED color as specified by command 238.
	* \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
	* \param[in] mode The LED Mode.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_setLEDMode(tss_device_id dongle_id, U8 mode, U32* timestamp);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Returns the current sensor LED mode, which can be 0 for standard or 1 for static.
	* \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
	* \param[out] mode The LED Mode.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_getLEDMode(tss_device_id dongle_id, U8* mode, U32* timestamp);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Sets the color of the LED on the dongle to the specified RGB color. This setting can be committed to non-volatile flash memory by calling the Commit Wireless Settings command.
	* \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
	* \param[in] rgb_color3 The RGB color.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_setLEDColor(tss_device_id dongle_id, const float* rgb_color3, U32* timestamp);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Returns the color of the LED on the dongle.
	* \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
	* \param[out] rgb_color3 The RGB color.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_getLEDColor(tss_device_id dongle_id, float* rgb_color3, U32* timestamp);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Gets the wired response header bitfield from the dongle.
	* \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
	* \param[out] bitfield The bitfield returned, see commmand charts for additional info.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_getWiredResponseHeaderBitfield(tss_device_id dongle_id, U32* bitfield, U32* timestamp);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Sets the wired response header bitfield from the dongle.
	* \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
	* \param[in] bitfield The bitfield returned, see commmand charts for additional info.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_setWiredResponseHeaderBitfield(tss_device_id dongle_id, U32 bitfield, U32* timestamp);
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \brief Return the current wireless channel of the  3-Space Dongle. In order for a Dongle to communicate with a 3-Space Sensor, they must both share the same channel.
    * \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
    * \param[out] channel The wireless channel ID.
	* \param[out] timestamp The returned timestamp in milliseconds.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_dongle_getWirelessChannel(tss_device_id dongle_id, U8* channel, U32* timestamp);
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \brief Set the wireless channel of the 3-Space Dongle. In order for a Dongle to communicate with a 3-Space Sensor, they must both share the same channel. Note that channel must be in the range 11 � 26 inclusive.
    * \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
    * \param[in] channel The wireless channel ID.
	* \param[out] timestamp The returned timestamp in milliseconds.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_dongle_setWirelessChannel(tss_device_id dongle_id, U8 channel, U32* timestamp);
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \brief Return the current wireless panID of the 3-Space Dongle. In order for a Dongle to communicate with a 3-Space Sensor, they must both share the same panID.
    * \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
    * \param[out] pan_id The wireless pan ID.
	* \param[out] timestamp The returned timestamp in milliseconds.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_dongle_getWirelessPanID(tss_device_id dongle_id, U16* pan_id, U32* timestamp);
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \brief Set the wireless panID of the 3-Space Dongle. In order for a Dongle to communicate with a 3-Space Sensor, they must both share the same panID. Note that panIDs must be in the range 1 � 65534 inclusive.
    * \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
    * \param[in] pan_id The wireless pan ID.
	* \param[out] timestamp The returned timestamp in milliseconds.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_dongle_setWirelessPanID(tss_device_id dongle_id, U16 pan_id, U32* timestamp);
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \brief Return the serial number of the PrioVR Device.
    * \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
    * \param[in] logical_id The tss_device_id that represents the 3-Space Device.
    * \param[out] serial_number The serial number.
	* \param[out] timestamp The returned timestamp in milliseconds.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_dongle_getSerialNumberAtLogicalID(tss_device_id dongle_id, U8 logical_id, U32* serial_number, U32* timestamp);
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \brief Set the serial number of the PrioVR Device.
    * \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
    * \param[in] logical_id The tss_device_id that represents the 3-Space Device.
    * \param[in] serial_number The serial number.
	* \param[out] timestamp The returned timestamp in milliseconds.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_dongle_setSerialNumberAtLogicalID(tss_device_id dongle_id, U8 logical_id, U32 serial_number, U32* timestamp);
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \brief Commits all committable wireless settings to non-volatile memory so that they will persist beyond a powercycle. For more information on Commitable Settings, please refer to the PrioVR User's Manual.
    * \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
	* \param[out] timestamp The returned timestamp in milliseconds.
    * \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_ERROR tss_dongle_commitWirelessSettings(tss_device_id dongle_id, U32* timestamp);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Return the wireless hardware address for the 3-Space Dongle.
	* \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
	* \param[out] address The wireless address.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_getWirelessAddress(tss_device_id dongle_id, U16* address, U32* timestamp);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Sets the wireless communication's current asynchronous flush mode, which can be 0 for auto flush and 1 for manual flush.
	* \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
	* \param[in] mode The auto flush mode.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_setWirelessStreamingAutoFlushMode(tss_device_id dongle_id, U8 mode, U32* timestamp);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Returns the wireless communication's current asynchronous flush mode, which can be 0 for auto flush and 1 for manual flush.
	* \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
	* \param[in] mode The auto flush mode.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_getWirelessStreamingAutoFlushMode(tss_device_id dongle_id, U8* mode, U32* timestamp);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Sets the current manual flush bitfield representing which logical Ids will respond to asynchronous requests.
	* \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
	* \param[in] manual_flush_bitfield The bitfield.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_setWirelessStreamingManualFlushBitfield(tss_device_id dongle_id, U16 manual_flush_bitfield, U32* timestamp);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Returns the current manual flush bitfield representing which logical Ids will respond to asynchronous requests.
	* \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
	* \param[out] manual_flush_bitfield The bitfield.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_getWirelessStreamingManualFlushBitfield(tss_device_id dongle_id, U16* manual_flush_bitfield, U32* timestamp);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Sends out a timestamp synchronization broadcast message to all wireless sensors that are listening on the same channel and PanID as the dongle. The message will essentially set each receiving sensor's timestamp to the same timestamp as stored in the dongle. 
	* \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_broadcastSynchronizationPulse(tss_device_id dongle_id, U32* timestamp);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Return the noise levels for each of the 16 wireless channels. A higher value corresponds to a noisier channel, which can significantly impact wireless reception and throughput.
	* \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
	* \param[out] channel_strengths16 The noise levels.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_getWirelessChannelNoiseLevels(tss_device_id dongle_id, U8* channel_strengths16, U32* timestamp);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Set the number of times a dongle will attempt to re-transmit a data request after timing out. Default value is 3. This setting can be committed to non-volatile flash memory by calling the Commit Wireless Settings command.
	* \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
	* \param[in] retries The amount of times to retry the command.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_setWirelessRetries(tss_device_id dongle_id, U8 retries, U32* timestamp);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Return the number of times a dongle will attempt to re-transmit a data request after timing out. Default value is 3. This setting can be committed to non-volatile flash memory by calling the Commit Wireless Settings command.
	* \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
	* \param[out] retries The amount of times to retry the command.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_getWirelessRetries(tss_device_id dongle_id, U8* retries, U32* timestamp);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief The dongle can simultaneously service up to sixteen individual data requests to wireless sensors. As sensors respond, requests are removed from the table. In the case that too many requests are sent to the dongle in too short a period, the dongle will begin tossing them out. This value will return the number of slots currently open. If this value is 0, no more wireless requests will be handled until some are internally processed.
	* \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
	* \param[out] slots_open The number of open slots.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_getWirelessSlotsOpen(tss_device_id dongle_id, U8* slots_open, U32* timestamp);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Returns a value indicating the reception strength of the most recently received packet. Higher values indicate a stronger link.
	* \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
	* \param[out] last_packet_signal_strength The packet strength of the last packet.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_getSignalStrength(tss_device_id dongle_id, U8* last_packet_signal_strength, U32* timestamp);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Configures the response header for data returned over a wireless connection. The only parameter is a four-byte bitfield that detemin_ratioes which data is prepended to all data responses. The following bits are used: 0x1: (1 byte) Success/Failure, with non-zero values representing failure.0x2: (4 bytes) Timestamp, in microseconds.0x4: (1 byte) Command echooutputs the called command. Returns 0xFF for streamed data. 0x8: (1 byte) Additive checksum over returned data, but not including response header.0x10: (1 byte) Logical ID0x20: (4 bytes) Serial number0x40: (1 byte) Data length, returns the length of the requested data, not including response header.This setting can be committed to non-volatile flash memory by calling the Commit Wireless Settings command.
	* \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
	* \param[in] header_bitfield The bitfield.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_setWirelessResponseHeaderBitfield(tss_device_id dongle_id, U32 header_bitfield, U32* timestamp);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Return the current wireless response header bitfield.
	* \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
	* \param[out] header_bitfield The bitfield.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_getWirelessResponseHeaderBitfield(tss_device_id dongle_id, U32* header_bitfield, U32* timestamp);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Specify the interval at which HID information is requested by the dongle. The default and minimum value is 15ms in synchronous HID mode. In asynchronous HID mode, the minimum is 5ms. This setting can be committed to non-volatile flash memory by calling the Commit Wireless Settings command.
	* \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
	* \param[in] HID_update_rate The update rate.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_setWirelessHIDUpdateRate(tss_device_id dongle_id, U8 HID_update_rate, U32* timestamp);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Return the interval at which HID information is requested by the dongle. 
	* \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
	* \param[out] HID_update_rate The update rate.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_getWirelessHIDUpdateRate(tss_device_id dongle_id, U8* HID_update_rate, U32* timestamp);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Sets the current wireless HID communication mode. Supplying a 0 makes wireless HID communication synchronous, while a 1 makes wireless HID asynchronous. For more information, refer to Section 3.3.4 Wireless Joystick/Mouse. This setting can be committed to non-volatile flash memory by calling the Commit Wireless Settings command.
	* \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
	* \param[in] HID_communication_mode The HID communication mode.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_setWirelessHIDAsynchronousMode(tss_device_id dongle_id, U8 HID_communication_mode, U32* timestamp);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Returns the current wireless HID communication mode, which can be a 0 for synchronous wireless HID or a 1 for asynchronous wireless HID.
	* \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
	* \param[out] HID_communication_mode The HID communication mode.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_getWirelessHIDAsynchronousMode(tss_device_id dongle_id, U8* HID_communication_mode, U32* timestamp);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Causes the sensor at the specified logical ID to return joystick HID data. Passing a -1 will disable wireless joystick data. For more information, refer to Section 3.3.4 Wireless Joystick/Mouse.
	* \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
	* \param[in] logical_ID The joystick logical ID.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_setJoystickLogicalID(tss_device_id dongle_id, U8 logical_ID, U32* timestamp);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Causes the sensor at the specified logical ID to return mouse HID data. Passing a -1 will disable wireless mouse data. For more information, refer to Section 3.3.4 Wireless Joystick/Mouse.
	* \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
	* \param[in] logical_ID The mouse logical ID.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_setMouseLogicalID(tss_device_id dongle_id, U8 logical_ID, U32* timestamp);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Returns the current logical ID of the joystick-enabled sensor or -1 if none exists.
	* \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
	* \param[out] logical_ID The joystick logical ID.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_getJoystickLogicalID(tss_device_id dongle_id, U8* logical_ID, U32* timestamp);
	/********************************************//**
	* \ingroup tss_dongle_methods
	* \brief Returns the current logical ID of the mouse-enabled sensor or -1 if none exists.
	* \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
	* \param[out] logical_ID The mouse logical ID.
	* \param[out] timestamp The returned timestamp in milliseconds.
	* \return TSS_NO_ERROR(0) if successful, non zero if an error occurred.
	***********************************************/
	TSS_EXPORT TSS_ERROR tss_dongle_getMouseLogicalID(tss_device_id dongle_id, U8* logical_ID, U32* timestamp);

    //Port functions
    /********************************************//**
    * \ingroup tss_port_methods
    * \brief Find Prio Devices connected to the computer.
    * \param[in] find_flags The flags for what devices to look for. 1-TSS_UNKNOWN, 2-TSS_BOOTLOADER, 4-TSS_USB, 8-TSS_EMBEDDED, 16-TSS_WIRELESS, 32-TSS_DONGLE, 64-TSS_DATALOGGER, 128-TSS_BLUETOOTH, 2147483647-TSS_FIND_ALL_KNOWN.
    ***********************************************/
	TSS_EXPORT void tss_findSensorPorts(U32 find_flags);
    /********************************************//**
    * \ingroup tss_port_methods
    * \brief Return the COM Port, and attached 3-Space Device type on the next unused COM Port.
    * \param[out] port_name The COM Port.
    * \param[out] sensor_type The type of 3-Space Device.
    * \return U32 if successful 1, if error 0.
    ***********************************************/
	//port_name must be TSS_PORT_NAME_SIZE characters long
	TSS_EXPORT TSS_ERROR tss_getNextSensorPort(char* port_name, U16* sensor_type, U8* connection_type);

#ifdef __cplusplus
}
#endif
