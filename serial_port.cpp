/****************************************************************************
*
*   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
*   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
*           Jaycee Lock,    <jaycee.lock@gmail.com>
*           Lorenz Meier,   <lm@inf.ethz.ch>
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
* 3. Neither the name PX4 nor the names of its contributors may be
*    used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/

/**
* @file serial_port.cpp
*
* @brief Serial interface functions
*
* Functions for opening, closing, reading and writing via serial ports
*
* @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
* @author Jaycee Lock,    <jaycee.lock@gmail.com>
* @author Lorenz Meier,   <lm@inf.ethz.ch>
*
*/


// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "serial_port.h"


// ----------------------------------------------------------------------------------
//   Serial Port Manager Class
// ----------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
Serial_Port::
Serial_Port(const char *uart_name_, int baudrate_)
{
	initialize_defaults();
	uart_name = uart_name_;
	baudrate = baudrate_;
}

Serial_Port::
Serial_Port()
{
	initialize_defaults();
}

Serial_Port::
~Serial_Port()
{
	// destroy mutex
	pthread_mutex_destroy(&lock);
}

void
Serial_Port::
initialize_defaults()
{
	// Initialize attributes
	debug = false;
	fd = -1;
	status = SERIAL_PORT_CLOSED;

	uart_name = (char*)"/dev/ttyUSB0";
	baudrate = 57600;

	// Start mutex
	int result = pthread_mutex_init(&lock, NULL);
	if (result != 0)
	{
		printf("\n mutex init failed\n");
		throw 1;
	}
}


// ------------------------------------------------------------------------------
//   Read from Serial - unix
// ------------------------------------------------------------------------------
//int
//Serial_Port::
//read_message(mavlink_message_t &message)
//{
//	uint8_t          cp;
//	mavlink_status_t status;
//	uint8_t          msgReceived = false;
//
//	// --------------------------------------------------------------------------
//	//   READ FROM PORT
//	// --------------------------------------------------------------------------
//
//	// this function locks the port during read
//	int result = _read_port(cp);
//
//
//	// --------------------------------------------------------------------------
//	//   PARSE MESSAGE
//	// --------------------------------------------------------------------------
//	if (result > 0)
//	{
//		// the parsing
//		msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);
//
//		// check for dropped packets
//		if ( (lastStatus.packet_rx_drop_count != status.packet_rx_drop_count) && debug )
//		{
//			printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
//			unsigned char v=cp;
//			fprintf(stderr,"%02x ", v);
//		}
//		lastStatus = status;
//	}
//
//	// Couldn't read from port
//	else
//	{
//		fprintf(stderr, "ERROR: Could not read from fd %d\n", fd);
//	}
//
//	// --------------------------------------------------------------------------
//	//   DEBUGGING REPORTS
//	// --------------------------------------------------------------------------
//	if(msgReceived && debug)
//	{
//		// Report info
//		printf("Received message from serial with ID #%d (sys:%d|comp:%d):\n", message.msgid, message.sysid, message.compid);
//
//		fprintf(stderr,"Received serial data: ");
//		unsigned int i;
//		uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
//
//		// check message is write length
//		unsigned int messageLength = mavlink_msg_to_send_buffer(buffer, &message);
//
//		// message length error
//		if (messageLength > MAVLINK_MAX_PACKET_LEN)
//		{
//			fprintf(stderr, "\nFATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE\n");
//		}
//
//		// print out the buffer
//		else
//		{
//			for (i=0; i<messageLength; i++)
//			{
//				unsigned char v=buffer[i];
//				fprintf(stderr,"%02x ", v);
//			}
//			fprintf(stderr,"\n");
//		}
//	}
//
//	// Done!
//	return msgReceived;
//}

// ------------------------------------------------------------------------------
//   Read from Serial - win32
// ------------------------------------------------------------------------------
int
Serial_Port::
read_message(mavlink_message_t &message)
{

	{
		uint8_t          cp;
		mavlink_status_t status;
		uint8_t          msgReceived = false;

		// --------------------------------------------------------------------------
		//   READ FROM PORT
		// --------------------------------------------------------------------------

		// this function locks the port during read
		int result = _read_port(cp);


		// --------------------------------------------------------------------------
		//   PARSE MESSAGE
		// --------------------------------------------------------------------------
		if (result > 0)
		{
			// the parsing
			msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);

			// check for dropped packets
			if ((lastStatus.packet_rx_drop_count != status.packet_rx_drop_count) && debug)
			{
				printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
				unsigned char v = cp;
				fprintf(stderr, "%02x ", v);
			}
			lastStatus = status;
		}

		// Couldn't read from port
		else
		{
			fprintf(stderr, "ERROR: Could not read from fd %d\n", fd);
		}

		// --------------------------------------------------------------------------
		//   DEBUGGING REPORTS
		// --------------------------------------------------------------------------
		if (msgReceived && debug)
		{
			// Report info
			printf("Received message from serial with ID #%d (sys:%d|comp:%d):\n", message.msgid, message.sysid, message.compid);

			fprintf(stderr, "Received serial data: ");
			unsigned int i;
			uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

			// check message is write length
			unsigned int messageLength = mavlink_msg_to_send_buffer(buffer, &message);

			// message length error
			if (messageLength > MAVLINK_MAX_PACKET_LEN)
			{
				fprintf(stderr, "\nFATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE\n");
			}

			// print out the buffer
			else
			{
				for (i = 0; i<messageLength; i++)
				{
					unsigned char v = buffer[i];
					fprintf(stderr, "%02x ", v);
				}
				fprintf(stderr, "\n");
			}
		}

		// Done!
		return msgReceived;
	}

	// ------------------------------------------------------------------------------
	//   Write to Serial - unix
	// ------------------------------------------------------------------------------
	//int
	//Serial_Port::
	//write_message(const mavlink_message_t &message)
	//{
	//	char buf[300];
	//
	//	// Translate message to buffer
	//	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);
	//
	//	// Write buffer to serial port, locks port while writing
	//	int bytesWritten = _write_port(buf,len);
	//
	//	return bytesWritten;
	//}

	// ------------------------------------------------------------------------------
	//   Write to Serial - win32
	// ------------------------------------------------------------------------------
	int
	Serial_Port::
	write_message(const mavlink_message_t &message)
	{
		char buf[300];

		// Translate message to buffer
		unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);

		int bytesWritten = _write_port(buf, len);

		return bytesWritten;
	}

	// ------------------------------------------------------------------------------
	//   Open Serial Port - unix
	// ------------------------------------------------------------------------------
	/**
	* throws EXIT_FAILURE if could not open the port
	*/
	//void
	//Serial_Port::
	//open_serial()
	//{
	//
	//	// --------------------------------------------------------------------------
	//	//   OPEN PORT
	//	// --------------------------------------------------------------------------
	//	printf("OPEN PORT\n");
	//
	//	fd = _open_port(uart_name);
	//
	//	// Check success
	//	if (fd == -1)
	//	{
	//		printf("failure, could not open port.\n");
	//		throw EXIT_FAILURE;
	//	}
	//
	//	// --------------------------------------------------------------------------
	//	//   SETUP PORT
	//	// --------------------------------------------------------------------------
	//	bool success = _setup_port(baudrate, 8, 1, false, false);
	//
	//	// --------------------------------------------------------------------------
	//	//   CHECK STATUS
	//	// --------------------------------------------------------------------------
	//	if (!success)
	//	{
	//		printf("failure, could not configure port.\n");
	//		throw EXIT_FAILURE;
	//	}
	//	if (fd <= 0)
	//	{
	//		printf("Connection attempt to port %s with %d baud, 8N1 failed, exiting.\n", uart_name, baudrate);
	//		throw EXIT_FAILURE;
	//	}
	//
	//	// --------------------------------------------------------------------------
	//	//   CONNECTED!
	//	// --------------------------------------------------------------------------
	//	printf("Connected to %s with %d baud, 8 data bits, no parity, 1 stop bit (8N1)\n", uart_name, baudrate);
	//	lastStatus.packet_rx_drop_count = 0;
	//
	//	status = true;
	//
	//	printf("\n");
	//
	//	return;
	//
	//}

	// ------------------------------------------------------------------------------
	//   Open Serial Port - win32
	// ------------------------------------------------------------------------------
	/**
	* throws EXIT_FAILURE if could not open the port
	*/
	void
		Serial_Port::
		open_serial()
	{
		// --------------------------------------------------------------------------
		//   OPEN PORT
		// --------------------------------------------------------------------------
		printf("OPEN PORT\n");

		fd = _open_port(uart_name);

		// Check success
		if (fd == -1)
		{
			printf("failure, could not open port.\n");
			throw EXIT_FAILURE;
		}

		// --------------------------------------------------------------------------
		//   SETUP PORT
		// --------------------------------------------------------------------------
		bool success = _setup_port(baudrate, 8, 1, false, false);

		// --------------------------------------------------------------------------
		//   CHECK STATUS
		// --------------------------------------------------------------------------
		if (!success)
		{
			printf("failure, could not configure port.\n");
			throw EXIT_FAILURE;
		}
		if (fd <= 0)
		{
			printf("Connection attempt to port %s with %d baud, 8N1 failed, exiting.\n", uart_name, baudrate);
			throw EXIT_FAILURE;
		}

		// --------------------------------------------------------------------------
		//   CONNECTED!
		// --------------------------------------------------------------------------
		printf("Connected to %s with %d baud, 8 data bits, no parity, 1 stop bit (8N1)\n", uart_name, baudrate);
		lastStatus.packet_rx_drop_count = 0;

		status = true;

		printf("\n");

		return;

	}

	// ------------------------------------------------------------------------------
	//   Close Serial Port - unix
	// ------------------------------------------------------------------------------
	//void
	//Serial_Port::
	//close_serial()
	//{
	//	printf("CLOSE PORT\n");
	//
	//	int result = close(fd);
	//
	//	if ( result )
	//	{
	//		fprintf(stderr,"WARNING: Error on port close (%i)\n", result );
	//	}
	//
	//	status = false;
	//
	//	printf("\n");
	//
	//}


	// ------------------------------------------------------------------------------
	//   Close Serial Port - win32
	// ------------------------------------------------------------------------------
	void
		Serial_Port::
		close_serial()
	{
		//Closing the serial port is the easiest task under the
		//	Windows operating system.It consists of calling a
		//	single function, CloseFile(), and checking the return
		//	value.Code that demonstrates this can be found be -
		//	low.
		//Close the fileHandle, thus
		//releasing the device.
		if (!CloseFile(fileHandle)) {
			fprintf(stderr, "Warning, could not stop serial port\n");
		}
	}

	// ------------------------------------------------------------------------------
	//   Convenience Functions - unix
	// ------------------------------------------------------------------------------
	//void
	//Serial_Port::
	//start()
	//{
	//	open_serial();
	//}
	//
	//void
	//Serial_Port::
	//stop()
	//{
	//	close_serial();
	//}

	// ------------------------------------------------------------------------------
	//   Convenience Functions - win32
	// ------------------------------------------------------------------------------
	void
		Serial_Port::
		start()
	{
		open_serial();
	}

	void
		Serial_Port::
		stop()
	{
		close_serial();
	}

	// ------------------------------------------------------------------------------
	//   Quit Handler - unix
	// ------------------------------------------------------------------------------
	//void
	//Serial_Port::
	//handle_quit( int sig )
	//{
	//	try {
	//		stop();
	//	}
	//	catch (int error) {
	//		fprintf(stderr,"Warning, could not stop serial port\n");
	//	}
	//}

	// ------------------------------------------------------------------------------
	//   Quit Handler - win32
	// ------------------------------------------------------------------------------
	void
		Serial_Port::
		handle_quit(int sig)
	{
		//Close the fileHandle, thus
		//releasing the device.
		if (!CloseFile(fileHandle)) {
			fprintf(stderr, "Warning, could not stop serial port\n");
		}
	}


	// ------------------------------------------------------------------------------
	//   Helper Function - Open Serial Port File Descriptor - unix
	// ------------------------------------------------------------------------------
	// Where the actual port opening happens, returns file descriptor 'fd'
	//int
	//Serial_Port::
	//_open_port(const char* port)
	//{
	//	// Open serial port
	//	// O_RDWR - Read and write
	//	// O_NOCTTY - Ignore special chars like CTRL-C
	//	fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
	//
	//	// Check for Errors
	//	if (fd == -1)
	//	{
	//		/* Could not open the port. */
	//		return(-1);
	//	}
	//
	//	// Finalize
	//	else
	//	{
	//		fcntl(fd, F_SETFL, 0);
	//	}
	//
	//	// Done!
	//	return fd;
	//}


	// ------------------------------------------------------------------------------
	//   Helper Function - Open Serial Port File Descriptor - win32
	// ------------------------------------------------------------------------------
	// Where the actual port opening happens, returns fileHandle 'fileHandle'
	int
		Serial_Port::
		_open_port(const char* gszPort)
	{

		//The function used to open the serial port in the Win -
		//dows operating system is CreateFile() which is used
		//as follows :

		HANDLE fileHandle;
		fileHandle =
			CreateFile(
				//the name of the port (as a string)
				//eg. COM1, COM2, COM4
				gszPort,
				//must have read AND write access to
				//port
				GENERIC_READ | GENERIC_WRITE,
				//sharing mode
				//ports CANNOT be shared
				//(hence the 0)
				0,
				//security attributes
				//0 here means that this file handle
				//cannot be inherited
				0,
				//The port MUST exist before-hand
				//we cannot create serial ports
				OPEN_EXISTING,
				//Overlapped/Non-Overlapped Mode.
				//This paper will deal with
				//non-overlapped communication.
				//To use overlapped communication
				//replace 0 with
				//FFILE_FLAG_OVERLAPPED
				0,
				//HANDLE of a template file
				//which will supply attributes and
				//permissions.  Not used with
				//port access.
				0);

		//CreateFile() returns a HANDLE object that can then
		//be used to access the port.If CreateFile() fails, the
		//HANDLE object that is returned is invalid and va -
		//lidity can be tested using the following code :
		if (fileHandle == INVALID_HANDLE_VALUE) {
			//error handling code here
			/* Could not open the port. */
			return(-1);
		}

		//Provided that the serial port is successfully opened
		//the next step is to configure it to the specific appli -
		//cation.

		return filehandle;
	}

	// ------------------------------------------------------------------------------
	//   Helper Function - Setup Serial Port - unix
	// ------------------------------------------------------------------------------
	// Sets configuration, flags, and baud rate
	//bool
	//Serial_Port::
	//_setup_port(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control)
	//{
	//	// Check file descriptor
	//	if(!isatty(fd))
	//	{
	//		fprintf(stderr, "\nERROR: file descriptor %d is NOT a serial port\n", fd);
	//		return false;
	//	}
	//
	//	// Read file descritor configuration
	//	struct termios  config;
	//	if(tcgetattr(fd, &config) < 0)
	//	{
	//		fprintf(stderr, "\nERROR: could not read configuration of fd %d\n", fd);
	//		return false;
	//	}
	//
	//	// Input flags - Turn off input processing
	//	// convert break to null byte, no CR to NL translation,
	//	// no NL to CR translation, don't mark parity errors or breaks
	//	// no input parity check, don't strip high bit off,
	//	// no XON/XOFF software flow control
	//	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
	//						INLCR | PARMRK | INPCK | ISTRIP | IXON);
	//
	//	// Output flags - Turn off output processing
	//	// no CR to NL translation, no NL to CR-NL translation,
	//	// no NL to CR translation, no column 0 CR suppression,
	//	// no Ctrl-D suppression, no fill characters, no case mapping,
	//	// no local output processing
	//	config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
	//						 ONOCR | OFILL | OPOST);
	//
	//	#ifdef OLCUC
	//		config.c_oflag &= ~OLCUC;
	//	#endif
	//
	//	#ifdef ONOEOT
	//		config.c_oflag &= ~ONOEOT;
	//	#endif
	//
	//	// No line processing:
	//	// echo off, echo newline off, canonical mode off,
	//	// extended input processing off, signal chars off
	//	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	//
	//	// Turn off character processing
	//	// clear current char size mask, no parity checking,
	//	// no output processing, force 8 bit input
	//	config.c_cflag &= ~(CSIZE | PARENB);
	//	config.c_cflag |= CS8;
	//
	//	// One input byte is enough to return from read()
	//	// Inter-character timer off
	//	config.c_cc[VMIN]  = 1;
	//	config.c_cc[VTIME] = 10; // was 0
	//
	//	// Get the current options for the port
	//	////struct termios options;
	//	////tcgetattr(fd, &options);
	//
	//	// Apply baudrate
	//	switch (baud)
	//	{
	//		case 1200:
	//			if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0)
	//			{
	//				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
	//				return false;
	//			}
	//			break;
	//		case 1800:
	//			cfsetispeed(&config, B1800);
	//			cfsetospeed(&config, B1800);
	//			break;
	//		case 9600:
	//			cfsetispeed(&config, B9600);
	//			cfsetospeed(&config, B9600);
	//			break;
	//		case 19200:
	//			cfsetispeed(&config, B19200);
	//			cfsetospeed(&config, B19200);
	//			break;
	//		case 38400:
	//			if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0)
	//			{
	//				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
	//				return false;
	//			}
	//			break;
	//		case 57600:
	//			if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0)
	//			{
	//				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
	//				return false;
	//			}
	//			break;
	//		case 115200:
	//			if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0)
	//			{
	//				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
	//				return false;
	//			}
	//			break;
	//
	//		// These two non-standard (by the 70'ties ) rates are fully supported on
	//		// current Debian and Mac OS versions (tested since 2010).
	//		case 460800:
	//			if (cfsetispeed(&config, B460800) < 0 || cfsetospeed(&config, B460800) < 0)
	//			{
	//				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
	//				return false;
	//			}
	//			break;
	//		case 921600:
	//			if (cfsetispeed(&config, B921600) < 0 || cfsetospeed(&config, B921600) < 0)
	//			{
	//				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
	//				return false;
	//			}
	//			break;
	//		default:
	//			fprintf(stderr, "ERROR: Desired baud rate %d could not be set, aborting.\n", baud);
	//			return false;
	//
	//			break;
	//	}
	//
	//	// Finally, apply the configuration
	//	if(tcsetattr(fd, TCSAFLUSH, &config) < 0)
	//	{
	//		fprintf(stderr, "\nERROR: could not set configuration of fd %d\n", fd);
	//		return false;
	//	}
	//
	//	// Done!
	//	return true;
	//}


	// ------------------------------------------------------------------------------
	//   Helper Function - Setup Serial Port - win32
	// ------------------------------------------------------------------------------
	// Sets configuration, flags, and baud rate
	bool
		Serial_Port::
		_setup_port(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control)
	{
		//In Windows, setting up the serial port requires three
		//steps.
		//1. Create a DCB object and initialize it using the
		//function BuildCommDCB().
		//2. Set the serial port settings using the initialized
		//DCB object using the function SetCommState().
		//3. Set the size of the serial port read and write
		//buffers using SetupComm().
		//Code to accomplish this can be found below.
		DCB dcb; //create the dcb
				 //first, set every field
				 //of the DCB to 0
				 //to make sure there are
				 //no invalid values
		FillMemory(&dcb, sizeof(dcb), 0);
		//set the length of the DCB
		dcb.DCBlength = sizeof(dcb);
		//try to build the DCB
		//The function takes a string that is formatted as
		//speed,parity,data size,stop bits the speed is the speed
		//of the device in BAUD the parity is the type or parity used
		//--n for none
		//--e for even
		//--o for odd
		//the data size is the number of bits
		//  that make up a work (typically 8)
		//the stop bits is the
		//    number of stop bits used
		//  typically 1 or 2

		switch (baud)
		{
		case 1200:
			if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 1800:
			cfsetispeed(&config, B1800);
			cfsetospeed(&config, B1800);
			break;
		case 9600:
			cfsetispeed(&config, B9600);
			cfsetospeed(&config, B9600);
			break;
		case 19200:
			cfsetispeed(&config, B19200);
			cfsetospeed(&config, B19200);
			break;
		case 38400:
			if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 57600:
			if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 115200:
			if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;

			// These two non-standard (by the 70'ties ) rates are fully supported on
			// current Debian and Mac OS versions (tested since 2010).
		case 460800:
			if (cfsetispeed(&config, B460800) < 0 || cfsetospeed(&config, B460800) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 921600:
			if (cfsetispeed(&config, B921600) < 0 || cfsetospeed(&config, B921600) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		default:
			fprintf(stderr, "ERROR: Desired baud rate %d could not be set, aborting.\n", baud);
			return false;

			break;
		}




		if (!BuildCommDCB("9600,n,8,1", &dcb)) {
			return false;
		}


		//set the state of fileHandle to be dcb
		//returns a boolean indicating success
		//or failure
		if (!SetCommState(filehandle, &dcb)) {
			return false;
		}


		//set the buffers to be size 1024
		//of fileHandle
		//Also returns a boolean indicating
		//success or failure
		if (!SetupComm(fileHandle,
			//in queue
			1024,
			//out queue
			1024))
		{
			return false;
		}


		//Next, the timeouts of the port must be set.It is im -
		//portant to note that if the timeouts of a port are not
		//set in windows, the API states that undefined results
		//will occur.This leads to difficulty debugging because
		//the errors are inconsistent and sporadic.To set the
		//timeouts of the port an object of type COMMTIME -
		//OUTS must be created and initialized and then ap -
		//plied to the port using the function SetCommTime -
		//outs(), as the code below demonstrates.
		COMMTIMEOUTS cmt; //create the object
						  //the maximum amount of time
						  //allowed to pass between
						  //the arrival of two bytes on
						  //the read line (in ms)
		cmt.ReadIntervalTimeout = 1000;
		//value used to calculate the total
		//time needed for a read operation
		//which is
		//  (num bytes to read) * (timeout)
		// in ms
		cmt.ReadTotalTimeoutMultiplier = 1000;
		//This value is added to
		//the previous one to generate
		//the timeout value
		//for a single read operation (in ms)
		cmt.ReadTotalTimeoutConstant = 1000;
		//the next two values are the same
		//as their read counterparts, only
		//applying to write operations
		cmt.WriteTotalTimeoutConstant = 1000;
		cmt.WriteTotalTimeoutMultiplier = 1000;
		//set the timeouts of fileHandle to be
		//what is contained in cmt
		//returns boolean success or failure


		if (!SetCommTimeouts(fileHandle, &cmt)) {
			//error code goes here
			fprintf(stderr, "\nERROR: could not set configuration of fd %d\n", fileHandle);
		}


		//Provided all the configuration functions returned suc -
		//cess, the serial port is now ready to be used to send
		//and receive data.It may be necessary, depending
		//on the application, to re - configure the serial port.It
		//may be necessary, for instance, to change the speed
		//of the port, or the timeout values in the middle of an
		//application.
	}


	// ------------------------------------------------------------------------------
	//   Read Port with Lock - unix
	// ------------------------------------------------------------------------------
	//int
	//Serial_Port::
	//_read_port(uint8_t &cp)
	//{
	//
	//	// Lock
	//	pthread_mutex_lock(&lock);
	//
	//	int result = read(fd, &cp, 1);
	//
	//
	//	// Unlock
	//	pthread_mutex_unlock(&lock);
	//
	//	return result;
	//}

	// ------------------------------------------------------------------------------
	//   Read Port with Lock - win32
	// ------------------------------------------------------------------------------
	int
		Serial_Port::
		_read_port(uint8_t &cp)
	{

		// Lock
		pthread_mutex_lock(&lock);

		//int result = read(fd, &cp, 1);

		data = &cp;

		/******************************
		*
		* Reading from a file
		*
		*******************************/
		//the amount of the data actually
		//read will be returned in
		//this variable
		DWORD read = -1;
		ReadFile(
			//the HANDLE that we
			//are reading from
			fileHandle,
			//a pointer to an array
			//of words that we
			//want to read
			data,
			//the size of the
			//array of values to
			//be read
			size,
			//the address of a DWORD
			//that the number of words
			//actually read will
			//be stored in
			&read,
			//a pointer to an
			//overlapped_reader struct
			//that is used in overlapped
			//reading.  NULL in out case
			NULL);

		// Unlock
		pthread_mutex_unlock(&lock);

		return read;
	}

	// ------------------------------------------------------------------------------
	//   Write Port with Lock - unix
	// ------------------------------------------------------------------------------
	//int
	//Serial_Port::
	//_write_port(char *buf, unsigned len)
	//{
	//
	//	// Lock
	//	pthread_mutex_lock(&lock);
	//
	//	// Write packet via serial link
	//	const int bytesWritten = static_cast<int>(write(fd, buf, len));
	//
	//	// Wait until all data has been written
	//	tcdrain(fd);
	//
	//	// Unlock
	//	pthread_mutex_unlock(&lock);
	//
	//	return bytesWritten;
	//}

	// ------------------------------------------------------------------------------
	//   Write Port with Lock - win32
	// ------------------------------------------------------------------------------
	int
		Serial_Port::
		_write_port(char *buf, unsigned len)
	{

		// Lock
		pthread_mutex_lock(&lock);

		// Write packet via serial link
		const int bytesWritten = static_cast<int>(write(fd, buf, len));

		data = buff;
		size = len;

		/******************************
		*
		* Writing to a file
		*
		*******************************/
		//the amount of the data actually
		//written will be returned in
		//this variable
		DWORD write = -1;

		WriteFile(
			//the HANDLE that we
			//are writing to
			fileHandle,
			//a pointer to an array
			//of words that we
			//want to write
			data,
			//the size of the
			//array of values to
			//be written
			size,
			//the address of a DWORD
			//that the number of words
			//actually written will
			//be stored in
			&write,
			//a pointer to an
			//overlapped_reader struct
			//that is used in overlapped
			//writing.  NULL in out case
			NULL);
		//Because we are using non - overlapped input / output
		//	operations the ReadFile() and WriteFile() operations
		//	will block until the requested data is either received
		//	or the operation times out.

		// Wait until all data has been written
		tcdrain(fd);

		// Unlock
		pthread_mutex_unlock(&lock);


		return write;
	}

