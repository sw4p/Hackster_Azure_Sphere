/*
 *  Some of the code in this file was copied from ST Micro.  Below is their required information.
 *
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 * This file handles the serial communication between Azure Sphere and the 
 * OBD-II module.
 * Owner: Swapnil Verma
 * E-mail: usav[dot]swapnil[at]gmail[dot]com
 */
#include <errno.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// applibs_versions.h defines the API struct versions to use for applibs APIs.
#include "applibs_versions.h"
#include "epoll_timerfd_utilities.h"
#include <applibs/log.h>
#include <applibs/uart.h>

#include "deviceTwin.h"
#include "azure_iot_utilities.h"
#include "mt3620_avnet_dev.h"
#include "build_options.h"
#include "uart.h"

int uartFd = -1;
static int obdPidTimerFd = -1;
#define receiveBufferSize 256
static uint8_t receiveBuffer[receiveBufferSize + 1]; // allow extra byte for string termination
static ssize_t bytesRead;

// Extern variable
extern int epollFd;
extern volatile sig_atomic_t terminationRequired;

/// <summary>
///     Helper function to send a fixed message via the given UART.
/// </summary>
/// <param name="uartFd">The open file descriptor of the UART to write to</param>
/// <param name="dataToSend">The data to send over the UART</param>
void SendUartMessage(int uartFd, const char* dataToSend)
{
	size_t totalBytesSent = 0;
	size_t totalBytesToSend = strlen(dataToSend);
	int sendIterations = 0;
	while (totalBytesSent < totalBytesToSend) {
		sendIterations++;

		// Send as much of the remaining data as possible
		size_t bytesLeftToSend = totalBytesToSend - totalBytesSent;
		const char* remainingMessageToSend = dataToSend + totalBytesSent;
		ssize_t bytesSent = write(uartFd, remainingMessageToSend, bytesLeftToSend);
		if (bytesSent < 0) {
			Log_Debug("ERROR: Could not write to UART: %s (%d).\n", strerror(errno), errno);
			terminationRequired = true;
			return;
		}

		totalBytesSent += (size_t)bytesSent;
	}

	Log_Debug("Sent %zu bytes over UART in %d calls.\n", totalBytesSent, sendIterations);
}

/// <summary>
///     Handle UART event: if there is incoming data, print it.
/// </summary>
static void UartEventHandler(EventData* eventData)
{
	// Make buffer NULL before reading anything into it
	memset(receiveBuffer, NULL, receiveBufferSize+1);
	bytesRead = 0;

	// Read incoming UART data. It is expected behavior that messages may be received in multiple
	// partial chunks.
	bytesRead = read(uartFd, receiveBuffer, receiveBufferSize);
	if (bytesRead < 0) {
		Log_Debug("ERROR: Could not read UART: %s (%d).\n", strerror(errno), errno);
		terminationRequired = true;
		return;
	}

	if (bytesRead > 0) {
		// Null terminate the buffer to make it a valid string, and print it
		receiveBuffer[bytesRead] = 0;
		Log_Debug("UART received %d bytes: '%s'.\n", bytesRead, (char*)receiveBuffer);
	}
}

/// <summary>
///     Read and report latest data from the vehicle.
/// </summary>
void ObdPidTimerEventHandler(EventData* eventData)
{
	// Send PID to read coolant temperature
	SendUartMessage( uartFd, "01 05" );
	// 01[2]34[5]67[8]90
	if (bytesRead > 0) {
		uint8_t data[2] = {'\0'};
		data[0] = receiveBuffer[6];
		data[1] = receiveBuffer[7];
		int dataDecimal = strtol(data, NULL, 0) - 40;
		Log_Debug("Coolant Temperature is %d.\n", dataDecimal);

#if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))
		// Allocate memory for a telemetry message to Azure
		char* pjsonBuffer = (char*)malloc(JSON_BUFFER_SIZE);
		if (pjsonBuffer == NULL) {
			Log_Debug("ERROR: not enough memory to send telemetry");
		}

		// construct the telemetry message
		snprintf(pjsonBuffer, JSON_BUFFER_SIZE, "{\"rpm\": \"%d\"}", dataDecimal);

		Log_Debug("\n[Info] Sending telemetry: %s\n", pjsonBuffer);
		AzureIoT_SendMessage(pjsonBuffer);
		free(pjsonBuffer);
#endif
	}
}

/// <summary>
///     Initializes the UART interface.
/// </summary>
/// <returns>0 on success, or -1 on failure</returns>
int initUart(void) {
	// Create a UART_Config object, open the UART and set up UART event handler
	UART_Config uartConfig;
	UART_InitConfig(&uartConfig);
	uartConfig.baudRate = 38400;
	uartConfig.flowControl = UART_FlowControl_None;
	uartFd = UART_Open(AVT_SK_CM1_ISU0_UART, &uartConfig);
	if (uartFd < 0) {
		Log_Debug("ERROR: Could not open UART: %s (%d).\n", strerror(errno), errno);
		return -1;
	}

	// event handler data structures. Only the event handler field needs to be populated.
	static EventData uartEventData = { .eventHandler = &UartEventHandler };
	if (RegisterEventHandlerToEpoll(epollFd, uartFd, &uartEventData, EPOLLIN) != 0) {
		return -1;
	}

	// Init the epoll interface to periodically run the ObdPidTimerEventHandler routine where we read the
	// vehicle data and report it

	// Define the period in the build_options.h file
	struct timespec obdPidReadPeriod = { .tv_sec = OBD_PID_READ_PERIOD_SECONDS,.tv_nsec = OBD_PID_READ_PERIOD_NANO_SECONDS };
	// event handler data structures. Only the event handler field needs to be populated.
	static EventData obdPidEventData = { .eventHandler = &ObdPidTimerEventHandler };
	obdPidTimerFd = CreateTimerFdAndAddToEpoll(epollFd, &obdPidReadPeriod, &obdPidEventData, EPOLLIN);
	if (obdPidTimerFd < 0) {
		return -1;
	}

	return( 0 );
}

/// <summary>
///     Closes the UART interface File Descriptors.
/// </summary>
void closeUart(void) {
	CloseFdAndPrintError(uartFd, "Uart");
}