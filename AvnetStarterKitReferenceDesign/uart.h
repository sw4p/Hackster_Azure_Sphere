/*
 * Owner: Swapnil Verma
 * E-mail: usav[dot]swapnil[at]gmail[dot]com
 */
#pragma once

int initUart(void);
void SendUartMessage(int uartFd, const char* dataToSend);
void closeUart(void);