#pragma once

int initUart(void);
void SendUartMessage(int uartFd, const char* dataToSend);
void closeUart(void);