#ifndef THREADS_H
#define THREADS_H

#include "stm32f2xx_hal.h"
#include "cmsis_os.h"

#include "global.h"
#include "fg.h"

#if CHECK_THREADS == 1
 uint32_t writeMessageStartTick, readPacketStartTick, parsePacketStartTick, readIOStartTick, /*writeIOStartTick,*/ monitorStartTick, uiStartTick;
 uint32_t writeMessageEndTick, readPacketEndTick, parsePacketEndTick, readIOEndTick, /*writeIOEndTick,*/ monitorEndTick, uiEndTick;
 uint32_t retryWaitTick;
 uint32_t blinkStartTick;
 uint32_t blinkEndTick;
#endif

 uint8_t toggleFlag;

osMessageQId buttonQID;
osMessageQId displayDigitsQID;

osThreadId blinkTID, uartTID;
osThreadId writeMessageTID, readPacketTID, parsePacketTID, readIOTID, /*writeIOTID,*/ monitorTID, uiTID;

uint8_t initThreads();

void blinkThread(void const *argument);
void uiThread(void const *argument);
void uartThread(void const *argument);

//void commsThread(void const *argument);
void writeMessageThread(void const *argument);
void readPacketThread(void const *argument);
void parsePacketThread(void const *argument);

void readIOThread(void const *argument);
//void writeIOThread(void const *argument);

void monitorThread(void const *argument);
#endif

