#ifndef COMMS_H
#define COMMS_H

#include "stm32f2xx_hal.h"
#include "cmsis_os.h"

#define DMA_BUFFER_LENGTH 256
#define RX_BUFFER_LENGTH 256
#define TX_BUFFER_LENGTH 512
#define PACKET_BUFFER_LENGTH 32
#define RESPONSE_BUFFER_LENGTH 256

#define DMA_TIMEOUT_MS  10

/* Definition for USARTx's DMA */
#define USARTx_TX_DMA_CHANNEL             DMA1_Stream6
#define USARTx_RX_DMA_CHANNEL             DMA1_Stream7

/* Definition for USARTx's NVIC */
#define USARTx_DMA_TX_IRQn                DMA2_Channel6_IRQn
#define USARTx_DMA_RX_IRQn                DMA2_Channel7_IRQn
#define USARTx_DMA_TX_IRQHandler          DMA2_Channel6_IRQHandler
#define USARTx_DMA_RX_IRQHandler          DMA2_Channel7_IRQHandler

typedef struct
{
    volatile uint8_t  flag;     /* Timeout event flag */
    uint16_t timer;             /* Timeout duration in msec */
    uint16_t prevCNDTR;         /* Holds previous value of DMA_CNDTR */
} DMA_Event_t;

UART_HandleTypeDef *handleUART2;
uint8_t flagByteTransmitted;
uint8_t flagPacketSent;
char lastMsg[RESPONSE_BUFFER_LENGTH];
char dma_rx_buf[DMA_BUFFER_LENGTH];
char rxBuffer[RX_BUFFER_LENGTH], txBuffer[TX_BUFFER_LENGTH];
char* packetBuffer[PACKET_BUFFER_LENGTH];
char data[DMA_BUFFER_LENGTH];

unsigned int rxMessageTail, rxMessageHead, txMessageTail, txMessageHead;
unsigned int packetTail, packetHead, packetPointer, flagPacketReceived;

uint8_t crc_calcCrc8(void *data_pointer, uint16_t number_of_bytes);
uint8_t initComms();
uint8_t writeMessage(char* msg);
void sendResponse(char* response);

#endif
