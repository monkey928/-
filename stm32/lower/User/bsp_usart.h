#ifndef BSP_USART_H
#define BSP_USART_H

#include "stm32f10x.h"
#include <stdint.h>

void USART1_BspInit(uint32_t baudrate);
void USART1_SendByte(uint8_t byte);
uint8_t USART1_SendBuffer(const uint8_t *buf, uint16_t len);
void USART1_TxIrqHandler(void);

#endif
