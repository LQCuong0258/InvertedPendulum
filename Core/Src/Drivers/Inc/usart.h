#ifndef USART_H
#define USART_H

#include "stm32f4xx_hal.h"
#include "string.h"

void Driver_USART_Init();
void HAL_UART_MspInit(UART_HandleTypeDef* huart);
void SendBuffer(UART_HandleTypeDef *huart, char* buffer);

#endif /* USART_H */ 