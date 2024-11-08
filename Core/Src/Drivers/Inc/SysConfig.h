#ifndef SYSCONFIG_H
#define SYSCONFIG_H

#include "stm32f4xx_hal.h"

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void Error_Handler(void);

#endif /* SYSCONFIG_H */ 