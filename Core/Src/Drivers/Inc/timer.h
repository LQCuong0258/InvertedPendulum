#ifndef __TIMER_H
#define __TIMER_H

#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "Active_Object.h"
#include "Estimator.h"
#include "Topic.h"

void Driver_Encoder_Init();
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* htim_encoder);

#endif /* __TIMER_H */ 