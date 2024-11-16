#ifndef ENCODER_H
#define ENCODER_H

#include "stm32f4xx_hal.h"
#include "Topic.h"
#include "dsp.h"

void Driver_Encoder_Init();
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* htim_encoder);

#endif /* ENCODER_H */ 