#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "stm32f4xx_hal.h"
#include "stdint.h"

int64_t PI_Vel(float SetPoint, float CurrentValue, float Ts);
int64_t PID_Pos(float SetPoint, float CurrentValue, float Ts);

#endif /* CONTROLLER_H */ 