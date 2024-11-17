/* 
 * Author: Syaoran
 * Created on: 2024-11-16
 */
#ifndef __BUTTER_WORTH_FILTOR_H
#define __BUTTER_WORTH_FILTOR_H

#include "arm_math.h"

#define MOTOR_FILTER_STAGES_NUM     1
#define PENDULUM_FILTER_STAGES_NUM  1

/* { bo, b1, b2, a1, a2 } */
#define COEFFICIENT_NUMBER          5

#endif /* __BUTTER_WORTH_FILTOR_H */