#ifndef TOPIC_H
#define TOPIC_H

#include "stdint.h"
#include "arm_math.h"

typedef struct {
    int16_t PendulumCnt;
    int32_t MotorCnt;
} Encoder;

typedef struct {
    float32_t position;
    float32_t velocity;
    float32_t acceleration;
    float32_t Prev_position;
} StateData;

typedef struct {
    StateData Motor;
    StateData Cart;
    StateData Pendulum;
} State;

/* Pulse width command */
typedef struct {
  int16_t d;
} PWC;

typedef struct {
  char message[7];
  uint8_t length;
} RecivedMessage;

#endif /* TOPIC_H */ 