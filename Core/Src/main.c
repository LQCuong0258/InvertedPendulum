#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

static struct Computer computer;
static struct Active * computer_AO = &computer.super;

static struct Estimator estimator;
static struct Active * estimator_AO = &estimator.super;

static struct Motor motor;
static struct Active * motor_AO = &motor.super;

int main (void) {
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

  Driver_USART_Init();
  Driver_Encoder_Init();

  Computer.new(&computer);
  computer.super.start(computer_AO, 1, 15, 1000);

  Estimator.new(&estimator);
  estimator.super.start(estimator_AO, 2, 20, 3000);

  Motor.new(&motor);
  motor.super.start(motor_AO, 3, 15, 2000);

  vTaskStartScheduler();
}