#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

QueueHandle_t CommuniQueue;
SemaphoreHandle_t CommuniSemaphore;

#define CPR 500               // Encoder Count Per Revolution
#define PPR           (CPR*4.0f) // Encoder Pulse Per Revolution  (2 Channel A-B)
#define GEAR          14
#define SAMPLE_TIME   10      // ms
#define Ts            ((float)SAMPLE_TIME * 0.001f)  // s
#define pi            3.14159265359f

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

extern int16_t PendulumCnt;
float v = 0.0f, x = 0.0f, ThetaCart = 0.0f;
float vdesign, xdesign;

extern int16_t MotorCnt, PreMotorCnt, DeltaCnt;

int64_t DutyCycle;
/**
 * 
 */
void MainTask(void * xTaskParameters) {

  for(;;) {
    v = ((float)DeltaCnt * 2.0f * pi) / (Ts * GEAR * PPR) ;   // rad/s
    PreMotorCnt = MotorCnt;
    ThetaCart = (MotorCnt * 2.0f * pi) / (GEAR * PPR);
    // x = (ThetaCart * 1.5f * 14.0f) / (2.0f);

    DutyCycle = PID_Pos(xdesign, ThetaCart, Ts);
    char SIG[100];
    sprintf(SIG, "N1 O d%d\n", DutyCycle);
    SendBuffer(&huart2, SIG);

    /* Buffer dùng để truyền dữ liệu cho Matlab*/
    // char buffer[100];
    // sprintf(buffer, "%ld,%0.2f\n", DutyCycle, v); 

    // xQueueSendToBack(CommuniQueue, buffer, portMAX_DELAY);
    // xSemaphoreGive(CommuniSemaphore);
    vTaskDelay(pdMS_TO_TICKS(SAMPLE_TIME));
  }
}

/**
 * 
 */
char buffer[100];
void CommunicationTask(void * xTaskParameters) {
  for(;;) {
    if (xQueueReceive(CommuniQueue, buffer, portMAX_DELAY) == pdTRUE) {
      SendBuffer(&huart3, buffer);
    }
  }
}

int main(void)
{
  /* Configuraion */
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

  Driver_USART_Init();
  Driver_Encoder_Init();

  /* USER CODE BEGIN 2 */
  // CommuniQueue = xQueueCreate(5, sizeof(char) * 100);
  // vSemaphoreCreateBinary(CommuniSemaphore);

  xTaskCreate(MainTask, NULL, configMINIMAL_STACK_SIZE, NULL, 3, NULL);
  // xTaskCreate(CommunicationTask, NULL, configMINIMAL_STACK_SIZE, NULL, 2, NULL);

  vTaskStartScheduler();

  /* USER CODE END 2 */

  /* Infinite loop */
  while (1) {}
}


