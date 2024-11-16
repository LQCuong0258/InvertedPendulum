#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

QueueHandle_t CommuniQueue;
SemaphoreHandle_t CommuniSemaphore;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

extern Encoder encoder_topic;
struct DSP * data_processor;
State state_topic = {.Motor = {0.0f}, .Cart = {0.0f}, .Pendulum = {0.0f}};

// float theta = 0.0f, omega = 0.0f, Prev_theta = 0.0f;

int64_t checkCnt = 0;
/**
 * 
 */
void MainTask(void * xTaskParameters) {

  for(;;) {    
    /* Cấp phát động bộ đệm */
    data_processor->procesNewData(data_processor, &encoder_topic, &state_topic);

    // DutyCycle = PID_Pos(xdesign, ThetaCart, Ts);
    // char SIG[100];
    // sprintf(SIG, "N1 O d%d\n", DutyCycle);
    // SendBuffer(&huart2, SIG);

    /* Buffer dùng để truyền dữ liệu cho Matlab*/
    char buffer[100];
    sprintf(buffer,
            "S%0.6f %0.6f %0.6f %0.6f\n",
            state_topic.Cart.position,
            state_topic.Cart.velocity,
            state_topic.Pendulum.position,
            state_topic.Pendulum.velocity);

    SendBuffer(&huart3, buffer);

    // xQueueSendToBack(CommuniQueue, buffer, portMAX_DELAY);
    // xSemaphoreGive(CommuniSemaphore);
    vTaskDelay(pdMS_TO_TICKS(1));
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

    // vTaskDelay(pdMS_TO_TICKS(SAMPLE_TIME));
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

  data_processor = (struct DSP * ) pvPortMalloc( sizeof(struct DSP) );
  dsp_new(data_processor);
  // StateSystem= (struct DSP * ) pvPortMalloc( sizeof(struct DSP) );

  CommuniQueue = xQueueCreate(5, sizeof(char) * 100);
  // vSemaphoreCreateBinary(CommuniSemaphore);

  xTaskCreate(MainTask, NULL, configMINIMAL_STACK_SIZE, NULL, 3, NULL);
  // xTaskCreate(CommunicationTask, NULL, configMINIMAL_STACK_SIZE, NULL, 2, NULL);

  vTaskStartScheduler();

  /* USER CODE END 2 */

  /* Infinite loop */
  while (1) {}
}