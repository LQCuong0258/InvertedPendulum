#include "usart.h"

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

extern struct Active * AO_Computer;
extern struct Active * AO_Motor;
extern struct Computer * computer;
extern struct Motor * motor; 

extern uint8_t rx_byte_data;

static const uint32_t TEN_POWER[] = {1, 10, 100, 1000};

static int16_t ExtractMessage(RecivedMessage * received_message) {
    int16_t d = 0;
    int16_t sign = 1;
    uint8_t n = received_message->length;
    uint8_t num;

    for(int index = 0; index < n; index++) {
        /* Để phân biệt giá trị âm hay dương của xung pwm */
        if(received_message->message[index] == '-') {
            sign = -1;
            continue;
        }

        /**
         * Đây là phương pháp sử dụng giá trị trong bảng mã ASCII
         * để chuyển đổi từ chuỗi sang số nguyên
         */
        num = received_message->message[index] - '0';
        d = d + (int16_t)(num * TEN_POWER[n - 1 - index]);
    }

    return d * sign;
}

/**
  * @brief  Tx Transfer completed callbacks.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(huart->Instance == USART2) {
        static const Event motor_sended_event = { COMMAND_SENDED_SIG };
        AO_Motor->postFromISR(AO_Motor, &motor_sended_event, &xHigherPriorityTaskWoken);
    }
    else if(huart->Instance == USART3) {
        static const Event sensor_event = { SENSOR_SENDED_SIG };
        AO_Computer->postFromISR(AO_Computer, &sensor_event, &xHigherPriorityTaskWoken);      
    }

    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

/**
  * @brief This function will be called if number of 
  * recived bytes equal data Size declared in HAL_UART_Receive_IT.
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(huart->Instance == USART3) {
        static RecivedMessage recived_message = { .length = 0 }; 
        static uint8_t is_data_comming = 0;
        static char byte_data;
        static const uint8_t max_length = 4;
        byte_data = rx_byte_data; 
    
        if(is_data_comming == 1) { /* Have recived 'S' */
            
            if(byte_data != 'S' && byte_data != 10) { /* Not 'S' and '\n' */
                recived_message.message[recived_message.length] = byte_data;
                recived_message.length++;
                if(recived_message.length > max_length) {
                    recived_message.length = 0;
                    is_data_comming = 0;
                }
            }
            else if(byte_data == 10)  { /* End of Line '\n' */
                static PWC pulse_width_command;
                static int16_t d;
                static const Event pwc_evt = {PWC_TRIGGER_SIG};
                d = ExtractMessage(&recived_message);
                pulse_width_command.d = d;
                xQueueOverwriteFromISR(motor->pwc_sub, &pulse_width_command, &xHigherPriorityTaskWoken);
                AO_Motor->postFromISR(AO_Motor, &pwc_evt, &xHigherPriorityTaskWoken);
                recived_message.length = 0;
                is_data_comming = 0;
            }
        }
        else {
            if(byte_data == 'S') { /* Witnessed double 'S' */
                if(recived_message.length != 0) recived_message.length = 0;
                is_data_comming = 1;
            }          
        }

        /*Enable Rx Interrupt, waiting for new byte data*/
        HAL_UART_Receive_IT(huart, &rx_byte_data, 1);
    }

    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}


static void UART_Init(UART_HandleTypeDef* huart);

/**
  * @brief This function initialize USART1.
  */
void Driver_USART_Init()
{
    huart2.Instance = USART2;
    huart3.Instance = USART3;
    UART_Init(&huart2);
    UART_Init(&huart3);
}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(huart->Instance==USART2)
    {
        /* Peripheral clock enable */
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        
        /** USART2 GPIO Configuration
         * PA2     ------> USART2_TX
         * PA3     ------> USART2_RX
         */
        GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* USART1 interrupt Init */
        HAL_NVIC_SetPriority(USART2_IRQn, 9, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
    }
    else if(huart->Instance==USART3)
    {
        /* Peripheral clock enable */
        __HAL_RCC_USART3_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /** USART3 GPIO Configuration
         * PB10     ------> USART3_TX
         * PB11     ------> USART3_RX
         */
        GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* USART3 interrupt Init */
        HAL_NVIC_SetPriority(USART3_IRQn, 10, 0);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
    }
}

static void UART_Init(UART_HandleTypeDef* huart)
{

    /**
     * Feature: Send signal control to Driver CC-SMART MSD_E20
     */
    if(huart->Instance == USART2)
    {
        huart->Init.BaudRate = 115200;
        huart->Init.WordLength = UART_WORDLENGTH_8B;
        huart->Init.StopBits = UART_STOPBITS_1;
        huart->Init.Parity = UART_PARITY_NONE;
        huart->Init.Mode = UART_MODE_TX_RX;
        huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
        huart->Init.OverSampling = UART_OVERSAMPLING_16;
        HAL_UART_Init(huart);
    }

    /**
     * Feature: Communicate with Matlab
     */
    else if(huart->Instance == USART3)
    {
        huart->Init.BaudRate = 921600;
        huart->Init.WordLength = UART_WORDLENGTH_8B;
        huart->Init.StopBits = UART_STOPBITS_1;
        huart->Init.Parity = UART_PARITY_NONE;
        huart->Init.Mode = UART_MODE_TX_RX;
        huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
        huart->Init.OverSampling = UART_OVERSAMPLING_16;
        HAL_UART_Init(huart);
    }
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart3);
}

void SendBuffer(UART_HandleTypeDef *huart, char* buffer) {
    uint64_t length = strlen(buffer);
    HAL_UART_Transmit_IT(huart, (uint8_t*) buffer, length);
}
