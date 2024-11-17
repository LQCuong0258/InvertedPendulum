/* 
 * Author: Syaoran
 * Created on: 2024-11-17
 */
#ifndef __COMPUTOR_H
#define __COMPUTOR_H

#include "stm32f4xx_hal.h"
#include "Active_Object.h"
#include "usart.h"

typedef struct Computer Computer;

typedef enum ComputerEvent {
    SENSOR_SENDED_SIG = USER_SIG + 1,
    STATE_UPDATED_SIG,                  /* Cập nhật trạng thái mới */
    NEW_MESSAGE_SIG                     /* Buffer mới do Matlab gửi về */
} ComputerEvent;



struct Computer {
    // Members --------------------------------
    Active super;                       /* Kế thừa Active Object */
    QueueHandle_t state_sub;
    QueueHandle_t received_message_sub; /* Queue để nhận tin nhắn từ Matlab */



    // Methods --------------------------------------------------------
    Status (*initial)(Computer *const self, Event const * const event);
    Status (*wait)(Computer *const self, Event const * const event);
    Status (*sending)(Computer *const self, Event const * const event);

    /* Hàm này sẽ được gọi để nhận tin nhắn mới từ Matlab */
    void (*public)(QueueHandle_t xQueue, const void * pvItemToQueue);
    void (*publicFromISR)(QueueHandle_t xQueue, const void * pvItemToQueue, BaseType_t *pxHigherPriorityTaskWoken);
};

#endif /* __COMPUTOR_H */
