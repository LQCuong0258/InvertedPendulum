#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "stm32f4xx_hal.h"
#include "Active_Object.h"

typedef enum {
    /* Sự kiện được kích hoạt khi Timer 6 Trigger*/
    TIMEOUT_100Hz_SIG = USER_SIG
} ControllerEvent;

struct Controller {
    /* Members---------- */
    struct Active super;   /* Kế thừa lớp Active-Object */
    /* Nhận về từ các máy trạng thái khác (Subscribe) */
    QueueHandle_t state_sub;
    /* Gửi đi cho các máy trạng thái khác (publish) */
    QueueHandle_t signal_pub;


    /* Methods---------- */
    
    /**
     * Các phương thức này sẽ trả về trạng thái của Active Object
     * sau khi nhận được tín hiệu Event và xử lý nó
     */
    Status (*init)(struct Controller * const self, Event const * const event);
    Status (*wait) (struct Controller * const self, Event const * const event);
    /*------------------*/
    void (*public) (QueueHandle_t xQueue, const void * pvItemToQueue);
    void (*publicFromISR) (QueueHandle_t xQueue, const void * pvItemToQueue,
                           BaseType_t * pxHigherPriorityTaskWoken);
};

extern const struct ControllerClass {
    void (*new) (struct Controller * self);
} Controller;



#endif /* CONTROLLER_H */ 