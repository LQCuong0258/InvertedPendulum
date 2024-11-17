#include "Computer.h"

extern UART_HandleTypeDef huart3;
static char tx_data[30];

extern struct StateEstimator * state_estimator;


uint8_t rx_byte_data;

static Status wait (Computer *const self, Event const * const event) {
    Status status;
    
    switch (event->signal) {
        case ENTRY_SIG:
            HAL_UART_Receive_IT(&huart3, &rx_byte_data, 1);           
            status = HANDLED_STATUS;
            break;

        case STATE_UPDATED_SIG:
            static State state_topic = {.Motor = {0.0f}, .Cart = {0.0f}, .Pendulum = {0.0f}};
            BaseType_t is_success;
            // is_success = xQueuePeek(state_estimator->state_pub, &state_topic, 0);
            
            if(is_success) {
                sprintf(tx_data,
                        "S%0.6f %0.6f %0.6f %0.6f\n",
                        1.0,1.1,1.2,1.3
                        // state_topic.Cart.velocity,
                        // state_topic.Pendulum.position,
                        // state_topic.Pendulum.velocity
                );
                SendBuffer(&huart3, tx_data);

                self->super.handler = (StateHandler) self->sending;
                status = TRAN_STATUS;
            }
            else
            {
                status = HANDLED_STATUS;
            }
        
            break;

        case EXIT_SIG:
            status = HANDLED_STATUS;
            break;
        
        default:
            status = IGNORED_STATUS;
            break;
    }
    return status;
}

static void public(QueueHandle_t xQueue, const void * pvItemToQueue) {

    xQueueOverwrite(xQueue, pvItemToQueue);
}

static void publicFromISR(QueueHandle_t xQueue, const void * pvItemToQueue, BaseType_t *pxHigherPriorityTaskWoken) {
  xQueueOverwriteFromISR(xQueue, pvItemToQueue, pxHigherPriorityTaskWoken);
}
