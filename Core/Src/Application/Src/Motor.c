#include "Motor.h"

extern UART_HandleTypeDef huart2;
static char tx_data[15];

struct Active * AO_Motor;
struct Motor * motor; 

static Status init (struct Motor * const self, Event const * const event) {
    Status status = TRAN_STATUS;
    self->super.handler = (StateHandler) self->wait;
    return status;
}

static Status wait (struct Motor * const self, Event const * const event) {
    Status status;
    
    switch (event->signal) {
        case ENTRY_SIG:
            status = HANDLED_STATUS;
            break;

        case PWC_TRIGGER_SIG:
            static PWC pwc_topic = {.d =0};
            BaseType_t is_success;
            is_success = xQueuePeek(self->pwc_sub, &pwc_topic, 0);

            if(is_success) {
                sprintf(tx_data, "N1 O d%d\n", pwc_topic.d);
                SendBuffer(&huart2, tx_data);

                self->super.handler = (StateHandler) self->sending;
                status = TRAN_STATUS;
            }
            else {
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

static Status sending (struct Motor * const self, Event const * const event) {
    Status status;
    
    switch (event->signal) {
        case ENTRY_SIG:
            status = HANDLED_STATUS;
            break;

        case COMMAND_SENDED_SIG:
            self->super.handler = (StateHandler)self->wait;
            status = TRAN_STATUS;
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

static void public (QueueHandle_t xQueue, const void * pvItemToQueue) {
    xQueueOverwrite(xQueue, pvItemToQueue);
}

static void publicFromISR (QueueHandle_t xQueue, const void * pvItemToQueue,
                           BaseType_t *pxHigherPriorityTaskWoken)
{
    xQueueOverwriteFromISR(xQueue, pvItemToQueue, pxHigherPriorityTaskWoken);
}

static void new(struct Motor * const self) {
    /* Assign Methods */
    self->init          = &init;
    self->wait          = &wait;
    self->sending       = &sending;
    self->public        = &public;
    self->publicFromISR = &publicFromISR;

    /*Initialize members*/
    Active_new(&self->super, (StateHandler)&init);

    /*Cache Ao for using in Encoder driver*/
    AO_Motor = &self->super;
    motor = self;

    /*Initialize Queue for Mailbox as subsribers, publishers*/
    self->pwc_sub = xQueueCreate( 1, sizeof( PWC ) );
}

const struct MotorClass Motor = { .new = &new };

