/* 
 * Author: Syaoran
 * Created on: 2024-11-17
 */
#ifndef __ACTIVE_OBJECT_H
#define __ACTIVE_OBJECT_H

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "Topic.h"

/* Forward declaration of the ActiveObject class */
typedef struct Active Active;

/* Event */
typedef struct {
    uint16_t signal;
} Event;

typedef enum {
    INIT_SIG,
    ENTRY_SIG,
    EXIT_SIG,
    USER_SIG
} ReservedSignal;

typedef enum {
    TRAN_STATUS,
    HANDLED_STATUS,
    IGNORED_STATUS,
    INIT_STATUS
} Status;


typedef Status (*StateHandler) (Active * const self, Event const * const event);


/* Active object */
struct Active {
    /* Members---------- */
    StateHandler handler;
    TaskHandle_t thread;
    StaticTask_t thread_cb;     /* Thread control-block (FreeRTOS static alloc) */
    QueueHandle_t queue;
    StaticQueue_t queue_cb;     /* Queue control-block (FreeRTOS static alloc) */

    /* Methods---------- */
    void (*init)(Active * const self, Event const * const event);
    void (*dispatch) (struct Active * const self, Event const * const event);
    void (*start) (Active * const self, UBaseType_t uxPriority, UBaseType_t uxQueueLength, uint32_t usStackDepth);
    void (*EventLoop) (void *pvParameters);
    void (*post) (Active * const self, Event const * const event);
    void (*postFromISR) (Active * const self, Event const * const event,
                         BaseType_t *pxHigherPriorityTaskWoken);

};

void Active_new (struct Active * const self, StateHandler handler);

#endif /* __ACTIVE_OBJECT_H */