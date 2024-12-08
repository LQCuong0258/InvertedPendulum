#include "Controller.h"

extern TIM_HandleTypeDef htim6;

struct Active * AO_Controller;
struct Controller * controller;

Status init (struct Controller * const self, Event const * const event) {
  Status status = TRAN_STATUS;
  self->super.handler = (StateHandler) self->wait;
  return status;
}

Status wait (struct Controller * const self, Event const * const event) {
  Status status;

  switch (event->signal) {
    case ENTRY_SIG:
      HAL_TIM_Base_Start_IT(&htim6);
      status = HANDLED_STATUS;
      break;

    case TIMEOUT_100Hz_SIG:
        static State state_topic = {.Motor = {0.0f}, .Cart = {0.0f}, .Pendulum = {0.0f}};

        BaseType_t is_success = xQueuePeek(self->state_sub, &state_topic, 0);

        if (is_success) {
          /* Controller is running */
        }

        /* Ghi đè dữ liệu từ state_topic*/
        self->public(self->state_pub, &state_topic);

        /**
         * Post STATE_UPDATED_SIG event into computer queue
         * Điều này sẽ khiến computer nhận được Queue state_pub và
         * gửi state_topic đến Matlab 
         */
        static const Event state_evt = {.signal = STATE_UPDATED_SIG};
        AO_Estimator->post(&computer->super, &state_evt);

        status = HANDLED_STATUS;
        break;
    
    default:
        status = IGNORED_STATUS;
        break;
  }
  return status;
}

static void public (QueueHandle_t xQueue, const void * pvItemToQueue) {
    /**
     * Ghi đè pvItemToQueue vào xQueue, thường dùng cho Queue có một phần tử,
     * như vậy sẽ có hai trường hợp xảy ra.
     * + Nếu xQueue hiện tại rỗng, thì pvItemToQueue sẽ được thêm vào xQueue
     * giống như cách hoạt động của xQueueSend()
     * + Nếu xQueue hiện tại không rỗng (tức là có một phần tử đang chờ được 
     * gửi đi), thì pvItemToQueue sẽ được ghi đè lên phần tử đang chờ
     * 
     * Điều này giúp cho xQueue luôn cập nhật dữ liệu mới nhất.
     */
    xQueueOverwrite(xQueue, pvItemToQueue);
}

static void publicFromISR (QueueHandle_t xQueue, const void * pvItemToQueue,
                           BaseType_t * pxHigherPriorityTaskWoken)
{
    /* Cách hoạt động tương tự như public(), nhưng đây là phiên bản dùng cho ISR */
    xQueueOverwriteFromISR(xQueue, pvItemToQueue, pxHigherPriorityTaskWoken);
}

static new (struct Controller * const self) {
  self->init          = &init;
  self->wait          = &wait;
  self->public        = &public;
  self->publicFromISR = &publicFromISR;

  AO_Controller = &self->super;
  controller = self;

  self->state_sub   = xQueueCreate(1, sizeof(State));
  self->signal_pub  = xQueueCreate(1, sizeof(PWC));

}

const struct ControllerClass Controller = { .new = &new };
