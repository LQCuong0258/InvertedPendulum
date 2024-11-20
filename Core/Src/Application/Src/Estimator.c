#include "Estimator.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern struct Computer * computer;
struct Active * AO_Estimator;
struct Estimator * estimator; 


Status init (struct Estimator * const self, Event const * const event) {
    Status status = TRAN_STATUS;

    self->super.handler = (StateHandler) self->wait;

    return status;
}

Status wait (struct Estimator * const self, Event const * const event) {
    Status status;

    switch (event->signal) {
        case INIT_SIG:
            HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
            HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);

            status = HANDLED_STATUS;
            break;

        case TIMEOUT_1kHz_SIG:
            static Encoder encoder_topic = {.MotorCnt = 0, .PendulumCnt = 0};
            static State state_topic = {.Motor = {0.0f}, .Cart = {0.0f}, .Pendulum = {0.0f}};

            /**
             * xQueuePeek() sẽ lấy giá trị từ self->encoder_sub và lưu vào encoder_topic
             * mà không xóa giá trị đó khỏi self->encoder_sub
             */
            BaseType_t is_success = xQueuePeek(self->encoder_sub, &encoder_topic, 0);

            if (is_success) {
                /**
                 * Sau khi thành công lấy được giá trị encoder từ self->encoder_sub
                 * thì tiến tính toán và xử lý lọc data để lấy được các giá trị cần thiết
                 * cho điều khiển [x v theta omega]
                 */
                self->data_processor->procesNewData(self->data_processor, &encoder_topic, &state_topic);
            }

            /* Ghi đè dữ liệu từ state_topic*/
            self->public(self->state_pub, &state_topic);

            /**
             * Post STATE_UPDATED_SIG event into computer queue
             * Điều này sẽ khiến computer nhận được Queue state_pub và
             * gửi state_topic đến Matlab 
             */
            static const Event state_evt = { STATE_UPDATED_SIG };
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

static void new (struct Estimator * const self) {

    /* Assign methods*/
    self->init          = &init;
    self->wait          = &wait;
    self->public        = &public;
    self->publicFromISR = &publicFromISR;

    /* Khởi tạo một Active Object mới kế thừa từ class Avtive Object */
    Active_new(&self->super, (StateHandler) &init);

    /* Cấp phát động cho biến kiểu DSP để lọc và xử lý data */
    self->data_processor = (struct DSP * ) pvPortMalloc( sizeof(struct DSP) );
    dsp_new(self->data_processor);

    AO_Estimator = &self->super;
    /**
     * Gán trực tiếp đối tường self vào biến con trỏ
     * StateEstimator để có thể sử dụng dữ liệu
     * của cấu trúc Estimator ở file khác
     */
    estimator = self;

    self->encoder_sub   = xQueueCreate(1, sizeof(Encoder));
    self->state_pub     = xQueueCreate(1, sizeof(State));
}

const struct EstimatorClass Estimator = { .new = &new };
