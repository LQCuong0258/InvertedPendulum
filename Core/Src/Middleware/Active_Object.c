#include "Active_Object.h"

/*-------------------------------------------------------------------------------------------------------*/
/* Event-loop thread function for all Active Object (FreeRTOS task signature) */
static void Active_eventloop (void *pvParameters) {
    
    Active *Active_instance = (Active *) pvParameters;

    /* Khởi tạo vào sự kiện */
    static Event const initEvt = { ENTRY_SIG };

    /* Active object must be provided */
    configASSERT(Active_instance);

    /* Initialize Active Object */
    (*Active_instance->dispatch) (Active_instance, &initEvt);
    for(;;) {
        Event * event;  /* Pointer to Event object (message) */
    
        /* Wait for any event and receive it into object 'event' */
        xQueueReceive(Active_instance->queue, &event, portMAX_DELAY);
        
        /* Dispatch to Active Object 'self' */
        (*Active_instance->dispatch) (Active_instance, event);
    }
}
/*-------------------------------------------------------------------------------------------------------*/

/* Khởi tạo hệ thống Active Object */
static void init(Active * const self, Event const * const event) {
    /* Sự kiện khởi tạo một Active Object */
    static Event const entry_evt = {.signal = ENTRY_SIG};

    (*self->handler)(self, event);
    (*self->handler)(self, &entry_evt);
}

static void start (Active * const self, UBaseType_t uxPriority, UBaseType_t uxQueueLength, uint32_t usStackDepth) {
    self->queue = xQueueCreate(uxQueueLength, sizeof(Event *));

    xTaskCreate(
        self->EventLoop,
        "ActiveObject",
        usStackDepth,
        self,
        uxPriority,
        &self->thread
    );

    /* Thread must be created */
    configASSERT(self->thread);
}

/* Hàm phân phối các sự kiện cho Active Object */
static void dispatch(struct Active * const self, Event const * const event)
{
    Status status;
    /* Lưu lại quá trình xử lý hiện tại */
    StateHandler prev_handler = self->handler;
    static Event const entry_evt = {.signal = ENTRY_SIG};
    static Event const exit_evt = {.signal = EXIT_SIG};

    /* Thực hiện xử lý sự kiện mới được nhận */
    status = (*self->handler)(self, event);
    /* Nếu sự kiện là đổi trạng thái thì sẽ thoát khỏi Active Object hiện tại */
    if(status == TRAN_STATUS) {
        (*prev_handler)(self, &exit_evt);
        (*self->handler)(self, &entry_evt);
    }
}

static void post (Active * const self, Event const * const event) {
    BaseType_t status = xQueueSendToBack(self->queue, (void *)&event, (TickType_t) 0);
    configASSERT(status == pdPASS);
}

static void postFromISR (Active * const self, Event const * const event,
                         BaseType_t * const pxHigherPriorityTaskWoken)
{
    BaseType_t status = xQueueSendToBackFromISR(self->queue, (void *)&event, pxHigherPriorityTaskWoken);
    configASSERT(status == pdPASS);
}

void Active_new (Active * const self, StateHandler handler) {
    self->handler = handler;

    self->EventLoop = &Active_eventloop;
    self->init = &init;
    self->start = &start;
    self->dispatch = &dispatch;
    self->post = &post;
    self->postFromISR = &postFromISR;
}