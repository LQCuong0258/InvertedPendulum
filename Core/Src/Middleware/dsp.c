#include "dsp.h"

extern float32_t MotorCoefficients[MOTOR_FILTER_STAGES_NUM * COEFFICIENT_NUMBER];
extern float32_t PendulumCoefficients[MOTOR_FILTER_STAGES_NUM * COEFFICIENT_NUMBER];

static void Differentiator_apply(Differentiator * const self, float32_t * input, float32_t * output)
{
    float32_t diff_x;
    float32_t filtered_diff_x;
    float32_t x = *input;

    /*Estimate first derivative*/
    diff_x = self->Cen_Diff_Coeffs[0] * x + self->Cen_Diff_Coeffs[1] * self->state[0] + self->Cen_Diff_Coeffs[2] * self->state[1];

    /*Store the updated state variables back into the state array*/
    self->state[1] = self->state[0]; self->state[0] = x;
    
    /*Filter estimated first derivative*/
    self->filter.numStages = 1;
    arm_biquad_cascade_df2T_f32(&self->filter, &diff_x, &filtered_diff_x, 1);

    /*Return output value*/
    *output = filtered_diff_x;
}

/**
 * 
 */
void Differentiator_new(Differentiator * const self, float32_t SampleTime, uint8_t num_states, float32_t * filter_coeffs)
{
    /*Assign member functions*/
    self->apply = &Differentiator_apply;

    /*Initialize data members*/
    self->SampleTime = SampleTime;
    self->Cen_Diff_Coeffs[0] = 3.0f / (2.0f * self->SampleTime);
    self->Cen_Diff_Coeffs[1] = -4.0f / (2.0f * self->SampleTime);
    self->Cen_Diff_Coeffs[2] = 1.0f / (2.0f * self->SampleTime);

    // self->state[0] = 0.0F; 
    // self->state[1] = 0.0F;

    /*Initialize filter*/
    self->filter_state = ( float32_t * ) pvPortMalloc( (2U * (uint32_t) num_states) * sizeof(float32_t) ); 

    arm_biquad_cascade_df2T_init_f32(&self->filter, num_states, filter_coeffs, self->filter_state);	
}


/* Các hàm dùng để ánh xạ vào các Methods của DSP struct----------*/

/**
 * Hàm này dùng để tính toán số pulse nhận được từ Encoder sang radian
 * Biểu diễn bằng thứ nguyên:
 * angle (radian) = resolution (radian/pulse) * counter (pulse)
 */
static void ConvertAngle (DSP * const self, Encoder const * const encoder_topic) {
    self->raw_motor_angle = self->motor_resolution * encoder_topic->MotorCnt;
    self->raw_pendulum_angle = self->pendulum_resolution * encoder_topic->PendulumCnt;
}

/**
 * Hàm này để lọc data (Low pass filter) nhưng vì hiện tại chưa xử lý được việc
 * lọc data được nên ở đây sẽ gán trực tiếp các giá trị sau khi tính toán được trạng thái
 * của hệ thống vào các biến lưu trạng thái của hệ thống (State)
 */
void filter (DSP * const self) {

    /* Data về vị trí after filted sẽ được lưu vào biến filted_.._angle */
    // Vì vị trí chưa lọc nên sẽ gán trực tiếp giá trị thô vào
    self->filted_motor_angle = self->raw_motor_angle;
    self->filted_pendulum_angle = self->raw_pendulum_angle;


    /**
     * Estimate first derivative and filter
     * Đạo hàm vị trí sang vận tốc sau đó dùng filter
     */
    self->motor_differentiator->apply(
        self->motor_differentiator,
        &self->filted_motor_angle,
        &self->filted_motor_velocity
    );

    self->pendulum_differentiator->apply(
        self->pendulum_differentiator,
        &self->filted_pendulum_angle,
        &self->filted_pendulum_velocity
    );



    // self->motor_velocity = (self->motor_angle - self->motor_state.Prev_position) / self->SampleTime;
    // /* Lưu giá trị hiện tại vào biến trước để tính toán velocity trong lần gọi tiếp theo */
    // self->motor_state.Prev_position = self->motor_angle;

    // // Tương tự

    // self->pendulum_velocity = (self->pendulum_angle - self->pendulum_state.Prev_position) /self->SampleTime;
    // self->pendulum_state.Prev_position = self->pendulum_angle;
}

/**
 * 
 */
void estimate (DSP * const self) {
    /* Lưu lại các trạng thái của hệ thống, vì đây là những
       thông tin cần thiết trong quá trình điều khiển */
    self->motor_state.position = self->filted_motor_angle;
    self->motor_state.velocity = self->filted_motor_velocity;

    self->pendulum_state.position = self->filted_pendulum_angle;
    self->pendulum_state.velocity = self->filted_pendulum_velocity;

    /* Riêng đối với trạng thái của xe, chúng ta cần tính toán vị trí sang tọa độ
       x(m) và vận tốc v(m/s) trước khi gán vào self->cart_state */
    self->cart_state.position = self->filted_motor_angle * self->gear_ratio;
    self->cart_state.velocity = self->filted_motor_velocity * self->gear_ratio;
}

void procesNewData (DSP * const self, Encoder const * const encoder_topic, State * const state_topic) {
    /* Chuyển đổi từ pulse sang radian */
    self->ConvertAngle(self, encoder_topic);

    /* Tiến hành lọc data và tính toán vận tốc*/
    self->filter(self);

    /* Lưu các giá trị trạng thái vào các thành phần của hệ thống */
    self->estimate(self);

    /* Cuối cùng là ánh xạ các giá trị trạng thái vào struct State sẽ dùng chung trong hệ thống */
    state_topic->Motor      = self->motor_state;
    state_topic->Cart       = self->cart_state;
    state_topic->Pendulum   = self->pendulum_state;
}

void dsp_new (DSP * const self) {
    /**
     * Vì hộp giảm tốc có tỉ số truyền là 1:14, 
     * Với bánh răng nhỏ có 14 teeth và Modun = 1.5 => radius = (14 * 1.5) / 2;
     * Khi đó tọa độ x = theta * radius;
     * Với theta là góc quay của trục động cơ, còn giá trị Encoder chúng ta thu về và tính toán
     * chính là góc quay của roto nên theta để tìm x cần nhân với giảm đi 14 lần vì tỉ số truyền (1:14)
     */
    self->gear_ratio = (1.0f / 14.0f) * ((14.0f * 1.5f * 0.001f) / 2.0f);//( 1.0F / 14.0F ) * 0.011F;

    /**
     * Motor's encoder have 500 Pulse per round
     * Pendulum's encoder have 1000 Pulse per round
     * và chúng ta đang đọc chế độ x4 (x2 trên mỗi channels)
     * như vậy 2 biến resolution này sẽ quy đổi cho chúng ta từ pulse sang radian
     * đơn vị (radian per pulse)
     */
    self->motor_resolution = (2.0f * PI) / (4.0f * 500.0f);
    self->pendulum_resolution = (2.0f * PI) / (4.0f * 1000.0f);

    /* Khởi tạo các giá trị ban đầu */
    self->motor_state       = (StateData){.position = .0f, .velocity = 0.0f, .acceleration = 0.0f, .Prev_position = 0.0f};
    self->cart_state        = (StateData){.position = .0f, .velocity = 0.0f, .acceleration = 0.0f, .Prev_position = 0.0f};
    self->pendulum_state    = (StateData){.position = .0f, .velocity = 0.0f, .acceleration = 0.0f, .Prev_position = 0.0f};

    /*Initialize Differentiators*/
    self->motor_differentiator = ( Differentiator * ) pvPortMalloc( sizeof(Differentiator) );
    self->pendulum_differentiator = ( Differentiator * ) pvPortMalloc( sizeof(Differentiator) );
    Differentiator_new(self->motor_differentiator, 1e-3f, MOTOR_FILTER_STAGES_NUM, &MotorCoefficients[0]);
    Differentiator_new(self->pendulum_differentiator, 1e-3f, PENDULUM_FILTER_STAGES_NUM, &PendulumCoefficients[0]);

    /*----------*/
    self->raw_motor_angle = 0.0f; self->filted_motor_angle = 0.0f; self->motor_angle = 0.0f;
    self->raw_pendulum_angle = 0.0f; self->filted_pendulum_angle = 0.0f; self->pendulum_angle = 0.0f; 

    self->raw_motor_velocity = 0.0f; self->filted_motor_velocity = 0.0f; self->motor_velocity = 0.0f;
    self->raw_pendulum_velocity = 0.0f; self->filted_pendulum_velocity = 0.0f; self->pendulum_velocity = 0.0f;

    /* Assign function pointer */
    self->ConvertAngle  = &ConvertAngle;
    self->filter        = &filter;
    self->estimate      = &estimate;
    self->procesNewData = &procesNewData;
}
