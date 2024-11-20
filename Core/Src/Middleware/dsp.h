/* 
 * Author: Syaoran
 * Created on: 2024-11-15
 */
#ifndef __DSP_H
#define __DSP_H

#include "FreeRTOS.h"
#include "arm_math.h"
#include "timer.h"
#include "Butter_Worth_Filtor.h"

/**---------------------------------------------------------------------------------
 * Ở file này còn thiếu một quy trình là lọc dữ liệu (Low pass fillter)
 * nên tạm thời sẽ gán trực tiếp các giá trị tính toán được vào State của hệ thống
 * sau này cần phải lọc dữ liệu để tránh các giá trị không mong muốn
 * giúp thu được các giá trị chính xác hơn và ổn định nhất của hệ thống.
 *///-------------------------------------------------------------------------------

/* Khai báo trước để các Method trong Differentiator struct có thể sử dụng*/
typedef struct Differentiator Differentiator;

/**
 * Struct được dùng như một bộ lọc, khi data được đưa vào sẽ được lọc và tính toán
 * đạo hàm vị trí sang vận tốc 
 */
struct Differentiator {
    /** 
     * Giá trị chu kỳ lấy mẫu SampleTime nghĩa là hệ thống sẽ lấy mẫu
     * (1/SampleTime) lần trong một giây.
     * Ví dụ, nếu SampleTime = 0.01s thì trong 1 giây hệ thống sẽ 
     * lấy được 100 lần dữ liệu.
     */
    float32_t SampleTime;

    /**
     * Central difference coefficients
     * Sử dụng đạo hàm bằng phương pháp sai phân bậc hai
     * đây là phương pháp đạo hàm nhờ các điểm lân cận
     * ứng dụng nhiều trong Machine Vision 
     */
    float32_t Cen_Diff_Coeffs[3];

    /**
     * Biến dùng để lưu giá trị trước đó, bao gồm 2 giá trị là
     * state[1] = thời điểm 1 lần trước đó so với hiện tại.
     */
    float32_t state[2];

    arm_biquad_cascade_df2T_instance_f32 filter;

    float32_t * filter_state;

    void (*apply) (Differentiator * const self, float32_t * input, float32_t * output);
};
void Differentiator_new(Differentiator * const self, float32_t sample_time, uint8_t num_states, float32_t * filter_coeffs);


/** 
 * Đây là cách khai báo trước cho struct DSP.
 * Vì methods trong DSP cần sử dụng biến có kiểu struct DSP.
 * nên nếu không khai báo trước thì sẽ báo lỗi (struct DSP chưa được định nghĩa).
 */
typedef struct DSP DSP;

struct DSP {
    /* Member-------------------- */

    /* Hằng số dùng để chuyển đổi gốc quay của motor sang tọa độ của cart */
    float32_t gear_ratio;

    /* Hai thành phần chứa giá trị độ phân giải của Encoder {motor, pendulum} */
    float32_t motor_resolution;
    float32_t pendulum_resolution;

    /**
     * Hai thành phần dùng để chứa giá trị góc của motor và pendulum
     * + raw là dữ liệu thô
     * + filted là dữ liệu đã được qua bộ lọc
     * + cuối cùng là dữ liệu đầu ra cuối cùng
     */
    /* Dữ liệu thô (raw) khi thu thập từ Encoder và chưa qua xử lý */
    float32_t raw_motor_angle;
    float32_t raw_pendulum_angle;

    float32_t filted_motor_angle;
    float32_t filted_pendulum_angle;

    float32_t motor_angle;
    float32_t pendulum_angle;

    /**
     * Hai thành phần dùng để chứa giá trị vận tốc góc của motor và pendulum
     * + raw là dữ liệu thô
     * + filted là dữ liệu đã được qua bộ lọc
     * + cuối cùng là dữ liệu đầu ra cuối cùng
     */
    /* Dữ liệu thô (raw) khi thu thập từ Encoder và chưa qua xử lý */
    float32_t raw_motor_velocity;
    float32_t raw_pendulum_velocity;

    float32_t filted_motor_velocity;
    float32_t filted_pendulum_velocity;

    float32_t motor_velocity;
    float32_t pendulum_velocity;


    /**
     * Hai thành phần có kiểu StateData được dùng để lưu trữ trạng thái
     * của motor và pendulum bao gồm các thông tin như: vị trí, vận tốc, gia tốc
     */
    StateData motor_state;
    StateData cart_state;
    StateData pendulum_state;


    /**
     * Đây là cấu trúc dùng để lưu các giá trị sau khi đi qua bộ lọc ButterWorth
     * (Low-pass-filter)
     */
    Differentiator * motor_differentiator;
    Differentiator * pendulum_differentiator;


    /* Methods-------------------- */

    /**
     * Phương thức được dùng để tính toán giá trị góc của motor và pendulum
     * từ giá trị Encoder thu được
     */
    void (*ConvertAngle)(DSP * const self, Encoder const * const encoder_topic);

    /**
     * Phương thức được dùng để lọc dữ liệu sau đó
     * tính toán các giá trị vận tốc góc của motor và pendulum
     */
    void (*filter)(DSP * const self);

    /**
     * Phương thức dùng để gán các giá trị tính toán được từ
     * Encoder's counter sang trạng thái của hệ thống
     */
    void (*estimate)(DSP * const self);

    /**
     * Phương thức dùng để update trạng thái mới nhất của hệ thống
     * sau quá trình lọc và xử lý data
     */
    void (*procesNewData)(DSP * const self, Encoder const * const encoder_topic, State * const state_topic);
};

/* Hàm dùng để khởi tạo giá trị của struct DSP */
void dsp_new (DSP * const self);

#endif /* __DSP_H */