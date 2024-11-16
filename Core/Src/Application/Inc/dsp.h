/* 
 * Author: Syaoran
 * Created on: 2024-11-15
 */
#ifndef __DSP_H
#define __DSP_H

#include "encoder.h"

/**---------------------------------------------------------------------------------
 * Ở file này còn thiếu một quy trình là lọc dữ liệu (Low pass fillter)
 * nên tạm thời sẽ gán trực tiếp các giá trị tính toán được vào State của hệ thống
 * sau này cần phải lọc dữ liệu để tránh các giá trị không mong muốn
 * giúp thu được các giá trị chính xác hơn và ổn định nhất của hệ thống.
 *///-------------------------------------------------------------------------------

/** 
 * Đây là cách khai báo trước cho struct DSP.
 * Vì methods trong DSP cần sử dụng biến có kiểu struct DSP.
 * nên nếu không khai báo trước thì sẽ báo lỗi (struct DSP chưa được định nghĩa).
 */
typedef struct DSP DSP;

struct DSP {
    /* Member-------------------- */

    /* Giá trị SampleTime */
    float32_t SampleTime;

    /* Hằng số dùng để chuyển đổi gốc quay của motor sang tọa độ của cart */
    float32_t gear_ratio;

    /* Hai thành phần chứa giá trị độ phân giải của Encoder {motor, pendulum} */
    float32_t motor_resolution;
    float32_t pendulum_resolution;

    /* Hai thành phần dùng để chứa giá trị góc của motor và pendulum */
    float32_t motor_angle;
    float32_t pendulum_angle;

    /* Hai thành phần dùng để chứa giá trị vận tốc góc của motor và pendulum */
    float32_t motor_velocity;
    float32_t pendulum_velocity;

    /**
     * Hai thành phần có kiểu StateData được dùng để lưu trữ trạng thái
     * của motor và pendulum bao gồm các thông tin như: vị trí, vận tốc, gia tốc
     */
    StateData motor_state;
    StateData cart_state;
    StateData pendulum_state;


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