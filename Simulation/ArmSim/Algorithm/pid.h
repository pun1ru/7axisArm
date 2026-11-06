#pragma once

/* 位置式 PID，带 D 项低通和积分、输出限幅。与实机 LibSE/Algorithm/PID 的数值行为一致。
 * 误差和输出的单位由调用方约定（这里是 deg -> N*m）。 */
class Pid {
public:
    struct Gain {
        float kp = 0.0f;
        float ki = 0.0f;
        float kd = 0.0f;
        float d_alpha = 0.0f;        // D 项低通系数 [0, 1)，0 = 不滤波
        float sum_error_max = 0.0f;  // 积分限幅
        float output_max = 0.0f;     // 输出限幅
    };

    Pid() = default;
    explicit Pid(const Gain &gain) : gain_(gain) {}

    float Update(float error);
    void Reset();

    float output() const { return output_; }

private:
    Gain gain_;
    float cur_error_ = 0.0f;
    float last_error_ = 0.0f;
    float sum_error_ = 0.0f;
    float deriv_filtered_ = 0.0f;
    float output_ = 0.0f;
};
