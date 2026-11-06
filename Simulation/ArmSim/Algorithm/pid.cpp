#include "pid.h"

namespace {

float Limit(float value, float max)
{
    return value < -max ? -max : (value > max ? max : value);
}

}  // namespace

float Pid::Update(float error)
{
    last_error_ = cur_error_;
    cur_error_ = error;
    sum_error_ = Limit(sum_error_ + cur_error_, gain_.sum_error_max);
    deriv_filtered_ = deriv_filtered_ * gain_.d_alpha +
                      (cur_error_ - last_error_) * (1.0f - gain_.d_alpha);
    output_ = Limit(gain_.kp * cur_error_ + gain_.ki * sum_error_ + gain_.kd * deriv_filtered_,
                    gain_.output_max);
    return output_;
}

void Pid::Reset()
{
    cur_error_ = 0.0f;
    last_error_ = 0.0f;
    sum_error_ = 0.0f;
    deriv_filtered_ = 0.0f;
    output_ = 0.0f;
}
