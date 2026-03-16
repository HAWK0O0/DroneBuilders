#pragma once

// PID Controller Class
// فئة متحكم PID
// 中文: PID 控制器类
// Implements Proportional-Integral-Derivative control algorithm
// تنفيذ خوارزمية التحكم التناسبي-التكاملي-المشتق
// 中文: 实现比例-积分-微分控制算法

#include "Control/controller_math.h"
#include "INPUT/drone_types.h"

class PidController {
 public:
  void Configure(const PidTuning& tuning) { tuning_ = tuning; }

  void Reset() {
    integral_ = 0.0f;
    last_output_ = 0.0f;
  }

  float Update(float error, float measured_rate_dps, float dt_seconds) {
    if (dt_seconds <= 0.0f) return 0.0f;

    integral_ += error * dt_seconds;
    integral_ = ControllerMath::Clamp(integral_, -tuning_.integral_limit, tuning_.integral_limit);

    const float output = tuning_.kp * error + tuning_.ki * integral_ -
                         tuning_.kd * measured_rate_dps;
    last_output_ = ControllerMath::Clamp(output, -tuning_.output_limit, tuning_.output_limit);
    return last_output_;
  }

  float last_output() const { return last_output_; }
  float integral() const { return integral_; }

 private:
  PidTuning tuning_{};
  float integral_ = 0.0f;
  float last_output_ = 0.0f;
};
