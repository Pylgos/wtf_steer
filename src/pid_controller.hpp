#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP
#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>

struct PidGain {
  float kp = 0;
  float ki = 0;
  float kd = 0;
  float max = 0;
  float min = 0;
  bool use_velocity_for_d_term = false;
  bool anti_windup = false;
};

class PidController {
 public:
  PidController(PidGain gain) : gain_{gain} {}

  void update(float present, std::chrono::nanoseconds dt) {
    float vel = 0.0;
    if(!std::isnan(prev_)) {
      vel = (present - prev_) / to_sec(dt);
    }
    update_with_vel(present, vel, dt);
  }

  void update_with_vel(float present, float velocity, std::chrono::nanoseconds dt) {
    prev_ = present;
    float dt_sec = to_sec(dt);
    if(std::isnan(target_) || std::isnan(present) || std::isnan(velocity) || dt_sec == 0.0f) {
      return;
    }
    float error = target_ - present;
    float p = error * gain_.kp;
    integral_ += error * dt_sec;
    float i = integral_ * gain_.ki;
    float d = 0.0;
    if(gain_.use_velocity_for_d_term) {
      d = velocity * gain_.kd;
    } else {
      d = ((error - prev_error_) / dt_sec) * gain_.kd;
    }
    prev_error_ = error;
    float output = p + i + d;
    float output_saturated = std::clamp(output, gain_.min, gain_.max);
    if (gain_.anti_windup && gain_.ki != 0) {
      float integral_max = gain_.max / gain_.ki;
      float integral_min = gain_.min / gain_.ki;
      integral_ = std::clamp(integral_, integral_min, integral_max);
      // integral_ -= (output - output_saturated) / gain_.ki;
    }
    output_ = output_saturated;
  }

  void set_target(float target) {
    target_ = target;
  }
  auto get_target() const {
    return target_;
  }

  float get_output() {
    return output_;
  }

  void reset() {
    output_ = 0.0;
    target_ = std::numeric_limits<float>::quiet_NaN();
    prev_ = std::numeric_limits<float>::quiet_NaN();
    integral_ = 0.0;
    prev_error_ = 0.0;
  }

  void set_gain(PidGain gain) {
    gain_ = gain;
    reset();
  }

 private:
  static float to_sec(std::chrono::nanoseconds dur) {
    return dur.count() * 1e-9f;
  }

  PidGain gain_;
  float output_ = 0.0;
  float target_ = std::numeric_limits<float>::quiet_NaN();
  float prev_ = std::numeric_limits<float>::quiet_NaN();
  float integral_ = 0.0;
  float prev_error_ = 0.0;
};

#endif
