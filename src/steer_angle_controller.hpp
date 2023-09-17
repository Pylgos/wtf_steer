#ifndef STEER_ANGLE_CONTROLLER_HPP
#define STEER_ANGLE_CONTROLLER_HPP
#include <anglelib.hpp>
#include <functional>
#include <pid_controller.hpp>

class SteerAngleController {
  using Angle = anglelib::Anglef;
  using Direction = anglelib::Directionf;

 public:
  SteerAngleController(PidGain gain, Angle home_angle = Angle::zero()) : pid_{gain}, home_angle_{home_angle} {}

  void update(Angle present, std::chrono::nanoseconds dt) {
    if(unwinding_ && is_unwound(present)) {
      unwinding_ = false;
      if(on_unwound_) on_unwound_(true);
    }

    Angle target_angle;
    if(unwinding_) {
      target_angle = home_angle_;
    } else {
      target_angle = present.closest_angle_of(target_dir_);
      constexpr float max_omega = 6.0f;  // [rad/sec]
      auto max = Angle{max_omega * std::chrono::duration<float>(dt).count()};
      float now_tgt = pid_.get_target();
      if(std::isnan(now_tgt)) now_tgt = 0;
      target_angle = Angle{now_tgt} + std::clamp(target_angle - Angle{now_tgt}, -max, max);
    }
    pid_.set_target(target_angle.rad());
    pid_.update(present.rad(), dt);
  }

  void set_tgt_direction(Direction dir) {
    target_dir_ = dir;
  }

  void start_unwinding() {
    unwinding_ = true;
  }

  void stop_unwinding() {
    if(unwinding_ && on_unwound_) on_unwound_(false);
    unwinding_ = false;
  }

  void on_unwound(std::function<void(bool)> f) {
    on_unwound_ = f;
  }

  bool is_unwound(Angle present) {
    return (home_angle_ - present).abs() < Angle::from_deg(10);
  }

  bool is_unwinding() {
    return unwinding_;
  }

  float get_output() {
    return pid_.get_output();
  }

  void set_gain(PidGain gain) {
    pid_.set_gain(gain);
  }

  void reset() {
    pid_.reset();
  }

 private:
  PidController pid_;
  Direction target_dir_;
  Angle home_angle_;
  std::function<void(bool)> on_unwound_ = nullptr;
  bool unwinding_ = false;
};

#endif
