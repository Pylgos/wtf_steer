#ifndef STEER_ANGLE_CONTROLLER_HPP
#define STEER_ANGLE_CONTROLLER_HPP
#include <anglelib.hpp>
#include <pid_controller.hpp>

class SteerAngleController {
  using Angle = anglelib::Anglef;
  using Direction = anglelib::Directionf;

public:
  SteerAngleController(PidGain gain, Angle home_angle = Angle::zero): pid_{gain}, home_angle_{home_angle} {}

  void update(Angle present, std::chrono::nanoseconds dt) {
    Angle target_angle = present.closest_angle_of(target_dir_);
    pid_.set_target(target_angle.rad());
    pid_.update(present.rad(), dt);
  }

  void set_tgt_direction(Direction dir) {
    target_dir_ = dir;
  }

  void unwind(Angle present, std::chrono::nanoseconds dt) {
    pid_.set_target(home_angle_.rad());
    pid_.update(present.rad(), dt);
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
};

#endif
