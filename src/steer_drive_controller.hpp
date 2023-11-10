#ifndef STEER_DRIVE_CONTROLLER
#define STEER_DRIVE_CONTROLLER

#include <anglelib.hpp>
#include <pid_controller.hpp>
#include <vmath.hpp>

class SteerDriveController {
  using Vec2 = vmath::Vec2f;
  using Angle = anglelib::Anglef;
  using Direction = anglelib::Directionf;

 public:
  SteerDriveController(PidGain gain) : pid_{gain} {}
  void set_target(const float target_drive_ang_vel) {
    target_drive_ang_vel_ = target_drive_ang_vel;
  }
  void update(const float present_drive_ang_vel, const std::chrono::nanoseconds dt) {
    const float pre_tgt = std::isnan(pid_.get_target()) ? present_drive_ang_vel : pid_.get_target();
    constexpr float max_accel = 50.0;  // [rad/s^2]
    const float max_dvel = max_accel * std::chrono::duration<float>{dt}.count();
    const float clamped_tgt = pre_tgt + std::clamp(target_drive_ang_vel_ - pre_tgt, -max_dvel, max_dvel);
    pid_.set_target(clamped_tgt);
    pid_.update(present_drive_ang_vel, dt);
  }
  float get_output() {
    return pid_.get_output();
  }
  void set_gain(const PidGain gain) {
    pid_.set_gain(gain);
  }
  void reset() {
    pid_.reset();
  }
 private:
  PidController pid_;
  float target_drive_ang_vel_;
};
#endif  // STEER_DRIVE_CONTROLLER
