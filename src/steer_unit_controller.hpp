#ifndef STEER_UNIT_CONTROLLER
#define STEER_UNIT_CONTROLLER
#include <anglelib.hpp>
#include <pid_controller.hpp>
#include <steer_angle_controller.hpp>
#include <steer_drive_controller.hpp>
#include <vmath.hpp>


class SteerUnitController {
  using Vec2 = vmath::Vec2f;
  using Angle = anglelib::Anglef;
  using Direction = anglelib::Directionf;

 public:
  SteerUnitController(PidGain steer_gain, PidGain drive_gain, float wheel_radius)
      : steer_controller_{steer_gain}, drive_controller_{drive_gain}, wheel_radius_{wheel_radius} {}

  void set_tgt_vel(Vec2 vel) {
    tgt_vel_ = vel;
  }

  void update(float present_drive_ang_vel, Angle present_steer_angle, std::chrono::nanoseconds dt) {
    odom_vel_ = Vec2(present_drive_ang_vel * wheel_radius_, 0).rotated(present_steer_angle);
    bool is_stopping = tgt_vel_.length() < 0.1;

    if(is_stopping) {
      steer_controller_.set_tgt_direction(last_tgt_dir_);
      drive_controller_.set_target(0.0);
      steer_controller_.update(present_steer_angle, dt);
      drive_controller_.update(present_drive_ang_vel, dt);
      return;
    }

    stop_unwinding();

    Direction tgt_dir = Direction::from_xy(tgt_vel_.x, tgt_vel_.y);
    Direction tgt_backward_dir = tgt_dir + Angle::half_turn();
    Angle tgt_angle = present_steer_angle.closest_angle_of(tgt_dir);
    Angle tgt_backward_angle = present_steer_angle.closest_angle_of(tgt_backward_dir);

    Angle velocity_cost = Angle::from_rad(std::min((float)M_PI / 4, odom_vel_.length() * 1.0f));
    Angle forward_cost =
        (tgt_angle - present_steer_angle).abs() + (present_drive_ang_vel < 0 ? velocity_cost : Angle::zero());
    Angle backward_cost =
        (tgt_backward_angle - present_steer_angle).abs() + (present_drive_ang_vel > 0 ? velocity_cost : Angle::zero());

    if(forward_cost < backward_cost) {
      steer_controller_.set_tgt_direction(tgt_dir);
      drive_controller_.set_target(tgt_vel_.length() * ang_error_ / wheel_radius_);
      last_tgt_dir_ = tgt_dir;
    } else {
      steer_controller_.set_tgt_direction(tgt_backward_dir);
      drive_controller_.set_target(-tgt_vel_.length() * ang_error_ / wheel_radius_);
      last_tgt_dir_ = tgt_backward_dir;
    }

    steer_controller_.update(present_steer_angle, dt);
    drive_controller_.update(present_drive_ang_vel, dt);
  }

  Direction calc_tgt_dir(float present_drive_ang_vel, Angle present_steer_angle) const {
    Direction tgt_dir = Direction::from_xy(tgt_vel_.x, tgt_vel_.y);
    Direction tgt_backward_dir = tgt_dir + Angle::half_turn();
    Angle tgt_angle = present_steer_angle.closest_angle_of(tgt_dir);
    Angle tgt_backward_angle = present_steer_angle.closest_angle_of(tgt_backward_dir);

    Angle velocity_cost = Angle::from_rad(std::min((float)M_PI / 4, odom_vel_.length() * 1.0f));
    Angle forward_cost =
        (tgt_angle - present_steer_angle).abs() + (present_drive_ang_vel < 0 ? velocity_cost : Angle::zero());
    Angle backward_cost =
        (tgt_backward_angle - present_steer_angle).abs() + (present_drive_ang_vel > 0 ? velocity_cost : Angle::zero());

    return forward_cost < backward_cost ? tgt_dir : tgt_backward_dir;
  }

  float get_angle_error(float present_drive_ang_vel, Angle present_steer_angle) const {
    Direction tgt_dir = calc_tgt_dir(present_drive_ang_vel, present_steer_angle);
    Angle ang = present_steer_angle - present_steer_angle.closest_angle_of(tgt_dir);
    return std::abs(std::cos(ang.rad()));
  }

  void start_unwinding() {
    steer_controller_.start_unwinding();
    last_tgt_dir_ = Direction::zero();
  }

  void stop_unwinding() {
    steer_controller_.stop_unwinding();
  }

  void on_unwound(std::function<void(bool)> f) {
    steer_controller_.on_unwound(f);
  }

  bool is_unwound(Angle present_steer_angle) {
    return steer_controller_.is_unwound(present_steer_angle);
  }

  bool is_unwinding() {
    return steer_controller_.is_unwinding();
  }

  float get_drive_output() {
    return drive_controller_.get_output();
  }

  float get_steer_output() {
    return steer_controller_.get_output();
  }

  void set_steer_gain(PidGain gain) {
    steer_controller_.set_gain(gain);
  }

  void set_drive_gain(PidGain gain) {
    drive_controller_.set_gain(gain);
  }

  void set_error(const float error) {
    ang_error_ = error;
  }

  void reset() {
    steer_controller_.reset();
    drive_controller_.reset();
  }

  Vec2 get_odom_vel() {
    return odom_vel_;
  }

 private:
  SteerAngleController steer_controller_;
  SteerDriveController drive_controller_;
  float wheel_radius_;
  Vec2 tgt_vel_;
  Vec2 odom_vel_;
  Direction last_tgt_dir_;
  float ang_error_ = 1;
};

#endif
