#ifndef STEER_UNIT_CONTROLLER
#define STEER_UNIT_CONTROLLER
#include <anglelib.hpp>
#include <pid_controller.hpp>
#include <steer_angle_controller.hpp>
#include <vmath.hpp>


class SteerUnitController {
  using Vec2 = vmath::Vec2f;
  using Angle = anglelib::Anglef;
  using Direction = anglelib::Directionf;

public:
  SteerUnitController(PidGain steer_gain, PidGain velocity_gain, float wheel_radius)
    : steer_controller_{steer_gain}, vel_controller_{velocity_gain}, wheel_radius_{wheel_radius}
  {}

  void set_tgt_vel(Vec2 vel) {
    tgt_vel_ = vel;
  }

  void update(float present_drive_ang_vel, Angle present_steer_angle, std::chrono::nanoseconds dt) {
    odom_vel_ = Vec2(present_drive_ang_vel * wheel_radius_, 0).rotated(present_steer_angle);

    if (tgt_vel_.length() < 0.1) {
      steer_controller_.set_tgt_direction(last_tgt_dir_);
      vel_controller_.set_target(0.0);
      steer_controller_.update(present_steer_angle, dt);
      vel_controller_.update(present_drive_ang_vel, dt);
      return;
    }

    Direction tgt_dir = Direction::from_xy(tgt_vel_.x, tgt_vel_.y);
    Direction tgt_backward_dir = tgt_dir + Angle::half_turn;
    Angle tgt_angle = present_steer_angle.closest_angle_of(tgt_dir);
    Angle tgt_backward_angle = present_steer_angle.closest_angle_of(tgt_backward_dir);
    if ((tgt_angle - present_steer_angle).abs() < (tgt_backward_angle - present_steer_angle).abs()) {
      steer_controller_.set_tgt_direction(tgt_dir);
      vel_controller_.set_target(tgt_vel_.length() / wheel_radius_);
      last_tgt_dir_ = tgt_dir;
    } else {
      steer_controller_.set_tgt_direction(tgt_backward_dir);
      vel_controller_.set_target(-tgt_vel_.length() / wheel_radius_);
      last_tgt_dir_ = tgt_backward_dir;
    }

    steer_controller_.update(present_steer_angle, dt);
    vel_controller_.update(present_drive_ang_vel, dt);
  }

  void unwind(float present_drive_ang_vel, Angle present_steer_angle, std::chrono::nanoseconds dt) {
    steer_controller_.unwind(present_steer_angle, dt);
    vel_controller_.set_target(0.0);;
    vel_controller_.update(present_drive_ang_vel, dt);
  }

  float get_drive_output() {
    return vel_controller_.get_output();
  }

  float get_steer_output() {
    return steer_controller_.get_output();
  }

  void set_steer_gain(PidGain gain) {
    steer_controller_.set_gain(gain);
  }

  void set_velocity_gain(PidGain gain) {
    vel_controller_.set_gain(gain);
  }

  void reset() {
    steer_controller_.reset();
    vel_controller_.reset();
  }

  Vec2 get_odom_vel() {
    return odom_vel_;
  }

private:
  SteerAngleController steer_controller_;
  PidController vel_controller_;
  float wheel_radius_;
  Vec2 tgt_vel_;
  Vec2 odom_vel_;
  Direction last_tgt_dir_;
};

#endif
