#ifndef STEER_4W_CONTROLLER_HPP
#define STEER_4W_CONTROLLER_HPP
#include <array>
#include <steer_unit_controller.hpp>
#include <vmath.hpp>

class Steer4WController {
  using Vec2 = vmath::Vec2f;
  using Vec3 = vmath::Vec3f;
  using Angle = anglelib::Anglef;
 public:
  Steer4WController(PidGain steer_gain, PidGain drive_gain, float wheel_radius, Vec2 displacement)
      : units_{{SteerUnitController{steer_gain, drive_gain, wheel_radius},
                SteerUnitController{steer_gain, drive_gain, wheel_radius},
                SteerUnitController{steer_gain, drive_gain, wheel_radius},
                SteerUnitController{steer_gain, drive_gain, wheel_radius}}} {
    displacements_[0] = Vec2{displacement.x, displacement.y};
    displacements_[1] = Vec2{-displacement.x, displacement.y};
    displacements_[2] = Vec2{-displacement.x, -displacement.y};
    displacements_[3] = Vec2{displacement.x, -displacement.y};
  }

  void update(std::array<float, 4> present_drive_ang_vels, std::array<Angle, 4> present_steer_angles,
              std::chrono::nanoseconds dt) {
    float errors = 1.0f;
    for(size_t i = 0; i < units_.size(); i++) {
      errors *= units_[i].get_angle_error(present_drive_ang_vels[i], present_steer_angles[i] - offset_[i]);
    }
    for(size_t i = 0; i < units_.size(); i++) {
      units_[i].set_error(std::sqrt(errors));
      units_[i].update(present_drive_ang_vels[i], present_steer_angles[i] - offset_[i], dt);
    }

    float ang_vel = 0;
    Vec2 linear_vel{0, 0};
    const float multiplier = 1.0f / units_.size();
    const Vec3 rot_90 = Vec3(0, 0, 1);
    for(size_t i = 0; i < units_.size(); i++) {
      Vec2 v = units_[i].get_odom_vel();
      Vec2 n = displacements_[i].normalized();
      Vec3 t = rot_90.cross(Vec3(n.x, n.y, 0));
      ang_vel += Vec2(t.x, t.y).dot(v) * multiplier / displacements_[i].length();
      linear_vel += v * multiplier;
    }
    odom_ang_vel_ = ang_vel;
    odom_linear_vel_ = linear_vel;
    odom_ang_pose_ += ang_vel * std::chrono::duration<float>(dt).count();
    odom_linear_pose_ += linear_vel.rotated(anglelib::Angle{odom_ang_pose_}) * std::chrono::duration<float>(dt).count();
  }

  void start_unwinding() {
    for(auto& unit: units_) {
      unit.on_unwound(std::bind(&Steer4WController::on_unit_unwound, this, std::placeholders::_1));
      unit.start_unwinding();
    }
  }

  void stop_unwinding() {
    for(auto& unit: units_) {
      unit.stop_unwinding();
    }
  }

  void on_unwound(std::function<void(bool)> f) {
    on_unwound_ = f;
  }

  void set_tgt_vel(Vec2 linear, float angular) {
    Vec3 ang_vec(0, 0, angular);
    for(size_t i = 0; i < units_.size(); i++) {
      Vec2 vel;
      Vec3 v = ang_vec.cross(Vec3(displacements_[i].x, displacements_[i].y, 0));
      vel += linear;
      vel += Vec2(v.x, v.y);
      units_[i].set_tgt_vel(vel);
    }
  }

  std::array<float, 4> get_drive_outputs() {
    std::array<float, 4> result;
    for(size_t i = 0; i < 4; i++) {
      result[i] = units_[i].get_drive_output();
    }
    return result;
  }

  std::array<float, 4> get_steer_outputs() {
    std::array<float, 4> result;
    for(size_t i = 0; i < 4; i++) {
      result[i] = units_[i].get_steer_output();
    }
    return result;
  }

  void set_steer_gain(PidGain gain) {
    for(auto&& u: units_) {
      u.set_steer_gain(gain);
    }
  }

  void set_drive_gain(PidGain gain) {
    for(auto&& u: units_) {
      u.set_drive_gain(gain);
    }
  }

  void set_steer_offset(int idx, Angle offset) {
    offset_[idx] = offset;
  }

  void reset() {
    for(auto&& u: units_) {
      u.reset();
    }
  }

  Vec2 get_odom_linear_vel() {
    return odom_linear_vel_;
  }

  float get_odom_ang_vel() {
    return odom_ang_vel_;
  }

  Vec2 get_odom_linear_pose() {
    return odom_linear_pose_;
  }

  float get_odom_ang_pose() {
    return odom_ang_pose_;
  }

 private:
  void on_unit_unwound(bool success) {
    if(success) {
      unwound_success_count_++;
      if(unwound_success_count_ != units_.size()) {
        return;
      }
      if(on_unwound_) on_unwound_(true);
    } else {
      if(on_unwound_) on_unwound_(false);
    }
    unwound_success_count_ = 0;
    for(auto& unit: units_) {
      unit.on_unwound(nullptr);
    }
  }

  std::array<SteerUnitController, 4> units_;
  std::array<Angle, 4> offset_ = {};
  std::array<Vec2, 4> displacements_;

  Vec2 odom_linear_vel_{0, 0};
  float odom_ang_vel_ = 0;
  Vec2 odom_linear_pose_{0, 0};
  float odom_ang_pose_ = 0;

  std::function<void(bool)> on_unwound_ = nullptr;
  size_t unwound_success_count_ = 0;
};


#endif
