#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP
#include "protocol.hpp"
#include "pid_controller.hpp"
#include "anglelib.hpp"
#include <vmath.hpp>
#include <mbed.h>
#include <functional>

class Controller {
  using Vec2 = vmath::Vec2f;
  using Angle = anglelib::Anglef;
public:
  using State = Feedback::CurrentState::State;

  Controller(std::function<void(const CANMessage&)> can_write_impl, uint16_t can_id = Command::ID)
      : can_id_{can_id}, can_write_impl_{can_write_impl} {}

  bool parse_packet(const CANMessage& msg, std::chrono::microseconds now) {
    if (msg.id != can_id_) return false;
    if (msg.len != 8) return true;
    if (msg.format != CANStandard) return true;
    
    last_cmd_received_ = now;

    Command cmd;
    memcpy(&cmd, msg.data, sizeof(cmd));

    switch (cmd.tag) {
      case Command::Tag::SET_TARGET_VELOCITY: {
        tgt_linear_vel_ = Vec2{cmd.set_target_velocity.vx * 1e-3f, cmd.set_target_velocity.vy * 1e-3f};
        tgt_ang_vel_ = cmd.set_target_velocity.ang_vel * 1e-3f;
      } break;

      case Command::Tag::SET_PARAM: {
        printf("set param\n");
        auto arg = cmd.set_param;
        switch (arg.id) {
          case ParamId::DRIVE_KP: asgn(drive_gain_.kp, arg.value); break;
          case ParamId::DRIVE_KI: asgn(drive_gain_.ki, arg.value); break;
          case ParamId::DRIVE_KD: asgn(drive_gain_.kd, arg.value); break;
          case ParamId::DRIVE_MAX: asgn(drive_gain_.max, arg.value); break;
          case ParamId::DRIVE_MIN: asgn(drive_gain_.min, arg.value); break;
          case ParamId::DRIVE_ANTIWINDUP: asgn(drive_gain_.anti_windup, arg.value); break;
          case ParamId::DRIVE_USE_VELOCITY_FOR_D_TERM: asgn(drive_gain_.use_velocity_for_d_term, arg.value); break;
          case ParamId::DRIVE_PID_MAX: break;

          case ParamId::STEER_KP: asgn(steer_gain_.kp, arg.value); break;
          case ParamId::STEER_KI: asgn(steer_gain_.ki, arg.value); break;
          case ParamId::STEER_KD: asgn(steer_gain_.kd, arg.value); break;
          case ParamId::STEER_MAX: asgn(steer_gain_.max, arg.value); break;
          case ParamId::STEER_MIN: asgn(steer_gain_.min, arg.value); break;
          case ParamId::STEER_ANTIWINDUP: asgn(steer_gain_.anti_windup, arg.value); break;
          case ParamId::STEER_USE_VELOCITY_FOR_D_TERM: asgn(steer_gain_.use_velocity_for_d_term, arg.value); break;
          case ParamId::STEER_PID_MAX: break;

          case ParamId::STEER0_OFFSET: call(on_steer_offset_, 0, Angle::from_rad(arg.value.float_value)); break;
          case ParamId::STEER1_OFFSET: call(on_steer_offset_, 1, Angle::from_rad(arg.value.float_value)); break;
          case ParamId::STEER2_OFFSET: call(on_steer_offset_, 2, Angle::from_rad(arg.value.float_value)); break;
          case ParamId::STEER3_OFFSET: call(on_steer_offset_, 3, Angle::from_rad(arg.value.float_value)); break;
        }

        Feedback fb;
        fb.tag = Feedback::Tag::PARAM_EVENT;
        fb.param_event.id = arg.id;
        fb.param_event.value = arg.value;
        CANMessage resp;
        resp.id = Feedback::ID;
        resp.type = CANData;
        resp.format = CANStandard;
        resp.len = 8;
        memcpy(resp.data, &fb, sizeof(fb));
        
        can_write_impl_(resp);
      } break;

      case Command::Tag::RESET_PID: call(on_reset_pid_); break;
      case Command::Tag::ACTIVATE: activate(); break;
    }

    return true;
  }

  void update(chrono::microseconds now) {
    if (is_timeout(now)) {
      deactivate();
    }

    if (now - last_state_publish_ > std::chrono::milliseconds(100)) {
      publish_state();
      last_state_publish_ = now;
    }

    publish_odom();
  }

  void activate() {
    if (state_ != State::RUNNING) {
      state_ = State::RUNNING;
      call(on_reset_pid_);
      call(on_activation_);
    }
  }

  void deactivate() {
    if (state_ != State::CONFIGURING) {
      tgt_linear_vel_ = {0.0, 0.0};
      tgt_ang_vel_ = 0.0;
      state_ = State::CONFIGURING;
      call(on_deactivation_);
    }
  }

  Vec2 get_tgt_linear_vel() { return tgt_linear_vel_; }
  float get_tgt_ang_vel() { return tgt_ang_vel_; }

  void on_reset_pid(std::function<void()> f) {
    on_reset_pid_ = f;
  }

  void on_activation(std::function<void()> f) {
    on_activation_ = f;
  }

  void on_deactivation(std::function<void()> f) {
    on_deactivation_ = f;
  }

  void on_steer_offset(std::function<void(int, anglelib::Anglef)> f) {
    on_steer_offset_ = f;
  }

  bool is_timeout(std::chrono::microseconds now) {
    return now - last_cmd_received_ > std::chrono::milliseconds(100);
  }

  State get_state() {
    return state_;
  }

  void set_odom(Vec2 linear, float angular) {
    odom_linear_vel_ = linear;
    odom_ang_vel_ = angular;
  }

  PidGain get_steer_gain() { return steer_gain_; }
  PidGain get_drive_gain() { return drive_gain_; }

private:
  uint16_t can_id_;
  std::function<void(const CANMessage&)> can_write_impl_;
  std::chrono::microseconds last_cmd_received_{0};
  std::chrono::microseconds last_state_publish_{0};

  Feedback::CurrentState::State state_ = Feedback::CurrentState::CONFIGURING;
  Vec2 tgt_linear_vel_ = {0.0, 0.0};
  float tgt_ang_vel_ = 0.0;
  PidGain steer_gain_{};
  PidGain drive_gain_{};
  Vec2 odom_linear_vel_{0, 0};
  float odom_ang_vel_ = 0;

  void asgn(float& dst, const ParamValue& val) {
    if (val.type == ParamType::FLOAT) dst = val.float_value;
  }

  void asgn(bool& dst, const ParamValue& val) {
    if (val.type == ParamType::INT) dst = (bool)val.int_value;
  }

  template<class Func, class ...Args>
  void call(Func f, Args... args) {
    if (f != nullptr)
      f(std::forward<Args>(args)...);
  }

  void publish_state() {
    CANMessage msg;
    msg.id = Feedback::ID;
    msg.len = 8;
    msg.format = CANStandard;
    msg.type = CANData;
    Feedback fb;
    fb.tag = Feedback::Tag::CURRENT_STATE;
    fb.current_state.state = get_state();
    memcpy(msg.data, &fb, sizeof(fb));
    can_write_impl_(msg);
  }

  void publish_odom() {
    CANMessage msg;
    msg.id = Feedback::ID;
    msg.len = 8;
    msg.format = CANStandard;
    msg.type = CANData;
    Feedback fb;
    fb.tag = Feedback::Tag::ODOMETRY;
    fb.odometry.vx = odom_linear_vel_.x * 1000;
    fb.odometry.vy = odom_linear_vel_.y * 1000;
    fb.odometry.ang_vel = odom_ang_vel_ * 1000;
    memcpy(msg.data, &fb, sizeof(fb));
    can_write_impl_(msg);
  }

  std::function<void()> on_reset_pid_ = nullptr;
  std::function<void()> on_activation_ = nullptr;
  std::function<void()> on_deactivation_ = nullptr;
  std::function<void(int idx, anglelib::Anglef offset)> on_steer_offset_ = nullptr;
};

#endif
