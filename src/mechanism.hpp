#ifndef MECHANISM_HPP
#define MECHANISM_HPP

#include <mbed.h>

#include <AwaitInterval.hpp>
#include <c620.hpp>
#include <first_penguin.hpp>
#include <optional>
#include <pid_controller.hpp>
#include <servo.hpp>

namespace {
constexpr float deg_to_rad(const int degree) {
  return degree * 2 * M_PI / 360;
}
constexpr float rad_to_deg(const float radian) {
  return radian * 360 / (2 * M_PI);
}
}  // namespace

struct Mechanism {
  void set_arm_angle_gain(PidGain gain) {
    arm_angle.pid.set_gain(gain);
  }
  void set_arm_length_gain(PidGain gain) {
    arm_length.pid.set_gain(gain);
  }
  void set_expander_gain(PidGain gain) {
    expander.pid.set_gain(gain);
  }

  struct Donfan {
    void task() {
      bool lim[2] = {!lim_fwd->read(), !lim_rev->read()};
      printf("d:%d ", lim[1] << 1 | lim[0]);
      if(dir_ == 1 && !lim[0] && !timeout(2s)) {
        fp->set_raw_duty(-8000);
      } else if(dir_ == -1 && !lim[1] && !timeout(2s)) {
        fp->set_raw_duty(8000);
      } else {
        fp->set_raw_duty(0);
      }
    }
    void set_dir(uint8_t dir) {
      dir_ = dir;
      timeout.reset();
    }
    FirstPenguin* fp;
    DigitalIn* lim_fwd;
    DigitalIn* lim_rev;
    int8_t dir_ = 0;
    AwaitInterval<> timeout;
  };
  struct Expander {
    static constexpr int enc_interval = -103000;
    void task() {
      if(state == Waiting && !lim->read()) {
        // 原点合わせ
        set_origin();
        set_lock(false);
        fp->set_raw_duty(0);
        enter_running();
      } else if(state == Waiting && !std::isnan(target)) {
        // キャリブレーション
        if(!calibrate_timeout.await(1500ms)) {
          printf("exp:calibrate ");
          wait_lock_and(false, [&] {
            fp->set_raw_duty(10000);
          });
        } else {
          printf("exp:calibrate stop ");
          enter_running();
        }
      } else if(state == Running) {
        if(!lim->read()) set_origin();
        float present_length = 1.0f / enc_interval * (enc->get_enc() - origin);
        const auto dt = dt_timer();
        const float previous_tgt = std::isnan(pid.get_target()) ? present_length : pid.get_target();
        constexpr float max_vel = 1.0;  // [m/s]
        const float max_dis = max_vel * std::chrono::duration<float>{dt}.count();
        const float new_tgt = previous_tgt + std::clamp(target - previous_tgt, -max_dis, max_dis);
        // 目標値が現在位置より上ならlock
        const bool is_lock = target - present_length > 0;
        wait_lock_and(is_lock, [&] {
          pid.set_target(new_tgt);
          pid.update(present_length, dt);
          constexpr float anti_gravity = 0.03;
          fp->set_duty(-anti_gravity - pid.get_output());
        });
        printf("exp:");
        printf("%1d ", is_lock << 1 | !lim->read());
        printf("% 6ld ", enc->get_enc() - origin);
        printf("% 5.2f ", present_length);
        printf("% 5.2f ", pid.get_target());
        printf("% 6d ", fp->get_raw_duty());
      }
    }
    void set_lock(bool is_lock) {
      servo->set_deg(is_lock ? 90 : 0);
    }
    void set_target(int16_t height) {
      if(height >= 0) {
        if(target > height / 900.0f) lock_wait.stop();
        target = height / 900.0f;
      } else {
        // キャリブレーション
        lock_wait.stop();
        target = 0;
        state = Waiting;
      }
    }
    void enter_running() {
      calibrate_timeout.stop();
      lock_wait.stop();
      origin = enc->get_enc();
      state = Running;
      pid.reset();
      dt_timer.reset();
    }
    void set_origin() {
      origin = enc->get_enc();
    }
    template<class F>
    void wait_lock_and(bool lock, F f) {
      set_lock(lock);
      if(lock_wait.elapsed(500ms)) {
        f();
      }
    }
    FirstPenguin* fp;
    const FirstPenguin* enc;
    DigitalIn* lim;
    Servo* servo;
    enum {
      Waiting,
      Running,
    } state = Waiting;
    float target = NAN;
    PidController pid = {PidGain{}};
    AwaitInterval<> dt_timer{};
    AwaitInterval<> calibrate_timeout{std::nullopt};
    AwaitInterval<> lock_wait{std::nullopt};
    int32_t origin = 0;
  };
  struct Collector {
    void task() {
      if(state != Storing && collecting) {
        state = Running;
      } else if(state == Stop || !lim->read()) {
        state = Stop;
      } else {
        state = Storing;
        collecting = false;
      }
      printf("c%d ", state);
      switch(state) {
        case Stop: {
          servo->set_deg(45);
          fp->set_raw_duty(0);
          break;
        }
        case Storing:
        case Running: {
          servo->set_deg(80);
          fp->set_raw_duty(8000);
          break;
        }
      }
    }
    FirstPenguin* fp;
    DigitalIn* lim;
    Servo* servo;
    enum {
      Stop,
      Running,
      Storing,
    } state = Stop;
    bool collecting = false;
  };
  struct ArmAngle {
    static constexpr int top_deg = 120;
    static constexpr int bottom_deg = -35;
    static constexpr float bottom_rad = deg_to_rad(bottom_deg);
    static constexpr int enc_interval = 4000;
    static constexpr int deg2enc = enc_interval / (top_deg - bottom_deg);
    static constexpr float enc_to_rad = deg_to_rad(top_deg - bottom_deg) / enc_interval;
    static constexpr float enc_to_mrad = enc_to_rad * 1000;
    void task() {
      if(state == Waiting && !lim->read()) {
        // 原点セット
        c620->set_raw_tgt_current(0);
        enter_running();
      } else if(state == Waiting && !std::isnan(target_angle)) {
        if(!calibrate_timeout.await(3s)) {
          // キャリブレーション
          printf("ang:calibrate ");
          c620->set_raw_tgt_current(-2000);
        } else {
          // 3s リミット踏めなかったらそこを原点にする
          printf("ang:stop calibrate ");
          enter_running();
        }
      } else if(state == Running) {
        if(!lim->read()) origin = enc->get_enc();
        const float present_rad = (enc->get_enc() - origin) * enc_to_rad + bottom_rad;
        const auto dt = dt_timer();
        constexpr float max_omega = 1.5f;  // [rad/sec]
        const float max = max_omega * chrono::duration<float>{dt}.count();
        const float pre_tgt = std::isnan(pid.get_target()) ? present_rad : pid.get_target();
        float new_tag_angle = pre_tgt + std::clamp(target_angle - pre_tgt, -max, max);
        constexpr float max_distance = M_PI / 4;
        new_tag_angle = present_rad + std::clamp(new_tag_angle - present_rad, -max_distance, max_distance);
        pid.set_target(new_tag_angle);
        pid.update(present_rad, dt);
        float anti_gravity = 1500 * std::cos(present_rad);
        c620->set_raw_tgt_current(std::clamp(16384 * pid.get_output() + anti_gravity, -16384.0f, 16384.0f));
        printf("ang:");
        printf("%1d ", !lim->read());
        printf("%6ld ", enc->get_enc() - origin);
        printf("% 4.0f ", rad_to_deg(present_rad));
        printf("% 4.0f ", rad_to_deg(target_angle));
        printf("% 4.0f ", rad_to_deg(new_tag_angle));
        printf("%6d ", c620->get_raw_tgt_current());
      }
    }
    void set_target(int16_t angle) {
      if(angle * 1e-3 >= bottom_rad) {
        target_angle = angle * 1e-3;
      } else {
        target_angle = bottom_rad;
        state = Waiting;
      }
    }
    void enter_running() {
      origin = enc->get_enc();
      c620->set_raw_tgt_current(0);
      state = Running;
      pid.reset();
      dt_timer.reset();
      calibrate_timeout.stop();
    }
    float get_angle() const {
      if(state != Running) return NAN;
      return (enc->get_enc() - origin) * enc_to_rad + bottom_rad;
    }
    C620* c620;
    const FirstPenguin* enc;
    DigitalIn* lim;
    enum {
      Waiting,
      Running,
    } state = Waiting;
    PidController pid = {PidGain{}};
    AwaitInterval<> dt_timer{};
    AwaitInterval<> calibrate_timeout{std::nullopt};
    float target_angle = NAN;
    int32_t origin = 0;
  };
  struct ArmLength {
    static constexpr int enc_interval = -9500;
    static constexpr int max_length = 1000;
    static constexpr float enc_to_m = 1e-3 * max_length / enc_interval;
    void task(ArmAngle* ang) {
      // リミットスイッチが押されたら原点を初期化
      if(state == Waiting && !lim->read()) {
        printf("len:stop ");
        fp->set_raw_duty(0);
        enter_running();
      } else if(state == Waiting && !std::isnan(target_length)) {
        if(!calibrate_timeout.await(1500ms)) {
          // キャリブレーション
          printf("len:calibrate ");
          fp->set_raw_duty(-15000);
        } else {
          // 一定時間 リミット踏めなかったらそこを原点にする
          printf("len:stop calibrate ");
          fp->set_raw_duty(0);
          enter_running();
        }
      } else if(state == Running) {
        if(!lim->read()) origin = enc->get_enc();
        const float present_length = (enc->get_enc() - origin) * enc_to_m;
        constexpr float max_vel = 1200 * 1e-3;  // [m/s]
        auto dt = dt_timer();
        const float max = max_vel * std::chrono::duration<float>{dt}.count();
        const float pre_tgt = std::isnan(pid.get_target()) ? present_length : pid.get_target();
        const float new_tag_length = pre_tgt + std::clamp(target_length - pre_tgt, -max, max);
        pid.set_target(new_tag_length);
        pid.update(present_length, dt);
        const float angle = ang->get_angle();
        const float anti_gravity = std::isnan(angle) ? 0 : 0.06 * std::sin(angle);
        fp->set_duty(pid.get_output() + anti_gravity);
        printf("len:");
        printf("%1d ", !lim->read());
        printf("%4ld ", enc->get_enc() - origin);
        printf("%4d ", (int)(present_length * 1e3));
        printf("%4d ", (int)(new_tag_length * 1e3));
        printf("%6d\t", fp->get_raw_duty());
      }
    }
    void set_target(int16_t length) {
      if(length >= 0) {
        target_length = length * 1e-3;
      } else {
        target_length = 0;
        state = Waiting;
      }
    }
    void enter_running() {
      calibrate_timeout.stop();
      origin = enc->get_enc();
      state = Running;
      pid.reset();
      dt_timer.reset();
    }
    FirstPenguin* fp;
    const FirstPenguin* enc;
    DigitalIn* lim;
    enum {
      Waiting,
      Running,
    } state = Waiting;
    PidController pid = {PidGain{}};
    AwaitInterval<> dt_timer{};
    AwaitInterval<> calibrate_timeout{std::nullopt};
    float target_length = NAN;
    int32_t origin = 0;
  };
  struct LargeWheel {
    void task() {
      duty += (tag_duty - duty) / 2;  // ローパスフィルタ
      c620_arr[0]->set_raw_tgt_current(-std::clamp((int)duty, -16384, 16384));
      c620_arr[1]->set_raw_tgt_current(std::clamp((int)duty, -16384, 16384));
      printf("l:");
      for(auto& e: c620_arr) printf("% 4.1f ", e->get_actual_current());
    }
    C620* c620_arr[2];
    int16_t tag_duty = 0;
    int16_t duty = 0;
  };

  void task() {
    donfan.task();
    expander.task();
    collector.task();
    arm_angle.task();
    arm_length.task(&arm_angle);
    large_wheel.task();
  }

  Donfan donfan;
  Expander expander;
  Collector collector;
  ArmAngle arm_angle;
  ArmLength arm_length;
  LargeWheel large_wheel;
};

#endif  // MECHANISM_HPP
