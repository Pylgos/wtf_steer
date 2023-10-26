#ifndef MECHANISM_HPP
#define MECHANISM_HPP

#include <mbed.h>

#include <c620.hpp>
#include <first_penguin.hpp>
#include <optional>
#include <pid_controller.hpp>
#include <servo.hpp>

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
      if(dir_ == 1 && !lim[0] && !is_timeout()) {
        fp->set_raw_duty(-8000);
      } else if(dir_ == -1 && !lim[1] && !is_timeout()) {
        fp->set_raw_duty(8000);
      } else {
        fp->set_raw_duty(0);
      }
    }
    void set_dir(uint8_t dir) {
      dir_ = dir;
      pre_ = HighResClock::now();
    }
    bool is_timeout() {
      auto now = HighResClock::now();
      return now - pre_ > 3s;
    }
    FirstPenguin* fp;
    DigitalIn* lim_fwd;
    DigitalIn* lim_rev;
    int8_t dir_ = 0;
    HighResClock::time_point pre_;
  };
  struct Expander {
    static constexpr int enc_interval = -1474;
    void task() {
      if(state == Waiting && !lim->read()) {
        // 原点合わせ
        set_origin();
        set_lock(false);
        fp->set_raw_duty(0);
        enter_running();
      } else if(state == Waiting && !std::isnan(target)) {
        // キャリブレーション
        auto now = HighResClock::now();
        if(!calibrate_start) calibrate_start = now;
        if(now - *calibrate_start < 1500ms) {
          printf("exp:calibrate ");
          set_lock(false);
          fp->set_raw_duty(15000);
        } else {
          printf("exp:calibrate stop ");
          enter_running();
        }
      } else if(state == Running) {
        if(!lim->read()) set_origin();
        auto now = HighResClock::now();
        float present_length = 1.0f / enc_interval * (enc->get_enc() - origin);
        // ローパスフィルタ
        float previous_tgt = pid.get_target();
        if(std::isnan(previous_tgt)) previous_tgt = present_length;
        previous_tgt += (target - previous_tgt) / 2;
        pid.set_target(previous_tgt);
        pid.update(present_length, now - pre);
        fp->set_duty(-pid.get_output());
        // 目標値が現在位置より下ならlock
        set_lock(previous_tgt - present_length < 0);
        pre = now;
        printf("exp:");
        printf("%1d ", !lim->read());
        printf("% 6ld ", enc->get_enc() - origin);
        printf("% 5.2f ", present_length);
        printf("% 5.2f ", pid.get_target());
        printf("% 6d ", fp->get_raw_duty());
      }
    }
    void set_lock(bool is_lock) {
      servo->set_deg(is_lock ? 45 : 90);
    }
    void set_target(int16_t height) {
      if(height >= 0) {
        target = height / 1000.0f;
      } else {
        // キャリブレーション
        target = 0;
        state = Waiting;
      }
    }
    void enter_running() {
      calibrate_start = std::nullopt;
      origin = enc->get_enc();
      state = Running;
      pid.reset();
      pre = HighResClock::now();
    }
    void set_origin() {
      origin = enc->get_enc();
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
    decltype(HighResClock::now()) pre = {};
    std::optional<decltype(HighResClock::now())> calibrate_start = std::nullopt;
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

      switch(state) {
        case Stop: {
          servo->set_deg(45);
          fp->set_raw_duty(0);
          break;
        }
        case Running: {
          servo->set_deg(90);
          fp->set_raw_duty(-8000);
          break;
        }
        case Storing: {
          servo->set_deg(45);
          fp->set_raw_duty(-8000);
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
    static constexpr int top_deg = 123;
    static constexpr int bottom_deg = -25;
    static constexpr int enc_interval = 3800;
    static constexpr int deg2enc = enc_interval / (top_deg - bottom_deg);
    static constexpr float enc_to_rad = M_PI / enc_interval;
    static constexpr float enc_to_mrad = enc_to_rad * 1000;
    void task() {
      if(state == Waiting && !lim->read()) {
        // 原点セット
        c620->set_raw_tgt_current(0);
        enter_running();
      } else if(state == Waiting && !std::isnan(target_angle)) {
        const auto now = HighResClock::now();
        if(!calibrate_start) calibrate_start = now;
        if(now - *calibrate_start < 3s) {
          // キャリブレーション
          printf("ang:calibrate ");
          c620->set_raw_tgt_current(-2000);
        } else {
          // 3s リミット踏めなかったらそこを原点にする
          printf("ang:stop calibrate ");
          enter_running();
        }
      } else if(state == Running) {
        auto now = HighResClock::now();
        if(!lim->read()) origin = enc->get_enc() - bottom_deg * deg2enc;
        auto present_rad = (enc->get_enc() - origin) * enc_to_rad;
        constexpr float max_omega = 1.5f;  // [rad/sec]
        auto max = max_omega * chrono::duration<float>{now - pre}.count();
        float pre_tgt = std::isnan(pid.get_target()) ? present_rad : pid.get_target();
        float new_tag_angle = pre_tgt + std::clamp(target_angle - pre_tgt, -max, max);
        constexpr float max_distance = M_PI / 4;
        new_tag_angle = present_rad + std::clamp(new_tag_angle - present_rad, -max_distance, max_distance);
        pid.set_target(new_tag_angle);
        pid.update(present_rad, now - pre);
        float anti_gravity = 1500 * std::cos(present_rad);
        c620->set_raw_tgt_current(std::clamp(16384 * pid.get_output() + anti_gravity, -16384.0f, 16384.0f));
        pre = now;
        printf("ang:");
        printf("%1d ", !lim->read());
        printf("%6ld ", enc->get_enc() - origin);
        printf("% 4.2f ", present_rad);
        printf("% 4.2f ", target_angle);
        printf("% 4.2f ", new_tag_angle);
        printf("%6d ", c620->get_raw_tgt_current());
      }
    }
    void set_target(int16_t angle) {
      if(angle >= bottom_deg * deg2enc * enc_to_mrad) {
        target_angle = angle * 1e-3;
      } else {
        target_angle = bottom_deg * deg2enc;
        state = Waiting;
      }
    }
    void enter_running() {
      origin = enc->get_enc() - bottom_deg * deg2enc;
      c620->set_raw_tgt_current(0);
      state = Running;
      pid.reset();
      pre = HighResClock::now();
      calibrate_start = std::nullopt;
    }
    C620* c620;
    const FirstPenguin* enc;
    DigitalIn* lim;
    enum {
      Waiting,
      Running,
    } state = Waiting;
    PidController pid = {PidGain{}};
    decltype(HighResClock::now()) pre = {};
    std::optional<decltype(HighResClock::now())> calibrate_start = std::nullopt;
    float target_angle = NAN;
    int32_t origin = 0;
  };
  struct ArmLength {
    static constexpr int enc_interval = 15000;
    static constexpr int max_length = 900;
    static constexpr float enc_to_m = 1e-3 * max_length / enc_interval;
    void task() {
      // リミットスイッチが押されたら原点を初期化
      if(state == Waiting && !lim->read()) {
        printf("len:stop ");
        fp->set_raw_duty(0);
        enter_running();
      } else if(state == Waiting && !std::isnan(target_length)) {
        auto now = HighResClock::now();
        if(!calibrate_start) calibrate_start = now;
        if(now - *calibrate_start < 1500ms) {
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
        auto now = HighResClock::now();
        if(!lim->read()) origin = enc->get_enc();
        const float present_length = (enc->get_enc() - origin) * enc_to_m;
        constexpr float max_vel = 1200 * 1e-3;  // [m/s]
        std::chrono::duration<float> dt = now - pre;
        const float max = max_vel * dt.count();
        const float pre_tgt = std::isnan(pid.get_target()) ? present_length : pid.get_target();
        float new_tag_length = pre_tgt + std::clamp(target_length - pre_tgt, -max, max);
        pid.set_target(new_tag_length);
        pid.update(present_length, now - pre);
        fp->set_duty(pid.get_output());
        pre = now;
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
      calibrate_start = std::nullopt;
      origin = enc->get_enc();
      state = Running;
      pid.reset();
      pre = HighResClock::now();
    }
    FirstPenguin* fp;
    const FirstPenguin* enc;
    DigitalIn* lim;
    enum {
      Waiting,
      Running,
    } state = Waiting;
    PidController pid = {PidGain{}};
    decltype(HighResClock::now()) pre = {};
    std::optional<decltype(HighResClock::now())> calibrate_start = std::nullopt;
    float target_length = NAN;
    int32_t origin = 0;
  };
  struct LargeWheel {
    void task() {
      duty += (tag_duty - duty) / 2;  // ローパスフィルタ
      c620_arr[0]->set_raw_tgt_current(duty);
      c620_arr[1]->set_raw_tgt_current(-duty);
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
    arm_length.task();
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
