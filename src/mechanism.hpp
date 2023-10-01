#ifndef MECHANISM_HPP
#define MECHANISM_HPP

#include <mbed.h>

#include <c620.hpp>
#include <first_penguin.hpp>
#include <optional>
#include <pid_controller.hpp>

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
      if(dir == 1 && !lim[0]) {
        fp->set_raw_duty(8000);
      } else if(dir == -1 && !lim[1]) {
        fp->set_raw_duty(-8000);
      } else {
        fp->set_raw_duty(0);
      }
    }
    FirstPenguin* fp;
    DigitalIn* lim_fwd;
    DigitalIn* lim_rev;
    int8_t dir = 0;
  };
  struct Expander {
    static constexpr int enc_interval = -1474;
    void task() {
      if(state == Waiting && !lim->read()) {
        // 原点合わせ
        origin = fp->get_enc();
        state = Running;
        pre = HighResClock::now();
      } else if(state == Waiting && !std::isnan(target)) {
        // キャリブレーション
        printf("exp:calibrate ");
        fp->set_raw_duty(3000);
      } else if(state == Running) {
        if(!lim->read()) origin = fp->get_enc();
        if(!lim_top->read()) normalization = 1.0f / (fp->get_enc() - origin);
        auto now = HighResClock::now();
        float present_length = (fp->get_enc() - origin) * normalization;
        // ローパスフィルタ
        float previous_tgt = pid.get_target();
        if(std::isnan(previous_tgt)) previous_tgt = present_length;
        previous_tgt += (target - previous_tgt) / 2;
        pid.set_target(previous_tgt);
        pid.update(present_length, now - pre);
        fp->set_duty(-pid.get_output());
        pre = now;
        printf("exp:");
        printf("%1d ", !lim_top->read() << 1 | !lim->read());
        printf("% 6ld ", fp->get_enc() - origin);
        printf("% 6f ", present_length);
        printf("% 6f ", pid.get_target());
        printf("% 6d ", fp->get_raw_duty());
      }
    }
    void set_target(int16_t height) {
      if(height >= 0) {
        target = height / 900.0f;
      } else {
        // キャリブレーション
        target = 0;
        state = Waiting;
      }
    }
    FirstPenguin* fp;
    DigitalIn* lim;
    DigitalIn* lim_top;
    enum {
      Waiting,
      Running,
    } state = Waiting;
    float target = NAN;
    PidController pid = {PidGain{}};
    decltype(HighResClock::now()) pre = {};
    int32_t origin = 0;
    float normalization = 1.0f / enc_interval;
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
        case Stop:
          fp->set_raw_duty(0);
          break;
        case Running:
        case Storing:
          fp->set_raw_duty(-8000);
          break;
      }
    }
    FirstPenguin* fp;
    DigitalIn* lim;
    enum {
      Stop,
      Running,
      Storing,
    } state = Stop;
    bool collecting = false;
  };
  struct ArmAngle {
    static constexpr int enc_interval = 3800;
    static constexpr int deg2enc = enc_interval / (100 - -60);
    // -60deg -> 100deg
    static constexpr float enc_to_rad = M_PI / enc_interval;
    static constexpr float enc_to_mrad = enc_to_rad * 1000;
    void task() {
      if(state == Waiting && !lim->read()) {
        // 原点セット
        enter_running();
      } else if(state == Waiting && !std::isnan(target_angle)) {
        const auto now = HighResClock::now();
        if(!calibrate_start) calibrate_start = now;
        if(now - *calibrate_start < 3s) {
          // キャリブレーション
          printf("ang:calibrate ");
          c620->set_raw_tgt_current(2000);
        } else {
          // 3s リミット踏めなかったらそこを原点にする
          printf("len:stop calibrate ");
          enter_running();
        }
      } else if(state == Running) {
        auto now = HighResClock::now();
        if(!lim->read()) origin = fp->get_enc() + 60 * deg2enc;
        auto present_rad = (fp->get_enc() - origin) * enc_to_rad;
        constexpr float max_omega = 1.5f;  // [rad/sec]
        auto max = max_omega * chrono::duration<float>{now - pre}.count();
        float pre_tgt = std::isnan(pid.get_target()) ? present_rad : pid.get_target();
        float new_tag_angle = pre_tgt + std::clamp(target_angle - pre_tgt, -max, max);
        constexpr float max_distance = M_PI / 4;
        new_tag_angle = present_rad + std::clamp(new_tag_angle - present_rad, -max_distance, max_distance);
        pid.set_target(new_tag_angle);
        pid.update(present_rad, now - pre);
        float anti_gravity = 1500 * std::cos(present_rad);
        c620->set_raw_tgt_current(-std::clamp(16384 * pid.get_output() + anti_gravity, -16384.0f, 16384.0f));
        pre = now;
        printf("ang:");
        printf("%1d ", !lim->read());
        printf("%6ld ", fp->get_enc() - origin);
        printf("% f ", present_rad);
        printf("% f ", target_angle);
        printf("% f ", new_tag_angle);
        printf("%6d ", c620->get_raw_tgt_current());
      }
    }
    void set_target(int16_t angle) {
      if(angle >= -M_PI / 3 * 1e3) {
        target_angle = angle * 1e-3;
      } else {
        target_angle = -60 * deg2enc;
        state = Waiting;
      }
    }
    bool is_top() const {
      auto present = (fp->get_enc() - origin);
      bool top = 75 * deg2enc < present && present < 105 * deg2enc;
      return state == Running && top;
    }
    bool is_up() const {
      return state == Running && (fp->get_enc() - origin) * enc_to_rad > M_PI / 6;
    }
    void enter_running() {
      origin = fp->get_enc() + 60 * deg2enc;
      c620->set_raw_tgt_current(0);
      state = Running;
      pre = HighResClock::now();
      calibrate_start = std::nullopt;
    }
    C620* c620;
    FirstPenguin* fp;
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
    static constexpr int enc_interval = 2680;
    static constexpr int max_length = 900;
    static constexpr float enc_to_m = 1e-3 * max_length / enc_interval;
    void task(Mechanism* mech) {
      // リミットスイッチが押されたら原点を初期化
      if(state == Waiting && !lim->read()) {
        printf("len:stop ");
        fp->set_raw_duty(0);
        if(mech->arm_angle.is_top()) enter_running();
      } else if(state == Waiting && (!std::isnan(pid.get_target()) || mech->arm_angle.is_top())) {
        auto now = HighResClock::now();
        if(!calibrate_start) calibrate_start = now;
        if(now - *calibrate_start < 3s) {
          // キャリブレーション
          printf("len:calibrate ");
          fp->set_raw_duty(-15000);
        } else {
          // 3s リミット踏めなかったらそこを原点にする
          printf("len:stop calibrate ");
          fp->set_raw_duty(0);
          if(mech->arm_angle.is_top()) enter_running();
        }
      } else if(state == Running && !mech->arm_angle.is_up()) {
        // 角度調整が下がれば原点を忘れる -> 上げるたびキャリブレーション必須
        pid.set_target(NAN);
        state = Waiting;
      } else if(state == Running) {
        auto now = HighResClock::now();
        if(!lim->read()) origin = fp->get_enc();
        const float present_length = (fp->get_enc() - origin) * enc_to_m;
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
        printf("%4ld ", fp->get_enc() - origin);
        printf("%4d ", (int)(present_length * 1e3));
        printf("%4d ", (int)(new_tag_length * 1e3));
        printf("%6d\t", fp->get_raw_duty());
      }
    }
    void set_target(int16_t length) {
      if(length >= 0) {
        target_length = length * 1e-3;
      } else {
        target_length = NAN;
        state = Waiting;
      }
    }
    void enter_running() {
      calibrate_start = std::nullopt;
      origin = fp->get_enc();
      state = Running;
      pre = HighResClock::now();
    }
    FirstPenguin* fp;
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
    arm_length.task(this);
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
