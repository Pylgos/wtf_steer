#ifndef MECHANISM_HPP
#define MECHANISM_HPP

#include <mbed.h>

#include <c620.hpp>
#include <first_penguin.hpp>
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
      if(lim[0] && dir == 1) dir = 0;
      if(lim[1] && dir == -1) dir = 0;

      if(dir == 1) {
        fp->set_raw_duty(8000);
      } else if(dir == -1) {
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
    void task() {
      auto now = HighResClock::now();
      pid.update(-fp->get_enc(), now - pre);
      fp->set_duty(-pid.get_output());
      pre = now;
    }
    FirstPenguin* fp;
    PidController pid = {PidGain{.kp = 0.0015, .max = 0.9, .min = -0.9}};
    decltype(HighResClock::now()) pre = HighResClock::now();
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
        origin = fp->get_enc() + 60 * deg2enc;
        state = Running;
        pre = HighResClock::now();
      } else if(state == Waiting && !std::isnan(target_angle)) {
        // キャリブレーション
        c620->set_raw_tgt_current(2000);
      } else if(state == Running) {
        auto now = HighResClock::now();
        chrono::duration<float> dt = now - set_time;
        float t = std::clamp(dt.count(), 0.0f, 1.0f);
        if(std::isnan(pre_tgt_angle)) pre_tgt_angle = 0;
        float new_tag_angle = std::lerp(pre_tgt_angle, target_angle, t);
        pid.set_target(new_tag_angle);
        auto present_rad = (fp->get_enc() - origin) * enc_to_rad;
        pid.update(present_rad * 1000, now - pre);
        float anti_gravity = 3000 * std::cos(M_PI / 6 + present_rad);
        c620->set_raw_tgt_current(-std::clamp(16384 * pid.get_output() + anti_gravity, -6000.0f, 6000.0f));
        pre = now;
      }
    }
    void set_target(int16_t angle) {
      if(target_angle == angle) return;
      pre_tgt_angle = target_angle;
      target_angle = angle;
      set_time = HighResClock::now();
    }
    C620* c620;
    FirstPenguin* fp;
    DigitalIn* lim;
    enum {
      Waiting,
      Running,
    } state;
    PidController pid = {PidGain{.kp = 0.4e-3, .ki = 0.03e-3, .max = 0.9, .min = -0.9}};
    decltype(HighResClock::now()) pre = {};
    decltype(HighResClock::now()) set_time = {};
    float pre_tgt_angle = NAN;
    float target_angle = NAN;
    int32_t origin = 0;
  };
  struct ArmLength {
    static constexpr int enc_interval = 23000;
    static constexpr int max_length = 900;
    static constexpr float enc_to_mm = (float)max_length / enc_interval;
    void task() {
      // リミットスイッチが押されたら原点を初期化
      if(state == Waiting && !lim->read()) {
        origin = fp->get_enc();
        state = Running;
        pre = HighResClock::now();
      } else if(state == Waiting && !std::isnan(pid.get_target())) {
        // キャリブレーション
        fp->set_duty(3000);
      } else if(state == Running) {
        auto now = HighResClock::now();
        pid.update((fp->get_enc() - origin) * enc_to_mm, now - pre);
        fp->set_duty(pid.get_output());
        pre = now;
      }
    }
    void set_target(int16_t length) {
      pid.set_target(length);
    }
    FirstPenguin* fp;
    DigitalIn* lim;
    enum {
      Waiting,
      Running,
    } state = Waiting;
    PidController pid = {PidGain{.kp = 0.5, .max = 0.9, .min = -0.9}};
    decltype(HighResClock::now()) pre = {};
    int32_t origin = 0;
  };
  struct LargeWheel {
    void task() {
      duty += (tag_duty - duty) / 2;  // ローパスフィルタ
      fp_arr[0]->set_raw_duty(duty);
      fp_arr[1]->set_raw_duty(-duty);
      c620_arr[0]->set_raw_tgt_current(duty);
      c620_arr[1]->set_raw_tgt_current(-duty);
    }
    FirstPenguin* fp_arr[2];
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
