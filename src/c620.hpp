#ifndef RCT_C620_HPP
#define RCT_C620_HPP

#include <mbed.h>

#include <anglelib.hpp>
#include <array>

class C620 {
  using Direction = anglelib::Directionf;

 public:
  C620() {}

  void set_gear_ratio(float gear_ratio) {
    gear_ratio_ = gear_ratio;
  }
  float get_gear_ratio() {
    return gear_ratio_;
  }

  void set_raw_tgt_current(int16_t raw_tgt_current) {
    raw_tgt_current_ = raw_tgt_current;
  }
  int16_t get_raw_tgt_current() const {
    return raw_tgt_current_;
  }

  void set_tgt_current(const float tgt_current) {
    raw_tgt_current_ = std::clamp(tgt_current, -20.0f, 20.0f) * (16384.0f / 20.0f);
  }
  float get_tgt_current() const {
    return raw_tgt_current_ * (20.0f / 16384.0f);
  }

  void set_tgt_torque(const float tgt_torque) {
    set_tgt_current(tgt_torque * (8.0f / 3.0f) * gear_ratio_);
  }
  float get_tgt_torque() {
    return get_tgt_current() * (3.0f / 8.0f) / gear_ratio_;
  }

  Direction get_direction() {
    return direction_;
  }
  float get_ang_vel() {
    return ang_vel_;
  }
  float get_actual_current() const {
    return actual_current_;
  }

  void parse_packet(const uint8_t data[8]) {
    direction_ = Direction::from_rad((data[0] << 8 | data[1]) * (float)(2 * anglelib::PI / 8191.0));
    ang_vel_ = (int16_t)(data[2] << 8 | data[3]) * (2 * anglelib::PI / 60.0) * gear_ratio_;
    actual_current_ = (int16_t)(data[4] << 8 | data[5]) * (20.0f / 16384.0f);
    temp_ = data[6];
  }

 private:
  float gear_ratio_ = 1;

  int16_t raw_tgt_current_ = 0;

  Direction direction_{0};
  float ang_vel_ = 0;
  float actual_current_ = 0;
  uint8_t temp_ = 0;
};

struct C620Array {
  auto begin() {
    return ary_.begin();
  }
  auto begin() const {
    return ary_.begin();
  }
  auto end() {
    return ary_.end();
  }
  auto end() const {
    return ary_.end();
  }
  C620& operator[](int index) {
    return ary_[index];
  }

  const C620& operator[](int index) const {
    return ary_[index];
  }

  std::array<CANMessage, 2> to_msgs() const {
    std::array<CANMessage, 2> result;
    for(size_t i = 0; i < result.size(); ++i) {
      for(size_t j = 0; j < 4; ++j) {
        auto&& c620 = ary_[i * 4 + j];
        result[i].data[j * 2] = c620.get_raw_tgt_current() >> 8;
        result[i].data[j * 2 + 1] = c620.get_raw_tgt_current() & 0xff;
      }
    }
    result[0].id = 0x200;
    result[1].id = 0x1ff;
    return result;
  }

  void parse_packet(const CANMessage& msg) {
    if(msg.format == CANStandard && msg.type == CANData && msg.len == 8 && 0x201 <= msg.id && msg.id <= 0x208) {
      ary_[msg.id - 0x201u].parse_packet(msg.data);
    }
  }

 private:
  std::array<C620, 8> ary_;
};

#endif  /// RCT_C620_H_
