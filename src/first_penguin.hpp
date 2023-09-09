#ifndef FIRST_PENGUIN_HPP
#define FIRST_PENGUIN_HPP

#include <mbed.h>

struct FPRxPacket {
  int32_t enc_;
  uint32_t adc_;
};

class FirstPenguin {
 public:
  void set_invert(bool invert) {
    scale_ = invert ? -1 : 1;
  }

  void set_raw_duty(int16_t duty) {
    raw_duty_ = duty;
  }
  int16_t get_raw_duty() const {
    return raw_duty_;
  }

  void set_duty(const float duty) {
    raw_duty_ = std::clamp(duty, -1.0f, 1.0f) * 16384 * scale_;
  }
  float get_duty() const {
    return raw_duty_ * (1.0f / 16384.0f) / scale_;
  }

  void parse_packet(const uint8_t data[8]) {
    memcpy(&rx, data, sizeof(rx));
  }
  int32_t get_enc() const {
    return rx.enc_;
  }

 private:
  int8_t scale_ = 1;
  int16_t raw_duty_ = 0;
  FPRxPacket rx = {};
};


class FirstPenguinArray {
 public:
  FirstPenguinArray(uint16_t base_can_id) : base_can_id_{base_can_id} {}

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

  FirstPenguin& operator[](int index) {
    return ary_[index];
  }

  const FirstPenguin& operator[](int index) const {
    return ary_[index];
  }

  CANMessage to_msg() const {
    CANMessage msg;
    msg.id = base_can_id_;
    msg.len = 8;
    for(size_t i = 0; i < 4; i++) {
      msg.data[i * 2] = ary_[i].get_raw_duty() & 0xff;
      msg.data[i * 2 + 1] = (ary_[i].get_raw_duty() >> 8) & 0xff;
    }
    return msg;
  }

  void parse_packet(const CANMessage& msg) {
    if(msg.format == CANStandard && msg.type == CANData && msg.len == sizeof(FPRxPacket) && base_can_id_ < msg.id &&
       msg.id <= base_can_id_ + 5u) {
      ary_[msg.id - base_can_id_ - 1].parse_packet(msg.data);
    }
  }

 private:
  uint16_t base_can_id_;
  std::array<FirstPenguin, 4> ary_;
};


#endif
