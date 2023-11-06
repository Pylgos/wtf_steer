#ifndef SERVO_HPP_
#define SERVO_HPP_

#include <mbed.h>

struct Servo {
  void set_deg(const uint8_t degree) {
    raw_out = degree * 0xff / 180;
  }
  uint8_t raw_out;
};

struct ServoArray {
  ServoArray(const uint32_t can_id) : can_id_{can_id} {}
  CANMessage to_msg() {
    static_assert(sizeof(servo_) <= 8);
    return CANMessage{can_id_, reinterpret_cast<const uint8_t*>(servo_), sizeof(servo_)};
  }
  Servo& operator[](int idx) & {
    return servo_[idx];
  }
  auto begin() {
    return std::begin(servo_);
  }
  auto end() {
    return std::end(servo_);
  }

 private:
  uint32_t can_id_;
  Servo servo_[8] = {};
};

#endif  // SERVO_HPP_
