#ifndef AMT21_HPP
#define AMT21_HPP

#include <mbed.h>

#include <anglelib.hpp>
#include <array>
#include <rs485.hpp>

class Amt21 {
  static constexpr float ticks_to_rads = (2.0 * anglelib::PI) / 4096.0;
  using Angle = anglelib::Anglef;
  using Direction = anglelib::Directionf;

 public:
  Amt21(Rs485* rs485, uint8_t address, float scale = 1.0, Angle offset = Angle::zero())
      : rs485_{rs485}, address_{address}, scale_{scale}, offset_{offset} {
    MBED_ASSERT(rs485);
  }

  Angle get_angle() {
    return angle_;
  }
  void set_offset(Angle offset) {
    offset_ = offset;
  }
  Angle get_offset() {
    return offset_;
  }

  bool update_pos() {
    Angle closest{1e9};
    for (size_t i = 0; i < 3; ++i) {
      Angle ang;
      if (!read_raw_angle(ang)) {
        return false;
      }
      
      if ((raw_angle_ - ang).abs() < (raw_angle_ - closest).abs()) {
        closest = ang;
      }
    }

    raw_angle_ = closest;
    angle_ = raw_angle_ * scale_ + offset_;
    return true;
  }

  void reset() {
    std::array<uint8_t, 2> req = {(uint8_t)(address_ + 2), 0x75};
    rs485_->send(req.data(), req.size());
  }

 private:
  Rs485* rs485_;
  uint8_t address_;
  float scale_;
  Angle offset_;
  Angle angle_ = {};
  Angle raw_angle_ = {};

  bool read_raw_angle(Angle& result) {
    for (size_t i = 0; i < 3; ++i) {
      rs485_->flush_read_buffer();
      rs485_->send(&address_, 1);
      if(uint16_t resp; rs485_->recv(&resp, sizeof(resp), 20us) && (is_valid(resp))) {
        uint16_t ticks = (resp & 0b0011'1111'1111'1111) >> 2;
        auto dir = Direction::from_rad(ticks * ticks_to_rads);
        result = raw_angle_.closest_angle_of(dir);
        return true;
      }
    }
    return false;
  }

  static bool is_valid(uint16_t raw_data) {
    auto b = [raw_data](int pos) {
      return (raw_data >> pos) & 1;
    };
    bool k1 = b(15);
    bool k0 = b(14);
    bool k1_computed = !(b(13) ^ b(11) ^ b(9) ^ b(7) ^ b(5) ^ b(3) ^ b(1));
    bool k0_computed = !(b(12) ^ b(10) ^ b(8) ^ b(6) ^ b(4) ^ b(2) ^ b(0));

    return (k1 == k1_computed) && (k0 == k0_computed);
  }
};

#endif
