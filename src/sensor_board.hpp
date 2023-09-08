#ifndef SENSOR_BOARD_HPP
#define SENSOR_BOARD_HPP

#include <mbed.h>

struct SensorBoard {
  SensorBoard(const uint32_t id_A, const uint32_t id_B) : id_{id_A, id_B} {};

  bool parse_packet(const CANMessage& msg) {
    if(msg.format == CANStandard && msg.type == CANData && msg.len == 8) {
      if(msg.id == id_[0]) {
        set_forward(msg.data);
        return true;
      } else if(msg.id == id_[1]) {
        set_backward(msg.data);
        return true;
      }
    }
    return false;
  }
  bool get_lim(const int idx) const {
    return lim_ & (1 << idx);
  }
  int16_t get_enc(const int idx) const {
    return enc_[idx];
  }
  uint64_t get_time_us() const {
    return time_us_;
  }

 private:
  void set_forward(const uint8_t data[8]) {
    time_us_ = uint64_t{data[4]} << 32 | data[3] << 24 | data[2] << 16 | data[1] << 8 | data[0];
    lim_ = data[5];
    enc_[4] = data[7] << 8 | data[6];
  }
  void set_backward(const uint8_t data[8]) {
    enc_[3] = data[1] << 8 | data[0];
    enc_[2] = data[3] << 8 | data[2];
    enc_[1] = data[5] << 8 | data[4];
    enc_[0] = data[7] << 8 | data[6];
  }

  const unsigned id_[2];
  uint64_t time_us_ = {};
  uint8_t lim_ = {};
  int16_t enc_[5] = {};
};

#endif  // SENSOR_BOARD_HPP
