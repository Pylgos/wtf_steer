#ifndef BNO055_HPP
#define BNO055_HPP

#include <mbed.h>

#include <Serial.hpp>

struct Bno055 {
  static constexpr auto timeout = 5ms;
  enum class Response : uint8_t {
    NO_RESPONSE = 0x00,  // 独自定義
    WRITE_SUCCESS = 0x01,
    READ_FAIL,
    WRITE_FAIL,
    REGMAP_INVALID_ADDRESS,
    REGMAP_WRITE_DISABLED,
    WRONG_START_BYTE,
    BUS_OVER_RUN_ERROR,
    MAX_LENGTH_ERROR,
    MIN_LENGTH_ERROR,
    RECEIVE_CHARACTER_TIMEOUT,
  };

  Bno055(PinName tx, PinName rx) : bus_{{tx, rx, 115200}} {}
  void init() {
    while(reg_write(OPR_MODE, {0x00}) != Response::WRITE_SUCCESS) {
      printf("bno055:Operating Mode setting CONFIGMODE\n");
    }
    ThisThread::sleep_for(19ms);
    while(reg_write(UNIT_SEL, {0x04}) != Response::WRITE_SUCCESS) {
      printf("bno055:UNIT_SEL setting\n");
    }
    while(reg_write(AXIS_MAP_CONFIG, {0x24}) != Response::WRITE_SUCCESS) {
      printf("bno055:AXIS_MAP_CONFIG setting\n");
    }
    while(reg_write(OPR_MODE, {0x08}) != Response::WRITE_SUCCESS) {
      printf("bno055:Operating Mode setting IMU\n");
    }
  }
  Response request_euler_angle() {
    return reg_read(0x1A, euler_angle);
  }
  float get_x_rad() const {
    return to_rad(euler_angle.x);
  }
  float get_y_rad() const {
    return to_rad(euler_angle.y);
  }
  float get_z_rad() const {
    return to_rad(euler_angle.z);
  }

 private:
  enum Register {
    UNIT_SEL = 0x3B,
    OPR_MODE = 0x3D,
    AXIS_MAP_CONFIG = 0x41,
    AXIS_MAP_SIGN = 0x42,
  };

  template<int N>
  Response reg_write(const uint8_t addr, const uint8_t (&data)[N]) {
    static_assert(N <= 128);
    bus_.flush_read_buffer();
    bus_.uart_transmit({0xAA, 0x00, addr, N});
    bus_.uart_transmit(data);
    if(uint8_t buf[2] = {}; bus_.uart_receive(buf, timeout) && buf[0] == 0xEE) {
      return Response{buf[1]};
    }
    return Response{};
  }
  template<int N>
  Response reg_read(const uint8_t addr, uint8_t (&buf)[N]) {
    static_assert(N <= 128);
    bus_.flush_read_buffer();
    bus_.uart_transmit({0xAA, 0x01, addr, N});
    if(uint8_t buf_byte; bus_.uart_receive(buf_byte, timeout)) {
      switch(buf_byte) {
        case 0xBB: {
          if(bus_.uart_receive(buf_byte, timeout) && buf_byte == N && bus_.uart_receive(buf, timeout)) {
            return Response::WRITE_SUCCESS;
          }
          break;
        }
        case 0xEE: {
          if(bus_.uart_receive(buf_byte, timeout)) {
            return Response{buf_byte};
          }
          break;
        }
      }
    }
    return Response{};
  }
  template<class T>
  Response reg_read(const uint8_t addr, T& buf) {
    return reg_read(addr, reinterpret_cast<uint8_t(&)[sizeof(buf)]>(buf));
  }
  static float to_rad(const int16_t val) {
    constexpr auto radian_representation = 900;
    constexpr float k = 1.0f / radian_representation;
    return val * k;
  }

  Serial bus_;
  struct Euler {
    int16_t x;
    int16_t y;
    int16_t z;
  } euler_angle;
};

#endif  // BNO055_HPP
