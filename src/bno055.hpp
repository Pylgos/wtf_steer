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
  bool try_init(const std::chrono::microseconds timeout) {
    auto start = HighResClock::now();
    while(HighResClock::now() - start < timeout) {
      if(request_config_mode() != Response::WRITE_SUCCESS) {
        printf("bno055:Operating Mode setting CONFIGMODE\n");
        continue;
      }
      if(request_imu_mode() != Response::WRITE_SUCCESS) {
        printf("bno055:Operating Mode setting IMU\n");
        continue;
      }
      return true;
    }
    return false;
  }
  Response request_config_mode() {
    return reg_write(OPR_MODE, {0x00});
  }
  Response request_imu_mode() {
    return reg_write(OPR_MODE, {0x08});
  }
  Response request_euler_x() {
    return reg_read(EUL_DATA_X, euler_angle.x);
  }
  Response request_euler_y() {
    return reg_read(EUL_DATA_Y, euler_angle.y);
  }
  Response request_euler_z() {
    return reg_read(EUL_DATA_Z, euler_angle.z);
  }
  Response request_euler_angle() {
    return reg_read(EUL_DATA, euler_angle);
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
    EUL_DATA_X = 0x1A,
    EUL_DATA_Y = 0x1C,
    EUL_DATA_Z = 0x1E,
    EUL_DATA = EUL_DATA_X,
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
    return Response::NO_RESPONSE;
  }
  template<int N>
  Response reg_read(const uint8_t addr, uint8_t (&buf)[N]) {
    static_assert(N <= 128);
    bus_.flush_read_buffer();
    bus_.uart_transmit({0xAA, 0x01, addr, N});
    if(uint8_t buf_byte; bus_.uart_receive(buf_byte, timeout)) {
      switch(buf_byte) {
        case 0xBB: {
          // Success
          if(bus_.uart_receive(buf_byte, timeout) && buf_byte == N) {      // length
            if(uint8_t raw_buf[N]; bus_.uart_receive(raw_buf, timeout)) {  // register data
              memcpy(buf, raw_buf, N);
              return Response::WRITE_SUCCESS;
            }
          }
          break;
        }
        case 0xEE: {
          // Fail
          if(bus_.uart_receive(buf_byte, timeout)) {
            return Response{buf_byte};
          }
          break;
        }
      }
    }
    return Response::NO_RESPONSE;
  }
  template<class T>
  Response reg_read(const uint8_t addr, T& buf) {
    return reg_read(addr, reinterpret_cast<uint8_t(&)[sizeof(buf)]>(buf));
  }
  static float to_rad(const int16_t val) {
    constexpr auto deg_rep = 16;
    constexpr float k = 2 * M_PI / (360 * deg_rep);
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
