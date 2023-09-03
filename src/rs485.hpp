#ifndef RCT_RS485_HPP
#define RCT_RS485_HPP

#include <mbed.h>

struct Rs485 {
  Rs485(const PinName tx, const PinName rx, const int baud, const PinName data_enable)
      : bus_{PB_6, PA_10, baud}, data_enable_{data_enable} {
    bus_.set_blocking(0);
    timer.start();
  }

  void send(void* buf, const int len) {
    data_enable_.write(1);
    bus_.write(buf, len);
    wait_us(5);
    data_enable_.write(0);
  }

  bool recv(void* buf, const size_t len, const std::chrono::microseconds timeout) {
    timer.reset();
    size_t bytes_read = 0;
    while(bytes_read != len) {
      if(bus_.readable()) {
        int ret = bus_.read((void*)((size_t)buf + bytes_read), len - bytes_read);
        if(ret > 0) {
          bytes_read += ret;
        }
      }
      if(timer.elapsed_time() > timeout) {
        return false;
      }
    }
    return true;
  }

  void flush_read_buffer() {
    uint8_t buf;
    while(bus_.read(&buf, 1) > 0) {}
  }

  Timer timer;
  BufferedSerial bus_;
  DigitalOut data_enable_;
};

#endif  // RCT_RS485_H_
