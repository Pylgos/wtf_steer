#ifndef SERIAL_HPP_
#define SERIAL_HPP_

#include <mbed.h>

struct Serial {
  template<int N>
  void uart_transmit(const uint8_t (&send)[N]) {
    bus_.write(send, N);
  }
  template<int N>
  bool uart_receive(uint8_t (&buf)[N], const std::chrono::microseconds timeout) {
    const auto pre = HighResClock::now();
    uint8_t* p = std::begin(buf);
    while(HighResClock::now() - pre < timeout) {
      if(bus_.readable() && bus_.read(p, 1) > 0 && ++p == std::end(buf)) return true;
    }
    return false;
  }
  template<class T>
  bool uart_receive(T& buf, const std::chrono::microseconds timeout) {
    return uart_receive(reinterpret_cast<uint8_t(&)[sizeof(buf)]>(buf), timeout);
  }
  void flush_read_buffer() {
    uint8_t buf;
    while(bus_.readable() && bus_.read(&buf, 1) > 0) {}
  }
  UnbufferedSerial bus_;
};

#endif  // SERIAL_HPP_
