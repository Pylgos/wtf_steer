#ifndef RCT_AWAIT_INTERVAL_HPP_
#define RCT_AWAIT_INTERVAL_HPP_
/// @file
/// @brief Provides AwaitInterval class.
/// @copyright Copyright (c) 2023 Yoshikawa Teru

#include <mbed.h>

#include <optional>

/// @brief Await time intervals in non-blocking and measure the elapsed time.
template<class Clock = mbed::HighResClock>
struct AwaitInterval {
  auto operator()(typename Clock::duration interval = {}) {
    auto now = Clock::now();
    auto dt = now - *pre_;
    auto elapsed = dt > interval;
    if(elapsed) reset(now);
    return Result{dt, elapsed};
  }
  auto await(typename Clock::duration interval = {}) {
    if(!pre_) pre_ = Clock::now();
    return this->operator()(interval);
  }
  auto elapsed(typename Clock::duration interval = {}) {
    if(!pre_) pre_ = Clock::now();
    auto now = Clock::now();
    auto dt = now - *pre_;
    auto elapsed = dt > interval;
    return Result{dt, elapsed};
  }
  void reset(typename Clock::time_point init = Clock::now()) {
    pre_ = init;
  }
  void stop() {
    pre_ = std::nullopt;
  }
  explicit operator bool() const {
    return pre_.has_value();
  }
  struct Result : Clock::duration {
    constexpr explicit operator bool() const {
      return elapsed;
    }
    bool elapsed;
  };
  std::optional<typename Clock::time_point> pre_ = Clock::now();
};

#endif  // RCT_AWAIT_INTERVAL_HPP_
