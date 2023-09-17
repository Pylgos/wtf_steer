#ifndef ANGLE_HPP
#define ANGLE_HPP

#include <cmath>


namespace anglelib {

template<class T>
static constexpr bool is_equal_approx(T a, T b) {
  using std::abs;
  constexpr T CMP_EPSILON = T(0.00001);
  // Check for exact equality first, required to handle "infinity" values.
  if(a == b) {
    return true;
  }
  // Then check for approximate equality.
  T tolerance = CMP_EPSILON * abs(a);
  if(tolerance < CMP_EPSILON) {
    tolerance = CMP_EPSILON;
  }
  return abs(a - b) < tolerance;
}

template<class T>
static constexpr bool is_zero_approx(T a) {
  return is_equal_approx(a, T(0));
}

template<class T>
static constexpr T wrapf(T value, T min, T max) {
  using std::floor;
  T range = max - min;
  if(is_zero_approx(range)) {
    return min;
  }
  T result = value - (range * floor((value - min) / range));
  if(is_equal_approx(result, max)) {
    return min;
  }
  return result;
}

constexpr double PI = 3.1415926535897932384626;
constexpr double TAU = 2 * PI;

template<class Rep>
struct Direction;

template<class Rep>
struct Angle {
  using rep = Rep;
  static constexpr Angle<Rep> zero() {
    return Angle{0};
  };
  static constexpr Angle<Rep> half_turn() {
    return Angle{PI};
  };
  static constexpr Angle<Rep> turn() {
    return Angle{2 * PI};
  };

  constexpr Angle() : value{0} {}

  constexpr explicit Angle(Rep radians) : value{radians} {}

  static constexpr Angle<Rep> from_deg(Rep degrees) {
    return Angle<Rep>(degrees * Rep((2 * PI) / 360));
  }

  static constexpr Angle<Rep> from_rad(Rep radians) {
    return Angle<Rep>(radians);
  }

  constexpr Rep deg() const {
    return value * Rep(360 / (2 * PI));
  }

  constexpr Rep rad() const {
    return value;
  }

  constexpr Angle<Rep>& operator+=(Angle<Rep> rhs) {
    value += rhs.value;
    return *this;
  }

  constexpr Angle<Rep>& operator-=(Angle<Rep> rhs) {
    value -= rhs.value;
    return *this;
  }

  constexpr Angle<Rep>& operator*=(Rep rhs) {
    value *= rhs;
    return *this;
  }

  constexpr Angle<Rep>& operator/=(Rep rhs) {
    value /= rhs;
    return *this;
  }

  constexpr Angle<Rep> operator+(Angle<Rep> rhs) const {
    return Angle<Rep>(value + rhs.value);
  }

  constexpr Angle<Rep> operator-(Angle<Rep> rhs) const {
    return Angle<Rep>(value - rhs.value);
  }

  constexpr Angle<Rep> operator*(Rep rhs) const {
    return Angle<Rep>(value * rhs);
  }

  constexpr Angle<Rep> operator/(Rep rhs) const {
    return Angle<Rep>(value / rhs);
  }

  constexpr bool operator==(Angle<Rep> rhs) const {
    return value == rhs.value;
  }

  constexpr bool operator!=(Angle<Rep> rhs) const {
    return value != rhs.value;
  }

  constexpr bool operator<(Angle<Rep> rhs) const {
    return value < rhs.value;
  }

  constexpr bool operator>(Angle<Rep> rhs) const {
    return value > rhs.value;
  }

  constexpr bool operator<=(Angle<Rep> rhs) const {
    return value <= rhs.value;
  }

  constexpr bool operator>=(Angle<Rep> rhs) const {
    return value >= rhs.value;
  }

  constexpr Angle<Rep> operator+() const {
    return *this;
  }

  constexpr Angle<Rep> operator-() const {
    return Angle<Rep>(-value);
  }

  constexpr Angle<Rep> abs() const {
    using std::abs;
    return Angle<Rep>(abs(value));
  }

  constexpr Direction<Rep> direction() const {
    return Direction<Rep>(value);
  }

  constexpr Angle<Rep> closest_angle_of(Direction<Rep> dir) const {
    Angle<Rep> diff = direction().angle_to(dir);
    return *this + diff;
  }

  Rep value{0};
};

template<class Rep>
struct Direction {
  using rep = Rep;
  static constexpr Direction<Rep> zero() {
    return Direction{0};
  };
  static constexpr Direction<Rep> min() {
    return Direction{0};
  };
  static constexpr Direction<Rep> max() {
    return Direction{2 * PI};
  };

  constexpr Direction() : value{0} {}

  constexpr explicit Direction(Rep radians) {
    value = radians;
    normalize();
  }

  static constexpr Direction<Rep> from_deg(Rep degrees) {
    return Direction<Rep>(degrees * Rep((2 * PI) / 360));
  }

  static constexpr Direction<Rep> from_rad(Rep radians) {
    return Direction<Rep>(radians);
  }

  static constexpr Direction<Rep> from_xy(Rep x, Rep y) {
    using std::atan2;
    return Direction<Rep>(atan2(y, x));
  }

  constexpr Rep deg() const {
    return value * Rep(360 / (2 * M_PI));
  }

  constexpr Rep rad() const {
    return value;
  }

  constexpr void normalize() {
    value = wrapf(value, Rep(0), Rep(2 * PI));
  }

  constexpr Direction<Rep>& operator+=(Angle<Rep> rhs) {
    value += rhs.value;
    normalize();
    return *this;
  }

  constexpr Direction<Rep>& operator-=(Angle<Rep> rhs) {
    value -= rhs.value;
    normalize();
    return *this;
  }

  constexpr Direction<Rep> operator+(Angle<Rep> rhs) const {
    Direction<Rep> ret(*this);
    ret += rhs;
    return ret;
  }

  constexpr Direction<Rep> operator-(Angle<Rep> rhs) const {
    Direction<Rep> ret(*this);
    ret -= rhs;
    return ret;
  }

  constexpr Angle<Rep> pi_to_pi() const {
    return Angle<Rep>(wrapf(value, Rep(-PI), Rep(PI)));
  }

  constexpr Angle<Rep> zero_to_2pi() const {
    return Angle<Rep>(value);
  }

  constexpr Angle<Rep> angle_to(Direction<Rep> dir) const {
    Angle<Rep> a = Angle<Rep>(dir.value - value).direction().zero_to_2pi();
    Angle<Rep> b{0};
    if(a > Angle<Rep>::turn() / 2) {
      b = a - Angle<Rep>::turn();
    } else {
      b = Angle<Rep>::turn() - a;
    }
    if(a.abs() < b.abs()) {
      return a;
    } else {
      return b;
    }
  }

  Rep value{0};
};

template<class Rep>
Rep sin(Angle<Rep> a) {
  using std::sin;
  return sin(a.rad());
}

template<class Rep>
Rep cos(Angle<Rep> a) {
  using std::cos;
  return cos(a.rad());
}

template<class Rep>
Rep sin(Direction<Rep> a) {
  using std::sin;
  return sin(a.rad());
}

template<class Rep>
Rep cos(Direction<Rep> a) {
  using std::cos;
  return cos(a.rad());
}

using Anglef = Angle<float>;
using Directionf = Direction<float>;

};  // namespace anglelib

#endif
