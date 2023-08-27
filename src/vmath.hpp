#ifndef VMATH_HPP
#define VMATH_HPP

#include <cmath>
#include <anglelib.hpp>

namespace vmath {

template<class T>
struct Vec2 {
  T x{0};
  T y{0};

  Vec2(T x_, T y_): x{x_}, y{y_} {}
  Vec2(): x{0}, y{0} {}

  T length2() {
    return x * x + y * y;
  }

  T length() {
    using std::sqrt;
    return sqrt(length2());
  }

  Vec2& normalize() {
    float len_inv = T(1) / length();
    x *= len_inv;
    y *= len_inv;
    return *this;
  }

  Vec2& operator+=(Vec2 rhs) {
    x += rhs.x;
    y += rhs.y;
    return *this;
  }

  Vec2& operator-=(Vec2 rhs) {
    x -= rhs.x;
    y -= rhs.y;
    return *this;
  }

  Vec2& operator*=(float rhs) {
    x *= rhs;
    y *= rhs;
    return *this;
  }

  Vec2& operator/=(float rhs) {
    x /= rhs;
    y /= rhs;
    return *this;
  }

  Vec2 operator+(Vec2 rhs) const {
    Vec2 result{*this};
    result += rhs;
    return result;
  }

  Vec2 operator-(Vec2 rhs) const {
    Vec2 result{*this};
    result -= rhs;
    return result;
  }

  Vec2 operator*(float rhs) const {
    Vec2 result{*this};
    result *= rhs;
    return result;
  }

  Vec2 operator/(float rhs) const {
    Vec2 result{*this};
    result /= rhs;
    return result;
  }

  Vec2 normalized() const {
    Vec2 result{*this};
    result.normalize();
    return result;
  }

  Vec2 rotated(anglelib::Angle<T> angle) const {
    const T s = anglelib::sin(angle);
    const T c = anglelib::cos(angle);
    return Vec2(
      x * c + y * -s,
      x * s + y * c
    );
  }

  float dot(Vec2 rhs) {
    return x * rhs.x + y * rhs.y;
  }
};

template<class T>
struct Vec3 {
  T x{0};
  T y{0};
  T z{0};

  Vec3(T x_, T y_, T z_): x{x_}, y{y_}, z{z_} {}
  Vec3(): x{0}, y{0}, z{0} {}

  T length2() {
    return x * x + y * y + z * z;
  }

  T length() {
    using std::sqrt;
    return sqrt(length2());
  }

  Vec3& operator+=(Vec3 rhs) {
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
    return *this;
  }

  Vec3& operator-=(Vec3 rhs) {
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;
    return *this;
  }

  Vec3& operator*=(float rhs) {
    x *= rhs;
    y *= rhs;
    z *= rhs;
    return *this;
  }

  Vec3& operator/=(float rhs) {
    x /= rhs;
    y /= rhs;
    z /= rhs;
    return *this;
  }

  Vec3 operator+(Vec3 rhs) const {
    Vec3 result{*this};
    result += rhs;
    return result;
  }

  Vec3 operator-(Vec3 rhs) const {
    Vec3 result{*this};
    result -= rhs;
    return result;
  }

  Vec3 operator*(float rhs) const {
    Vec3 result{*this};
    result *= rhs;
    return result;
  }

  Vec3 operator/(float rhs) const {
    Vec3 result{*this};
    result /= rhs;
    return result;
  }

  float dot(Vec3 rhs) const {
    return Vec3(x * rhs.x + y * rhs.y + z * rhs.z);
  }

  Vec3 cross(Vec3 rhs) const {
    return Vec3{
      y*rhs.z - z*rhs.y,
      z*rhs.x - x*rhs.z,
      x*rhs.y - y*rhs.x,
    };
  }
};

using Vec2f = Vec2<float>;
using Vec2d = Vec2<double>;

using Vec3f = Vec3<float>;
using Vec3d = Vec3<double>;

}

#endif
