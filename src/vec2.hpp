#pragma once
#include <cmath>

// Минимальный 2D-вектор (double — чтобы ошибки округления не мешали анализу
// CCD)
struct Vec2 {
  double x = 0.0, y = 0.0;
  Vec2() = default;
  Vec2(double x_, double y_) : x(x_), y(y_) {}

  Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
  Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
  Vec2 operator*(double k) const { return {x * k, y * k}; }
  Vec2 operator/(double k) const { return {x / k, y / k}; }
  Vec2 operator-() const { return {-x, -y}; }
  Vec2& operator+=(const Vec2& o) {
    x += o.x;
    y += o.y;
    return *this;
  }
  Vec2& operator-=(const Vec2& o) {
    x -= o.x;
    y -= o.y;
    return *this;
  }
  Vec2& operator*=(double k) {
    x *= k;
    y *= k;
    return *this;
  }
};

inline double dot(const Vec2& a, const Vec2& b) {
  return a.x * b.x + a.y * b.y;
}
inline double len2(const Vec2& a) { return dot(a, a); }
inline double len(const Vec2& a) { return std::sqrt(dot(a, a)); }
inline Vec2 normalize(const Vec2& a) {
  double l = len(a);
  return l > 1e-12 ? a / l : Vec2(1.0, 0.0);
}
inline double clampd(double v, double lo, double hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}
