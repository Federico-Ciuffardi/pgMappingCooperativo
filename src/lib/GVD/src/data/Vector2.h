#pragma once

#include <ostream>
#include <cmath>

#define Vector2_X Vector2( 1, 0, 0)
#define Vector2_Y Vector2( 0, 1, 0)

typedef int Num;

using namespace std;

struct Vector2 {
  Num x, y;

  Vector2();
  Vector2(Num x, Num y);

  Vector2 operator+(const Vector2) const;
  Vector2& operator+=(const Vector2);

  Vector2 operator-(const Vector2) const;
  Vector2& operator-=(const Vector2);

  Vector2 operator*(const Num) const;

  Vector2 operator*(const Vector2) const;

  Vector2 cross(const Vector2) const; // cross-product

  Num dot(const Vector2) const;

  Num angle_to(const Vector2) const;

  Vector2 rotate(const Vector2 k, Num angle) const;

  Vector2 operator/(const Num) const;

  Vector2 operator-() const;

  Num length() const;

  Num length_squared() const;

  Vector2 normalize() const;

  Vector2 symmetric(Vector2) const;

  Num distance_to(Vector2) const;

  bool operator==(const Vector2) const;
  bool operator<(const Vector2) const;

  friend ostream& operator<<(ostream& out, const Vector2);
};

size_t hash_value(const Vector2 &v);

ostream& operator<<(ostream& out, const Vector2);

Vector2 operator*(Num f, const Vector2);
