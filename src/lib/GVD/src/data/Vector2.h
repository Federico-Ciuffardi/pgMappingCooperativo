#pragma once

#include <ostream>
#include <cmath>
#include "Num.h"

#define Vector2_X Vector2( 1, 0, 0)
#define Vector2_Y Vector2( 0, 1, 0)

using namespace std;


struct Vector2 {
  typedef Int InNum;

  InNum x, y;

  Vector2();
  Vector2(InNum x, InNum y);

  Vector2 operator+(const Vector2) const;
  Vector2& operator+=(const Vector2);

  Vector2 operator-() const;
  Vector2 operator-(const Vector2) const;
  Vector2& operator-=(const Vector2);

  Vector2 operator*(const Float) const;

  Vector2 operator*(const Vector2) const;

  Vector2 cross(const Vector2) const; // cross-product

  Float dot(const Vector2) const;

  Float angle(Vector2 v);
  Float angle_to(const Vector2) const;

  Vector2 rotate(const Vector2 k, Float angle) const;

  Vector2 operator/(const Float) const;

  Float length() const;

  Float lengthSquared() const;

  Vector2 normalize() const;

  Vector2 symmetric(Vector2) const;

  Float distanceTo(Vector2) const;
  Float distanceToSquared(Vector2) const;

  bool operator==(const Vector2) const;
  bool operator!=(const Vector2) const;
  bool operator<(const Vector2) const;
  bool operator>(const Vector2) const;

  friend ostream& operator<<(ostream& out, const Vector2);
};

size_t hash_value(const Vector2 v);

ostream& operator<<(ostream& out, const Vector2);

Vector2 operator*(Float f, const Vector2);
