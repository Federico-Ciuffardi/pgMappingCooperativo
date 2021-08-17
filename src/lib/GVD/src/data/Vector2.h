#pragma once

#include <ostream>
#include <cmath>
#include "boost/functional/hash.hpp"
#include <boost/fusion/support/pair.hpp>
#include <boost/fusion/include/pair.hpp>
#include "Num.h"

#define Vector2_X Vector2( 1, 0, 0)
#define Vector2_Y Vector2( 0, 1, 0)

using namespace std;

template<typename innum>
struct Vector2 {

  typedef innum InNum;

  InNum x, y;

  Vector2() {
    this->x = 0;
    this->y = 0;
  }

  Vector2(InNum x, InNum y) {
    this->x = x;
    this->y = y;
  }

Vector2   operator+(const Vector2 v) const {
    return Vector2(x + v.x, y + v.y);
  }

Vector2&   operator+=(const Vector2 v) {
    *this = (*this + v);
    return *this;
  }

  Vector2   operator-() const{
    return Vector2() - *this;
  }
  Vector2   operator-(const Vector2 v) const {
    return Vector2(x - v.x, y - v.y);
  }
  Vector2&   operator-=(const Vector2 v) {
    *this = (*this - v);
    return *this;
  }

  Vector2   operator*(const Float f) const {
    return Vector2(f*x, f*y);
  }

  Vector2   operator*(const Vector2 v) const {
    return Vector2(x*v.x, y*v.y);
  }

  Vector2   cross(const Vector2 u) const {
    Vector2 v = *this;
    return Vector2(v.x*u.y, -v.y*u.x);
  }

  Float   dot(const Vector2 v) const {
    return x * v.x + y * v.y;
  }

  Float   angle(Vector2 v) {
    return atan2(v.y, v.x);
  }
  Float   angle_to(const Vector2 v) const{
    return acos(this->dot(v)/(this->length() * v.length()));
  }

  Vector2   operator/(const Float f) const {
    return Vector2(x/f, y/f);
  }

  Float   length() const{
    return sqrt(this->dot(*this));
  }

  Float   lengthSquared() const{
    return this->dot(*this);
  }

  Vector2   normalize() const{
    return this->length() == 0 ? Vector2() : *this/this->length();
  }

  Float   distanceTo(Vector2 v) const{
    return sqrt(this->distanceToSquared(v));
  }
  Float   distanceToSquared(Vector2 v) const{
    return ((*this) - v).lengthSquared();
  }
  bool   adjacent(Vector2 v) const{
    return this->distanceToSquared(v) <= 2;
  };

  bool   operator==(const Vector2 v) const {
    return (x == v.x) && (y == v.y);
  }
  bool   operator!=(const Vector2 v) const {
    return x != v.x || y != v.y;
  }

  bool   operator>(const Vector2 v) const {
    return (x > v.x) || (x == v.x && y > v.y);
  }
  bool   operator<(const Vector2 v) const {
    return (x < v.x) || (x == v.x && y < v.y);
  }

  template<typename InNum2>
  friend ostream& operator<<(ostream& out, const Vector2<InNum2>);

  template<typename InNum2>
  static Vector2<InNum> cast(Vector2<InNum2> in){
    Vector2<InNum> out; 
    out.x = in.x;
    out.y = in.y;
    return out;
  }

};

template<typename InNum>
inline size_t hash_value(const Vector2<InNum> v){
  return boost::hash_value(pair<InNum,InNum>(v.x,v.y));
}

template<typename InNum>
inline ostream& operator<<(ostream& out, const Vector2<InNum> v) {
  out << "(" << v.x << "," << v.y << ")";
  return out;
}

template<typename InNum>
inline Vector2<InNum> operator*(Float f, const Vector2<InNum> v) {
  return Vector2<InNum>(f*v.x, f*v.y);
}

