#include "Vector2.h"
#include "boost/functional/hash.hpp"
#include <boost/fusion/support/pair.hpp>
#include <boost/fusion/include/pair.hpp>

Vector2::Vector2() {
  this->x = 0;
  this->y = 0;
}

Vector2::Vector2(Num x, Num y) {
  this->x = x;
  this->y = y;
}

Vector2 Vector2::operator+(const Vector2 v) const {
  return Vector2(x + v.x, y + v.y);
}

Vector2& Vector2::operator+=(const Vector2 v) {
  *this = (*this + v);
  return *this;
}

Vector2 Vector2::operator-(const Vector2 v) const {
  return Vector2(x - v.x, y - v.y);
}

Vector2& Vector2::operator-=(const Vector2 v) {
  *this = (*this - v);
  return *this;
}

Vector2 Vector2::operator*(const Num f) const {
  return Vector2(f*x, f*y);
}
Vector2 operator*(Num f, const Vector2 v) {
  return Vector2(f*v.x, f*v.y);
}

Vector2 Vector2::operator*(const Vector2 v) const {
  return Vector2(x*v.x, y*v.y);
}

Vector2 Vector2::cross(const Vector2 u) const {
  Vector2 v = *this;
  return Vector2(v.x*u.y, -v.y*u.x);
}

Num Vector2::dot(const Vector2 v) const {
  return x * v.x + y * v.y;
}

Num Vector2::angle_to(const Vector2 v) const{
  return acos(this->dot(v)/(this->length() * v.length()));
}

Vector2 Vector2::operator/(const Num f) const {
  return Vector2(x/f, y/f);
}

Vector2 Vector2::operator-() const{
  return Vector2() - *this;
}

Num Vector2::length() const{
  return sqrt(this->dot(*this));
}

Num Vector2::length_squared() const{
  return this->dot(*this);
}

Vector2 Vector2::normalize() const{
  return this->length() == 0 ? Vector2() : *this/this->length();
}

Num Vector2::distance_to(Vector2 v) const{
  return ((*this) - v).length();
}

bool Vector2::operator==(const Vector2 v) const {
  return (x == v.x) && (y == v.y);
}

bool Vector2::operator<(const Vector2 v) const {
  return (x < v.x) || (x == v.x && y < v.y);
}

size_t hash_value(const Vector2 &v){
  return boost::hash_value(pair<int,int>(v.x,v.y));
}

ostream& operator<<(ostream& out, const Vector2 v) {
  out << "(" << v.x << "," << v.y << "," << ")";
  return out;
}
