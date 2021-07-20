#include "Vector2.h"
#include "boost/functional/hash.hpp"
#include <boost/fusion/support/pair.hpp>
#include <boost/fusion/include/pair.hpp>

Vector2::Vector2() {
  this->x = 0;
  this->y = 0;
}

Vector2::Vector2(InNum x, InNum y) {
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

Vector2 Vector2::operator-() const{
  return Vector2() - *this;
}
Vector2 Vector2::operator-(const Vector2 v) const {
  return Vector2(x - v.x, y - v.y);
}
Vector2& Vector2::operator-=(const Vector2 v) {
  *this = (*this - v);
  return *this;
}

Vector2 Vector2::operator*(const Float f) const {
  return Vector2(f*x, f*y);
}
Vector2 operator*(Float f, const Vector2 v) {
  return Vector2(f*v.x, f*v.y);
}

Vector2 Vector2::operator*(const Vector2 v) const {
  return Vector2(x*v.x, y*v.y);
}

Vector2 Vector2::cross(const Vector2 u) const {
  Vector2 v = *this;
  return Vector2(v.x*u.y, -v.y*u.x);
}

Float Vector2::dot(const Vector2 v) const {
  return x * v.x + y * v.y;
}

Float Vector2::angle(Vector2 v) {
  return atan2(v.y, v.x);
}
Float Vector2::angle_to(const Vector2 v) const{
  return acos(this->dot(v)/(this->length() * v.length()));
}

Vector2 Vector2::operator/(const Float f) const {
  return Vector2(x/f, y/f);
}

Float Vector2::length() const{
  return sqrt(this->dot(*this));
}

Float Vector2::lengthSquared() const{
  return this->dot(*this);
}

Vector2 Vector2::normalize() const{
  return this->length() == 0 ? Vector2() : *this/this->length();
}

Float Vector2::distanceTo(Vector2 v) const{
  return sqrt(this->distanceToSquared(v));
}
Float Vector2::distanceToSquared(Vector2 v) const{
  return ((*this) - v).lengthSquared();
}

bool Vector2::operator==(const Vector2 v) const {
  return (x == v.x) && (y == v.y);
}
bool Vector2::operator!=(const Vector2 v) const {
  return x != v.x || y != v.y;
}

bool Vector2::operator>(const Vector2 v) const {
  return (x > v.x) || (x == v.x && y > v.y);
}
bool Vector2::operator<(const Vector2 v) const {
  return (x < v.x) || (x == v.x && y < v.y);
}

size_t hash_value(const Vector2 v){
  return boost::hash_value(pair<Vector2::InNum,Vector2::InNum>(v.x,v.y));
}

ostream& operator<<(ostream& out, const Vector2 v) {
  out << "(" << v.x << "," << v.y << ")";
  return out;
}
