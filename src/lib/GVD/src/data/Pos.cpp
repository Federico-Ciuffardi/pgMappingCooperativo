#include "Pos.h"

bool same_direction(Pos p1, Pos p2) {
  float normalized_p1_x = p1.x / p1.length();
  float normalized_p1_y = p1.y / p1.length();

  float normalized_p2_x = p2.x / p2.length();
  float normalized_p2_y = p2.y / p2.length(); 

  bool same_x = normalized_p1_x == -normalized_p2_x;
  bool same_y = normalized_p1_y == -normalized_p2_y;

  return same_x && same_y;
}
