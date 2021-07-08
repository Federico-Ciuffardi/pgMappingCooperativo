#include "DistMap.h"

DistMap::DistMap(pair<Int,Int> size){
 distMap = DistMapType(size); 
}


void DistMap::DistCell::add_obs(Pos p) {
  obs.push_back(p);
}

bool DistMap::DistCell::has_obs(Pos p) {
  return find(obs.begin(), obs.end(), p) != obs.end();
}

bool DistMap::DistCell::operator>(const DistMap::DistCell& d) const {
  return this->distance > d.distance;
}

bool DistMap::DistCell::operator==(const DistMap::DistCell& d) const {
  return this->distance == d.distance;
}

ostream& operator<<(ostream& out, const DistMap::DistCell& cell) {
  // out<<"("<< cell.distance<<", "<< cell.obs.size() << " )";
  if (cell.distance != 0) {
    out << cell.obs.size();
  } else {
    out << "=";
  }
  return out;
}

DistMap::DistCell& DistMap::operator[](Pos p){
  return distMap.cell(p);
}
