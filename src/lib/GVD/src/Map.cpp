#include "Map.h"

ostream& operator<<(ostream& out, const CellState cs) {
  switch (cs) {
    case Occupied: return out<<"██";
    case Unknown:  return out<<" ?";
    case Free:     return out<<"  ";
    case Critical: return out<<" !";
    case Frontier: return out<<" F";
  }
  return out<<"Invalid CellState";
}


