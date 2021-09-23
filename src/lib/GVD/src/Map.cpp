#include "Map.h"

CellState toOccupancyState(CellState cs){
  switch (cs) {
    case CriticalLine:
    case Critical: return Free;    
    /* case Frontier: */ 
    default:       return cs;
  }
}

ostream& operator<<(ostream& out, const CellState cs) {
  switch (cs) {
    case Occupied:     return out<<"██";
    case Unknown:      return out<<" ?";
    case Free:         return out<<"  ";
    case Critical:     return out<<" !";
    case Frontier:     return out<<" F";
    case CriticalLine: return out<<"--";
  }
  return out<<"Invalid CellState";
}


