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

void updateMap(boost::unordered_map<Pos,CellState> &updatedCells,Map &map, Pos p, CellState newCellState) {
  if((GvdConfig::get()->connectivityMethod == 1 || GvdConfig::get()->connectivityMethod == 2) &&
      (!map.inside(p+Pos(1,1)) || !map.inside(p+Pos(-1,-1)))){ // the cell is on the border of the map
    newCellState = Occupied;
  }

  CellState oldState = toOccupancyState(map[p]);
  newCellState = toOccupancyState(newCellState);
  // skip if the update does not change the current value or if the new value is unknown (this should not be taken into account)
  if( oldState == newCellState ) return; 

  // if the cell is free and has an unknown neighbor then it is actually a frontier
  if (newCellState == Free) {
    bool hasUnknwonNeighbor = false;
    for(Pos pN : map.adj(p,{Occupied})){
      hasUnknwonNeighbor = map[pN] == Unknown;
      if(hasUnknwonNeighbor) break;
    }
    if(hasUnknwonNeighbor){
      newCellState = Frontier;
    }
  }
  if(oldState == newCellState) return; // skip if the update does not change the current value (frontier -> free -> frontier)

  // Make the update effective
  if(is_elem(p,updatedCells)){
    if(updatedCells[p] == newCellState){
      updatedCells.erase(p);
    }
  }else{
     updatedCells[p] = oldState;
  }
  map[p] = newCellState;

  // if the current cell was unknown and the neighbor is a frontier try to set it to free
  // to check if it is still a frontier
  if( oldState ==  Unknown ){
    for(Pos pN : map.adj(p)){
      if(map[pN] == Frontier){
        updateMap(updatedCells,map,pN,Free);
      }
    }
  }
}
