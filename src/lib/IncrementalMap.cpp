#include "IncrementalMap.h"
#include "GVD/src/Map.h"
#include "conversion.h"
#include "nav_msgs/OccupancyGrid.h"

  void IncrementalMap::update(const OccupancyGridConstPtr& newOccupancyGrid){
  // Update occupancyGrid header and info
  occupancyGrid.header = newOccupancyGrid->header;
  occupancyGrid.info   = newOccupancyGrid->info;

  // create map if was not initialized
  if(map.size().first == 0){
    map = Map(make_pair(newOccupancyGrid->info.width, newOccupancyGrid->info.height),Unknown);
    occupancyGrid.data.resize(newOccupancyGrid->data.size());
  }

  // Update map and occupancyGrid.data
  for(int i = 0; i < newOccupancyGrid->data.size(); i++){
    Pos p = toPos(i,newOccupancyGrid->info.width);

    update(p,toCellType(newOccupancyGrid->data[i]));
    occupancyGrid.data[i] = newOccupancyGrid->data[i];
  }
}

void IncrementalMap::update(const OccupancyGridUpdateConstPtr& occGridUpdate){
  Pos updateToGlobal = Pos(occGridUpdate->x,occGridUpdate->y);
  for (int updateInd = 0; updateInd < occGridUpdate->data.size(); updateInd++) {
    int globalInd = toInt( updateToGlobal + toPos(updateInd,occGridUpdate->width), occupancyGrid.info.width);
    Pos globalPos = toPos(globalInd,occupancyGrid.info.width);

    update(globalPos,toCellType(occGridUpdate->data[updateInd]));
    occupancyGrid.data[globalInd] = occGridUpdate->data[updateInd];
  }
}

void IncrementalMap::update(Pos p, CellState newCellState) {
  CellState oldState = map[p];
  if(oldState == newCellState) return; // skip if the update does not change the current value

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
  if(is_elem(p,lastCellState)){
    if(lastCellState[p] == newCellState){
      lastCellState.erase(p);
    }
  }else{
     lastCellState[p] = oldState;
  }
  map[p] = newCellState;

  // if the current cell was unknown and the neighbor is a frontier try to set it to free
  // to check if it is still a frontier
  if( oldState ==  Unknown ){
    for(Pos pN : map.adj(p)){
      if(map[pN] == Frontier){
        update(pN,Free);
      }
    }
  }
}


