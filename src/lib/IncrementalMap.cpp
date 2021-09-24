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
  updateMap(updatedCells, map, p, newCellState);
}


