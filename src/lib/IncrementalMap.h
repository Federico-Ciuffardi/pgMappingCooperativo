#pragma once

#include "GVD/src/data/Pos.h"
#include "GVD/src/Map.h"
#include "nav_msgs/OccupancyGrid.h"
#include "map_msgs/OccupancyGridUpdate.h"

using namespace nav_msgs;
using namespace map_msgs;

class IncrementalMap{
 private:
  //////////
  // vars //
  //////////
  
  ///////////////
  // Functions //
  ///////////////

 public:
  //////////
  // vars //
  //////////

  boost::unordered_set<int> coveredIndices;

  nav_msgs::OccupancyGrid occupancyGrid;
  
  Map map;

  MapUpdatedCells updatedCells; // cell -> last cell

  boost::unordered_map<Pos, int> knowNeighbors;

  ///////////////
  // Functions //
  ///////////////

  void update(Pos p, CellState cellState);
  void update(const OccupancyGridConstPtr& newOccupancyGrid);
  void update(const OccupancyGridUpdateConstPtr& update);
};
