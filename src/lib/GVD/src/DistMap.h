#pragma once

#include "utils.h"
#include "Map.h"

/*
 *  dist cell, col and grid
 */
struct DistMap{
  struct DistCell {
    PosSet sources;
    Float distance = inf;

    void addSource(Pos);
    bool hasSource(Pos);

    bool operator>(const DistCell& d) const;
    bool operator==(const DistCell& p) const;
    friend ostream& operator<<(ostream& out, const DistCell& cell);
  };

  typedef Grid<DistCell> DistMapType;
  typedef CellState CellType; 


  DistMapType distMap;
  StateGrid grid;
  DistPosQueue objectiveDQueue;
  PosSet waveCrashPoss;

  DistMapType::reference operator[](Pos p);

  vector<CellType> nonTraversables;
  vector<CellType> sources;
  vector<CellType> objectives;

  DistMap(pair<Int,Int>, vector<CellType> sources, vector<CellType> nonTraversables, vector<CellType> objectives = {});
  void update(StateGrid&);

  pair<Int,Int> size();

  friend ostream& operator<<(ostream& out, const DistMap& cell);
};

/* boost::tuple<DistMap, DistPosQueue> calculate_distances(StateGrid, CellState sourceType); */

// util
inline bool isObstacleGenerated(Pos p, DistMap distMap){
  bool obstacleGenerated = true;
  for(Pos source : distMap[p].sources){
    obstacleGenerated = distMap.grid[source] == Occupied;
    if(!obstacleGenerated) break; 
  }
  return obstacleGenerated;
}
