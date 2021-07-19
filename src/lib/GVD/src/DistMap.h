#pragma once

#include "utils.h"
#include "Map.h"
#include "data/Num.h"

/*
 *  dist cell, col and grid
 */
struct DistMap{
  typedef CellState      CellType; // could be set as template if needed
  typedef Grid<CellType> MapType; // could be set as template if needed

  struct DistCell {
    PosSet sources;
    Float distance = INF;

    PosSet crashingWaves;

    bool operator>(const DistCell& d) const;
    bool operator==(const DistCell& p) const;
    friend ostream& operator<<(ostream& out, const DistCell& cell);
  };

  typedef Grid<DistCell> DistMapType;

  // Results
  DistMapType distMap;
  DistPosQueue objectiveDQueue;
  PosSet waveCrashPoss;

  // Info
  MapType& map;

  // Configuration
  vector<CellType> nonTraversables;
  vector<CellType> sources;
  vector<CellType> objectives;

  // Constructor
  DistMap(MapType&, vector<CellType> sources, vector<CellType> nonTraversables, vector<CellType> objectives = {});

  // functions
  pair<Int,Int> size();
  DistMapType::reference operator[](Pos);
  void update();

  friend ostream& operator<<(ostream& out, const DistMap&);
};

// util

// Could be modified to accept a arbitrary CellType value if needed due to DistMap templating
inline bool isObstacleGenerated(Pos p, DistMap& distMap, StateGrid& sg){ 
  for(Pos source : distMap[p].sources){
    if(sg[source] != Occupied) return false; 
  }
  /* for(Pos crashing : distMap[p].crashingWaves){ */
  /*   for(Pos source : distMap[crashing].sources){ */
  /*     if(sg[source] != Occupied) return false; */ 
  /*   } */
  /* } */
  int obstacleWaves = 0;
  for(Pos crashing : distMap[p].crashingWaves){
    bool obstacleWave;
    for(Pos source : distMap[crashing].sources){
      obstacleWave = sg[source] == Occupied;
      if(!obstacleWave) break;
    }
    if(obstacleWave) obstacleWaves++;
    if(obstacleWaves >= 1) break;
  }
  return (obstacleWaves >= 1);
}
