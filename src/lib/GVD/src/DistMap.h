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

    PosSet crashingWaves;

    bool operator>(const DistCell& d) const;
    bool operator==(const DistCell& p) const;
    friend ostream& operator<<(ostream& out, const DistCell& cell);
  };

  typedef Grid<DistCell> DistMapType;
  typedef CellState CellType; 


  DistMapType distMap;
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
