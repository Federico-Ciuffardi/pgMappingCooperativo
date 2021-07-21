#pragma once

#include "utils.h"
#include "Map.h"
#include "data/Num.h"

struct DistMap{
  typedef CellState      CellType; // could be set as template if needed
  typedef Grid<CellType> MapType; // could be set as template if needed

  struct DistCell {
    PosSet sources;
    Float distance = INF;

    PosSet pseudoSources;

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
inline PosSet basisPoints(Pos p, DistMap& distMap){
  PosSet res;

  // sources U pseudoSources
  res.insert(distMap[p].sources.begin(), distMap[p].sources.end());
  res.insert(distMap[p].pseudoSources.begin(), distMap[p].pseudoSources.end());

  return res;
}

