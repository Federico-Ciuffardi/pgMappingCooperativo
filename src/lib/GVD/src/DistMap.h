#pragma once

#include "utils.h"
#include "Map.h"

/*
 *  dist cell, col and grid
 */
struct DistMap{
  struct DistCell {
    vector<Pos> obs;
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
  DistPosQueue fullDQueue;

  DistMapType::reference operator[](Pos p);

  vector<CellType> nonTraversables;
  vector<CellType> sources;

  DistMap(pair<Int,Int>, vector<CellType> sourceStates, vector<CellType> nonTraversables);
  void update(StateGrid);

  friend ostream& operator<<(ostream& out, const DistMap& cell);
};

/* boost::tuple<DistMap, DistPosQueue> calculate_distances(StateGrid, CellState sourceType); */

