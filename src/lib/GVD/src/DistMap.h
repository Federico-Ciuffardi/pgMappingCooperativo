#pragma once

#include "utils.h"
#include "Map.h"

/*
 *  dist cell, col and grid
 */
struct DistMap{
  struct DistCell {
    vector<Pos> obs;
    float distance;

    DistCell(){};

    void add_obs(Pos);
    bool has_obs(Pos);

    bool operator>(const DistCell& d) const;
    bool operator==(const DistCell& p) const;
    friend ostream& operator<<(ostream& out, const DistCell& cell);
  };

  typedef Grid<DistCell> DistMapType;
  DistMapType distMap;

  DistMap(pair<Int,Int>);
  DistMapType::reference operator[](Pos p);
};

boost::tuple<DistMap, DistPosQueue> calculate_distances(StateGrid, CellState sourceType);

