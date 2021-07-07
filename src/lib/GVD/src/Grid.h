#pragma once

#include <bits/stdc++.h>
#include <iostream>

#include <math.h>
#include <cfloat>

#include "data/Pos.h"

using namespace std;

template<typename CellType>
struct Grid{
  typedef vector<CellType> ColType;
  typedef vector<ColType>  GridType;
  GridType grid;

  // diplacement to possible neighbors (defaults to 8-connected)
  vector<Pos> neighborDisplacement = {Pos(-1, -1), Pos(-1, 0), Pos(-1, 1), Pos(0, 1),
                                      Pos(1, 1),   Pos(1, 0),  Pos(1, -1), Pos(0, -1)};

  // returns grid size
  pair<Int, Int> size() {
    return pair<Int, Int>(grid.size(), grid[0].size());
  }

  // true if Pos is inside the grid bounds
  bool inside(Int x, Int y) {
    pair<Int, Int> size = this->size();
    return ((x >= 0 && x < size.first) && (y >= 0 && y < size.second));
  }

  // true if Pos is inside the grid bounds
  bool inside(Pos p) {
    return inside(p.x,p.y);
  }

  // returns the cell corresponding to x, y
  typename ColType::reference cell(Int x, Int y) {
    return grid[x][y];
  }
  // returns the cell corresponding to Pos p
  typename ColType::reference cell(Pos p) {
    return grid[p.x][p.y];
  }

  // returns all *valid* neighbors Pos of a given Pos p in the grid
  PosSet adj(Pos p, vector<CellType> invalids = vector<CellType>()) {
    PosSet adj;
    for (auto it = neighborDisplacement.begin(); it != neighborDisplacement.end(); it++) {
      Pos n = p + (*it);
      if (inside(n)) {
        bool valid = true;
        for(auto it : invalids.begin()){
          valid = cell(n) != *it;
        }
        if (valid) {
          adj.insert(n);
        }
      }
    }
    return adj;
  }
};
