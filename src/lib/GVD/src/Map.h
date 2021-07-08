#pragma once

#include "data/Grid.h"

template<typename T>
struct Map{
  typedef T CellType;

};

enum CellState { Occupied, Unknown, Free, Critical, Frontier };
typedef Grid<CellState> StateGrid;

typedef Map<CellState> StateMap;

