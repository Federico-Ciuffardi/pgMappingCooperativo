#pragma once

#include <unordered_map>
#include <utility>
#include "Map.h"
#include "data/Grid.h"

struct ConnectedComponents{
  typedef CellState      CellType; // could be set as template if needed
  typedef Grid<CellType> MapType; // could be set as template if needed

  typedef Int IdType;

  struct ConectedComponent{
    IdType id;
    PosSet members;
    boost::unordered_map<CellType, PosSet> typeMembers;
    Pos center;
  };

  unordered_map<Int, ConectedComponent> connectedComponents;

  Grid<IdType> idGrid;

  ConnectedComponents(pair<Int, Int> size, vector<CellType> nonTraversable);
    
  void update(MapType);
};

