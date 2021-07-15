#pragma once

#include <unordered_map>
#include <utility>
#include "Map.h"
#include "data/Grid.h"


struct ConnectedComponents{
  struct ConectedComponent{
    Int id;
    PosSet members;
    boost::unordered_map<CellState, PosSet> typeMembers;
    Pos center;
  };

  unordered_map<Int, ConectedComponent> connectedComponents;

  ConnectedComponents(pair<Int, Int>);




}

