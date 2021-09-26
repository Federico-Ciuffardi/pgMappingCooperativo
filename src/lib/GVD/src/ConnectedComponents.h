#pragma once

#include <unordered_map>
#include <utility>
#include "Map.h"
#include "data/Grid.h"

#define NULL_ID NULL_INT 

class ConnectedComponents{
public:
  typedef CellState      CellType; // could be set as template if needed
  typedef Grid<CellType> MapType; // could be set as template if needed

  typedef Int IdType;
  static const IdType firstId = 0;
  IdType lastId = 0;

  struct Component{
    PosSet members;
    boost::unordered_map<CellType, PosSet> typeMembers;
  };

  typedef std::map<IdType, Component> Components;

  // Results
  Components connectedComponents;
  Grid<IdType> idGrid;

  // Info
  MapType& map;

  // Config
  vector<CellType> nonTraversables;

  // Constructor
  ConnectedComponents(MapType& map, vector<CellType> nonTraversables);
    
  // functions
  void update(); // Non incremental

private:
  // Aux Funcs
  IdType genId();
  void add(Pos p, IdType);
  void fill(Pos p, IdType id);
};
