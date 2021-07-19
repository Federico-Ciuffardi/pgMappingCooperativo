#pragma once

#include "DistMap.h"
#include "Gvd.h"
#include "data/Grid.h"
#include <boost/graph/adjacency_list.hpp>

/*
 *  critical info 
 */
struct CriticalInfo{
  float mindToF; //min distance to frontier
  vector<Pos> frontiers;
};

typedef PosMap<CriticalInfo>::type CriticalInfos;

struct Segment{
  Int id;
  PosSet members;
  PosMap<PosSet>::type typeMembers;
  Pos center;
};

typedef boost::unordered_map<Int, Segment> segments;

struct TopoMap{
  typedef CellState      CellType; // could be set as template if needed
  typedef Grid<CellType> MapType; // could be set as template if needed

  // Results
  DistMap* distMap;
  Gvd* gvd;
  CriticalInfos cis;

  // Constructors
  TopoMap(Gvd*);
  TopoMap(pair<Int, Int> size);

  // Functions
  void update(MapType&);

  // Destructor
  ~TopoMap();
};

