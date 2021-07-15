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
  DistMap* distMap;
  Gvd* gvd;
  CriticalInfos cis;

  void update(StateGrid& g);
  TopoMap(Gvd* gvd);
  TopoMap(pair<Int, Int> size);
  ~TopoMap();
};

