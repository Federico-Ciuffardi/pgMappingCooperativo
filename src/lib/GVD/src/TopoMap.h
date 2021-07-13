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

typedef boost::unordered_map<Pos, CriticalInfo> CriticalInfos;

struct TopoMap{
  DistMap* distMap;
  Gvd* gvd;
  CriticalInfos cis;

  void update(StateGrid& g);
  TopoMap(Gvd* gvd);
  TopoMap(pair<Int, Int> size);
  ~TopoMap();
};

