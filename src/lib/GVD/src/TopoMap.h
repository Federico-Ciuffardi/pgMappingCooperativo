#pragma once

#include "DistMap.h"
#include "Gvd.h"
#include "data/Grid.h"
#include <boost/graph/adjacency_list.hpp>

/*
 *  critical info 
 */
struct critical_info{
  float mind_f; //min distance to frontier
  vector<Pos> frontiers;
};

typedef boost::unordered_map<Pos, critical_info> criticals_info;

struct TopoMap{
  DistMap* distMap;
  Gvd* gvd;
  criticals_info cis;

  void update(StateGrid& g);
  TopoMap(Gvd* gvd);
  TopoMap(pair<Int, Int> size);
  ~TopoMap();
};

