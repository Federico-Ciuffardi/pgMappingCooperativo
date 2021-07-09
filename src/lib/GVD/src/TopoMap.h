#pragma once

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


template<typename T>
struct TopoMap{
};

