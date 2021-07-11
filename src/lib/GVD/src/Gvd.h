#pragma once

#include <bits/stdc++.h>
#include <iostream>
#include "data/Pos.h"
#include "data/Grid.h"
#include "data/Graph.h"
#include "Map.h"
#include "DistMap.h"
#include "TopoMap.h"
#include <boost/graph/adjacency_list.hpp>

using namespace boost;

/*
 *  GvdGraph definition
 */
typedef Grid<bool> GridGvd;

struct GvdVertexProperty {
  Pos p;
  bool is_critical;
  Pos segment;
  GvdVertexProperty(){};
  GvdVertexProperty(Pos p, bool is_critical, Pos segment) {
    this->p = p;
    this->is_critical = is_critical;
    this->segment = Pos(-1,-1);
  }
  GvdVertexProperty(Pos p){
    this->p = p;
    this->is_critical = false;
    this->segment = Pos(-1, -1);
  };
};

typedef PosGraph<adjacency_list<listS, listS, bidirectionalS, GvdVertexProperty,property<edge_weight_t, float>>> GvdGraph;
typedef PosGraph<adjacency_list<vecS, vecS, bidirectionalS, GvdVertexProperty,property<edge_weight_t, float>>> GvdVecGraph;

class Gvd {
  void update(StateGrid g);
};

/*
 *  functions
 */
GridGvd get_grid_gvd(DistMap dg);

//boost::unordered_map<Pos, DistPos> get_critical_points(grid_type ogrid, DistMap dg, GvdGraph& gvd);
// boost::unordered_map<Pos,bool> get_local_mins(DistMap dg, GvdGraph gvd);

boost::tuple<criticals_info, GvdGraph> get_points_of_interest(StateGrid g);
