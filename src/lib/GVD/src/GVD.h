#ifndef GVD_H
#define GVD_H

#include <bits/stdc++.h>
#include <iostream>
#include "data/Pos.h"
#include "data/Grid.h"
#include "data/Graph.h"
#include "Map.h"
#include "DistMap.h"
#include <boost/graph/adjacency_list.hpp>

using namespace std;

/*
 *  critical info 
 */
struct critical_info{
  float mind_f; //min distance to frontier
  vector<Pos> frontiers;
};
typedef boost::unordered_map<Pos, critical_info> criticals_info;

/*
 *  GVD
 */
typedef Grid<bool> GridGvd;

using namespace boost;

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

typedef Graph<adjacency_list<listS, listS, bidirectionalS, GvdVertexProperty,property<edge_weight_t, float>>> GVD;
typedef Graph<adjacency_list<vecS, vecS, bidirectionalS, GvdVertexProperty,property<edge_weight_t, float>>> VecGVD;

/*
 *  functions
 */
boost::tuple<DistMap, DistPosQueue> calculate_distances(StateGrid, CellState sourceType);

boost::tuple<boost::unordered_map<Pos,Pos>, Pos> find_paths_to_gvd(StateGrid, VecGVD, Pos p_pos);

GridGvd get_grid_gvd(DistMap dg, DistPosQueue);

//boost::unordered_map<Pos, DistPos> get_critical_points(grid_type ogrid, DistMap dg, GVD& gvd);
// boost::unordered_map<Pos,bool> get_local_mins(DistMap dg, GVD gvd);

boost::tuple<criticals_info, GVD> get_points_of_interest(StateGrid g);

#endif
