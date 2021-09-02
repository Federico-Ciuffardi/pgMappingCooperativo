#pragma once

#include <bits/stdc++.h>
#include <iostream>
#include "data/Pos.h"
#include "data/Grid.h"
#include "data/Graph.h"
#include "Map.h"
#include "DistMap.h"
#include <boost/graph/adjacency_list.hpp>
#include "GvdConfig.h"

using namespace boost;

/////////////////
// Definitions //
/////////////////
bool connectivityAux(Pos p , StateGrid& sg, DistMap& distMap);

typedef Grid<bool> GridGvd;
struct GvdVertexProperty {
  Pos p = NULL_POS;

  bool isLocalMin = false;
  bool degreeConstrain = false;

  GvdVertexProperty(){};

  GvdVertexProperty(Pos p) {
    this->p = p;
  }
};

typedef PosGraph<adjacency_list<listS, listS, bidirectionalS, GvdVertexProperty,property<edge_weight_t, Float>>> GvdGraph;
typedef PosGraph<adjacency_list<vecS, vecS, bidirectionalS, GvdVertexProperty,property<edge_weight_t, Float>>> GvdVecGraph;

struct Gvd {
  typedef CellState      CellType; // could be set as template if needed
  typedef Grid<CellType> MapType; // could be set as template if needed

  // Results
  GridGvd gridGvd;
  GvdGraph* graphGvd = NULL;
  DistMap* distMap = NULL;

  // Info
  MapType& map;

  // Constructors
  Gvd(DistMap*);
  Gvd(MapType&);

  // Functions
  bool isConnectivityAux(Pos p);

  void update();

  // Destructor
  ~Gvd();
};
