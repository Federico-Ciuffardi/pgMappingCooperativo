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

bool connectivityAux(Pos p , Map& map, DistMap& distMap);
  
typedef Grid<bool> GridGvd;
struct GvdVertexProperty {
  Pos p = NULL_POS;

  bool isLocalMin = false;

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

  ///////////////
  // variables //
  ///////////////

  // Configuration
  vector<CellType> nonTraversables;
  vector<CellType> sources;

  // Results
  GridGvd gridGvd;
  GvdGraph graphGvd;
  DistMap* distMap = NULL;
  float updateTime = 0;

  // Info
  MapType& map;

  //////////////
  // Funcions //
  //////////////

  // Constructors
  Gvd(MapType&);

  // Functions
  bool isConnectivityAux(Pos p);
  int neighbors(Pos p);

  void update();
  void update(MapUpdatedCells &mapUpdatedCells);

  // Destructor
  ~Gvd();

 private:
  //////////////
  // Funcions //
  //////////////

  void updateBase(PosSet &candidates);
  void cleanUp(Pos p, GvdGraph &graph, Int simplification, Int vertexRemoval);


};
