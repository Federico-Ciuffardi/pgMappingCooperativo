#pragma once

#include <bits/stdc++.h>
#include <iostream>
#include "data/Pos.h"
#include "data/Grid.h"
#include "data/Graph.h"
#include "Map.h"
#include "DistMap.h"
#include <boost/graph/adjacency_list.hpp>

using namespace boost;

////////////
// Config //
////////////

// connectivityMethod: how connectivity is assured
// 0: unknown cell generate waves
// 1: unknown cells are considered unobstructed and map is surrounded with obstacles
// 2: unknown cells are considered unobstructed
static const int connectivityMethod = 0;

// GVD vertex simplification: how hard are the GVD vercices eroded/deleted to
// simplify or thin the GVD:
// 0: no simplification
// 1: the cells that are considered auxiliar [*] (for connectivity) are discarded
//    if they can be safely discarded without disconnecting the GVD.
// 2: All cells  will be discarded if they can be safely discarded without
//    disconnecting the GVD.
// [*] The cells that are considered auxiliar depends on the connectivity method used:
//       0: Cells that have less than 2 obstacles as basis points  are auxiliar
//       (are a products of unknown cells)
//       1 and 2: Cells that are unknown or are close to unknown
static const int vertexSimplificationMethod = 1;


// GVD edge simplification: how hard are the GVD edges deleted to simplify the GVD:
// 0: no simplification
// 1: edges to vertices that can be accessed through a neighbor of grater degree are deleted
// 2: edges to vertices that can be accessed through a neighbor of grater or equal degree are deleted
static const int edgeSimplificationMethod = 2;

// Allow GVD edge simplification to remove vertex: if when removing edges the verte
// 0: Do not allow vertex removal
// 1: Allow vertex removal
static const int edgeSimplificationAllowVertexRemoval = 1;

/////////////////
// Definitions //
/////////////////
bool connectivityAux(Pos p , StateGrid& sg, DistMap& distMap);

typedef Grid<bool> GridGvd;
struct GvdVertexProperty {
  Pos p = NULL_POS;

  bool isLocalMin = false;
  bool degreeConstrain = false;

  bool is_critical = false;

  Int segmentId = -1;

  Pos segment = NULL_POS;

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
  void update();

  // Destructor
  ~Gvd();
};
