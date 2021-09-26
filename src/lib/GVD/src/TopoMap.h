#pragma once

#include "DistMap.h"
#include "Gvd.h"
#include "data/Grid.h"
#include "ConnectedComponents.h"
#include <boost/graph/adjacency_list.hpp>

/*
 *  critical info 
 */
struct CriticalInfo{
  vector<Pos> frontiers;
  PosSet criticalLines;
};

typedef PosMap<CriticalInfo>::type CriticalInfos;

typedef ConnectedComponents::Component Segment;
typedef ConnectedComponents::Components Segments;

struct TopoMap{
  typedef CellState      CellType; // could be set as template if needed
  typedef Grid<CellType> MapType; // could be set as template if needed

  ///////////////
  // Variables //
  ///////////////

  // Results
  CriticalInfos criticalInfos;
  PosSet criticalLines;

  DistMap* distMap;
  Gvd* gvd;

  ConnectedComponents* segmenter;

  // Info
  MapType& map;

  ///////////////
  // Functions //
  ///////////////

  void update();
  void update(MapUpdatedCells &mapUpdatedCells);

  // Constructors
  TopoMap(Gvd*);
  TopoMap(MapType&);

  // Destructor
  ~TopoMap();

 private:
  //////////////
  // Funcions //
  //////////////

  void updateBase(PosSet &candidates);

};

