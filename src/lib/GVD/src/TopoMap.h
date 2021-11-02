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
  // config
  Float minCriticalLineAngle = M_PI*0.75; //M_PI/1.5;

  // Results
  CriticalInfos criticalInfos;
  boost::unordered_map<Pos,int> criticalLines;

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

  bool minAngleConstrain(DistMap& distMap, Pos p, boost::unordered_map<Pos,pair<Pos,Pos>> &criticalLine);

  void updateBase(PosSet &candidates);

};

