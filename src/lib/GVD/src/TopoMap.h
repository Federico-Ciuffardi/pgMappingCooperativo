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

struct Segment{
  Int id;
  PosSet members;
  PosMap<PosSet>::type typeMembers;
  Pos center;
};

typedef boost::unordered_map<Int, Segment> segments;

struct TopoMap{
  typedef CellState      CellType; // could be set as template if needed
  typedef Grid<CellType> MapType; // could be set as template if needed

  // Results
  CriticalInfos criticalInfos;
  
  // Sub results
  DistMap* distMap;
  Gvd* gvd;
  ConnectedComponents* segmenter;

  // Info
  MapType& map;

  // Constructors
  TopoMap(Gvd*);
  TopoMap(MapType&);

  // Functions
  void update();

  // Destructor
  ~TopoMap();

 private:
  // internal functions
  void setCriticals(StateGrid& stateGrid, GvdGraph& gvd, DistMap& distMap);
  void get_critical_points(StateGrid& stateGrid, DistMap& distMap, GvdGraph& gvd, ConnectedComponents& segmenter);

};

