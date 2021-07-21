#include "Gvd.h"

#include <math.h>
#include <cfloat>
#include <utility>
#include "DistMap.h"
#include "Map.h"
#include "data/Pos.h"

// Gvd class utils functions

/* If removing `p` from  disconects the graph represented with the Grid then  */
template<typename CellType>
int disconnectsOnRemoval(Pos p, Grid<CellType>& gridGraph) {
  vector<Pos> neighbor = gridGraph.neighborDisplacement;

  int res = 0;

  for (int i = 0; i < 8; i++) {
    Pos pN1 = p + neighbor[i];
    bool v1 = gridGraph.inside(pN1) && gridGraph[pN1];

    Pos pN2 =  p + neighbor[(i + 1) % 8];
    bool v2 = gridGraph.inside(pN2) && gridGraph[pN2];

    // if non diagonal
    if (neighbor[i].x == 0 || neighbor[i].y == 0) {
      Pos pN3 =   p + neighbor[(i + 2) % 8];
      bool v3 = gridGraph.inside(pN3) && gridGraph[pN3];
      res += v1 && !v2 && !v3;
    } else {
      res += v1 && !v2;
    }

    if (res > 1) break;
  }
  return res > 1;
}

// p has at least 2 obstacles as basis points  
bool isObstacleGenerated(Pos p, DistMap& distMap, StateGrid& sg){ 
  int obstacleBasis = 0;
  for(Pos bp : basisPoints(p, distMap)){
    obstacleBasis += sg[bp] == Occupied;
    if(obstacleBasis > 1) return true;
  }
  return false;
}

// filter condition specific to the connectivity method applied
bool necesary(Pos p , StateGrid& sg, DistMap& distMap){
  switch (connectivityMethod) {
    case 0:
      return isObstacleGenerated(p,distMap,sg);
      break;
    case 1:
    case 2:
      // p is not unknown or a neighbor of unknown
      if (sg[p] == Unknown) return false;
      for (Pos pn : sg.adj(p)){
        if (sg[pn] == Unknown) return false;
      } 
      return true;
      break;
  }
  return true;
}

// returns a boolean matrix, a cell is true if it should belong to the GvdGraph
// and false otherwise
GridGvd getGridGvd(DistMap& distMap, StateGrid& sg) {
  // Initialize grid GVD
  GridGvd gridGvd(distMap.size(), false);
  GridGvd obstGridGvd(distMap.size(), false);

  // Set the candidates to belong to the GVD
  DistPosQueue gvdCandidatesQueue;
  for (Pos p : distMap.waveCrashPoss){
    gvdCandidatesQueue.push(make_pair(distMap[p].distance, p));
    gridGvd[p] = true;
  }

  // Simplify GVD
  while (!gvdCandidatesQueue.empty() && simplificationMethod>0) {
    Float d = gvdCandidatesQueue.top().first;
    Pos p = gvdCandidatesQueue.top().second;
    gvdCandidatesQueue.pop();

    switch (simplificationMethod) {
      /* case 0: */
      /*   gridGvd[p] = true; */
      /*   break; */
      case 1:
        gridGvd[p] =  (necesary(p,sg,distMap) && distMap[p].sources.size() > 1 && d <= 2) 
                   || disconnectsOnRemoval(p, gridGvd);
        break;
      case 2:
        gridGvd[p] = disconnectsOnRemoval(p, gridGvd); 
        break;
    }
  }
  return gridGvd;
}

// Gvd class definition
Gvd::Gvd(DistMap* distMap) : map(distMap->map){
  this->distMap = distMap;
}

Gvd::Gvd(MapType& map) : map(map){
  switch (connectivityMethod) {
    case 0:
      this->distMap = new DistMap(map,{Occupied,Unknown},{Occupied,Unknown});
      break;
    case 1:
    case 2:
      this->distMap = new DistMap(map,{Occupied},{Occupied});
      break;
  }
}

void Gvd::update(){
  // if connectivityMethod is 1 fill boders with obstacles
  if(connectivityMethod == 1){
    pair<Int,Int> size = map.size();
    for(int x = 0; x < size.first; x++){
      map[x][0] = Occupied;
      map[x][size.second - 1] = Occupied;
    }
    for(int y = 0; y < size.second; y++){
      map[0][y] = Occupied;
      map[size.first - 1][y] = Occupied;
    }
  }

  // Clean old result
  if(graphGvd) delete graphGvd;

  // Get new result / replace old
  cout << "debug :: Update distMap" << endl;
  distMap->update();
  cout << "debug :: Generate gridGVD" << endl;
  gridGvd = getGridGvd(*distMap, map);
  cout << "debug :: Generate graphGVD" << endl;
  graphGvd = new GvdGraph(gridGvd);
}

Gvd::~Gvd(){
  delete distMap;
}
