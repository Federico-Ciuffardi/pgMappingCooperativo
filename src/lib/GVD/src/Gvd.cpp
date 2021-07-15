#include "Gvd.h"

#include <math.h>
#include <cfloat>
#include <utility>
#include "DistMap.h"
#include "Map.h"
#include "data/Pos.h"

/*
 *  main funcs
 */

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

/* returns a boolean matrix, a cell is true if it belongs to the GvdGraph and false
 * otherwise*/
GridGvd getGridGvd(DistMap& distMap, StateGrid sg) {
  // Initialize grid GVD
  GridGvd gridGvd(distMap.size(), false);
  GridGvd obstGridGvd(distMap.size(), false);

  // Set the candidates to belong to the GVD
  DistPosQueue gvdCandidatesQueue;
  for (Pos p : distMap.waveCrashPoss){
    gvdCandidatesQueue.push(make_pair(distMap[p].distance, p));

    gridGvd[p] = true;

    obstGridGvd[p] = isObstacleGenerated(p,distMap);
  }

  // Discard unnecesary candidates
  while (!gvdCandidatesQueue.empty()) {
    Float d = gvdCandidatesQueue.top().first;
    Pos p = gvdCandidatesQueue.top().second;
    gvdCandidatesQueue.pop();

    // all  
    /* gridGvd[p] = true; */
    // Preserve shape 
    gridGvd[p] =  (obstGridGvd[p] && distMap[p].sources.size() > 1 && d <= 2) || disconnectsOnRemoval(p, gridGvd);
    // Max celanup
    /* gridGvd[p] = disconnectsOnRemoval(p, gridGvd); */ 
  }
  return gridGvd;
}

Gvd::Gvd(DistMap* distMap){
  this->distMap = distMap;
}

Gvd::Gvd(pair<Int, Int> size){
  this->distMap = new DistMap(size,{Occupied,Unknown},{Occupied,Unknown});
}

void Gvd::update(StateGrid& sg){
  cout << "debug :: Update distMap" << endl;
  distMap->update(sg);
  cout << "debug :: Generate gridGVD" << endl;
  gridGvd = getGridGvd(*distMap, sg);
  cout << "debug :: Generate graphGVD" << endl;
  if(graphGvd) delete graphGvd;
  graphGvd = new GvdGraph(gridGvd);
}

Gvd::~Gvd(){
  delete distMap;
}
