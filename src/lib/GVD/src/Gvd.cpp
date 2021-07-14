#include "Gvd.h"

#include <math.h>
#include <cfloat>
#include <utility>
#include "DistMap.h"

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
GridGvd getGridGvd(DistMap& distMap) {
  // initialize GridGvd
  GridGvd gridGvd(distMap.size());
  for (Pos p : gridGvd) {
    gridGvd[p] = distMap[p].distance != 0 && distMap[p].distance != inf;  // is not a wall or unknown
  }

  // compute grid GvdGraph
  while (!distMap.fullDQueue.empty()) {
    // get the x cell to process
    Pos p = distMap.fullDQueue.top().second;
    distMap.fullDQueue.pop();

    // Remove from GvdGraph if it does not belongs to the GvdGraph by definition ands does
    // not disconects the GvdGraph
    if (distMap[p].parents.size() <= 1 && !disconnectsOnRemoval(p, gridGvd)) {
      gridGvd[p] = false;
    }
  }
  return gridGvd;
}

Gvd::Gvd(DistMap* distMap){
  this->distMap = distMap;
}

Gvd::Gvd(pair<Int, Int> size){
  this->distMap = new DistMap(size,{Occupied},{Occupied,Unknown});
}

void Gvd::update(StateGrid& sg){
  cout << "debug :: Update distMap" << endl;
  distMap->update(sg);
  cout << "debug :: Generate gridGVD" << endl;
  gridGvd = getGridGvd(*distMap);
  cout << "debug :: Generate graphGVD" << endl;
  if(graphGvd) delete graphGvd;
  graphGvd = new GvdGraph(gridGvd);
}

Gvd::~Gvd(){
  delete distMap;
}
