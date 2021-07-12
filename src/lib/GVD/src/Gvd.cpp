#include "Gvd.h"

#include <math.h>
#include <cfloat>
#include <utility>
#include "DistMap.h"

/*
 *  main funcs
 */

/* given a GraphType for the neighbors of a Pos return the number of conex components if Pos would be
 * removed */
int A(Pos p, GridGvd ggvd) {
  int res = 0;
  bool prev_np = 0;
  vector<Pos> neighbor = ggvd.neighborDisplacement;
  for (int i = 0; i < 8; i++) {
    bool v1 = ggvd.cell( p + neighbor[i]);
    bool v2 = ggvd.cell( p + neighbor[(i + 1) % 8]);
    if (neighbor[i].x == 0 || neighbor[i].y == 0) {
      bool v3 = ggvd.cell( p + neighbor[(i + 2) % 8]);
      res += v1 && !v2 && !v3;
    } else {
      res += v1 && !v2;
    }
    if (res > 1)
      break;
  }
  return res;
}

/* returns a boolean matrix, a cell is true if it belongs to the GvdGraph and false
 * otherwise*/
GridGvd getGridGvd(DistMap dg) {
  // get sizes
  pair<Int,Int> size = dg.distMap.size();

  // initialize GridGvd
  GridGvd GridGvd;
  for (int i = 0; i < size.first; i++) {
    GridGvd.grid.push_back(vector<bool>());
    for (int j = 0; j < size.second; j++) {
      GridGvd.grid[i].push_back(dg[Pos(i,j)].distance != 0);  // is not a wall or unknown
    }
  }

  // compute grid GvdGraph
  while (!dg.fullDQueue.empty()) {
    // get the x cell to process
    Pos current_Pos = dg.fullDQueue.top().second;
    dg.fullDQueue.pop();

    int cx = current_Pos.x;
    int cy = current_Pos.y;

    // Remove from GvdGraph if it does not belongs to the GvdGraph by definition ands does
    // not disconects the GvdGraph
    if (dg[Pos(cx,cy)].obs.size() <= 1 && A(current_Pos, GridGvd) <= 1) {
      GridGvd.cell(cx,cy) = false;
    }
  }
  return GridGvd;
}

Gvd::Gvd(DistMap* distMap){
  this->distMap = distMap;
}

Gvd::Gvd(pair<Int, Int> size){
  this->distMap = new DistMap(size,{Occupied},{Occupied,Unknown});
}

void Gvd::update(StateGrid sg){
  cout << "debug :: Update distMap" << endl;
  distMap->update(sg);
  cout << "debug :: Generate gridGVD" << endl;
  gridGvd = getGridGvd(*distMap);
  cout << "debug :: Generate graphGVD" << endl;
  graphGvd = GvdGraph(gridGvd);
}

Gvd::~Gvd(){
  delete distMap;
}
