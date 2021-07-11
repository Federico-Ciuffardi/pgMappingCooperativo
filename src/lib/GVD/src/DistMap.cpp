#include "DistMap.h"

//////////////
// DistCell //
//////////////
void DistMap::DistCell::addSource(Pos p) {
  obs.push_back(p);
}

bool DistMap::DistCell::hasSource(Pos p) {
  return find(obs.begin(), obs.end(), p) != obs.end();
}

bool DistMap::DistCell::operator>(const DistMap::DistCell& d) const {
  return this->distance > d.distance;
}

bool DistMap::DistCell::operator==(const DistMap::DistCell& d) const {
  return this->distance == d.distance;
}

ostream& operator<<(ostream& out, const DistMap::DistCell& cell) {
  // out<<"("<< cell.distance<<", "<< cell.obs.size() << " )";
  if (cell.distance != 0) {
    out << cell.obs.size();
  } else {
    out << "=";
  }
  return out;
}

DistMap::DistMapType::reference DistMap::operator[](Pos p){
  return distMap.cell(p);
}

/////////////
// DistMap //
/////////////
DistMap::DistMap(pair<Int,Int> size, vector<CellState> sources, vector<CellState> nonTraversables){
 distMap = DistMapType(size); 
 this->sources = sources;
 this->nonTraversables = nonTraversables;
}

void DistMap::update(StateGrid grid) {
  // initialize the dgrid and the distance queues
  DistPosQueue dqueue; // new queue
  fullDQueue = DistPosQueue(); //clear las full_dqueue

  // clear reset distanceMap
  for (Pos p : grid) {
    CellState cState = grid[p];

    DistMap::DistCell dcell;
    if (is_elem(cState, sources) || cState == Unknown) {  // TODO generalizar mejor
      dcell.distance = 0;
      if (is_elem(cState,sources)) {
        dcell.addSource(p);
        dqueue.push(DistPos(0, p));
      }
    } else {
      dcell.distance = inf;
    }
    distMap[p] = dcell;
  }

  DistPosQueue next_dqueue;
  while (!dqueue.empty()) {
    while (!dqueue.empty()) {
      Pos p = dqueue.top().second;
      dqueue.pop();

      // Tell neighbors to compute distance
      for (Pos np : grid.adj(p,nonTraversables)) {
        if (distMap[np].distance != inf) continue; // already computed

        // Compute np distance to the nearest source
        for (Pos nnp : grid.adj(np)) {
          /* if (distMap[nnp].distance == inf) continue; // invalid distance, can safely avoid computation */
          if(distMap[nnp].obs.empty()) continue;

          float d = np.distance_to(distMap[nnp].obs[0]);
          if (d < distMap[np].distance) {
            distMap[np].distance = d;
            distMap[np].obs.clear();
            distMap[np].addSource(distMap[nnp].obs[0]);
            distMap[np].distance = distMap[np].distance;
          } else if (d == distMap[np].distance && !distMap[np].hasSource(distMap[nnp].obs[0])) {
            distMap[np].addSource(distMap[nnp].obs[0]);
          }
        }
        next_dqueue.push(DistPos(distMap[np].distance, np));

        if (is_elem(Critical,sources) && grid[np] != Frontier) {
          continue;
        }
        fullDQueue.push(DistPos(distMap[np].distance, np));
      }
    }
    dqueue = next_dqueue;
    next_dqueue = DistPosQueue();
  }
  /* return boost::make_tuple(dgrid, full_dqueue); */
}
