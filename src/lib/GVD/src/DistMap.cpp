#include "DistMap.h"

DistMap::DistMap(pair<Int,Int> size){
 distMap = DistMapType(size); 
}


void DistMap::DistCell::add_obs(Pos p) {
  obs.push_back(p);
}

bool DistMap::DistCell::has_obs(Pos p) {
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

/* returns the DistMap corresponding to the original grid, relative to
 * Occupide or Critical (from_type) */
boost::tuple<DistMap, DistPosQueue> calculate_distances(StateGrid ogrid, CellState sourceState) {
  // get grid size
  pair<Int, Int> size = ogrid.size();

  // initialize the dgrid and the distance queues
  DistMap dgrid(ogrid.size());
  DistPosQueue dqueue, full_dqueue;
  for (int x = 0; x < size.first; x++) {
    for (int y = 0; y < size.second; y++) {
      Pos p = Pos(x, y);
      CellState cState = ogrid.cell(p);

      DistMap::DistCell dcell;
      if (cState == sourceState || cState == Unknown) {  // TODO es asi porque sirve para una funcion
                                                         // Posterior pero esta semanticamente mal
        dcell.distance = 0;
        if (cState == sourceState) {
          dcell.add_obs(Pos(x, y));
          dqueue.push(DistPos(0, Pos(x, y)));
        }
      } else {
        dcell.distance = FLT_MAX;
      }
      dgrid[p] = dcell;
    }
  }

  DistPosQueue next_dqueue;
  while (!dqueue.empty()) {
    while (!dqueue.empty()) {
      // get the x cell to process
      Pos p = dqueue.top().second;
      dqueue.pop();

      // Look at neighbors to find a new free cell that needs its distance updated

      for (Pos np : ogrid.adj(p)) {
        CellState nCState = ogrid.cell(np);
        DistMap::DistCell& n_dcell = dgrid[np];
        bool is_traversable = nCState != Unknown && nCState != Occupied;
        if (is_traversable && n_dcell.distance == FLT_MAX) {
          float min_distance = FLT_MAX;

          // look at neighbors of freecell to find cells whose distance has already been found
          for (Pos nnp : ogrid.adj(np)) {
            CellState nnCState = ogrid.cell(nnp);
            DistMap::DistCell& nnDCell = dgrid[nnp];
            if (nnDCell.obs.size() > 0) {
              // find distance to neighbor's closest cell and update the number of obstacles at
              // that distance
              float d = np.distance_to(nnDCell.obs[0]);
              if (d < min_distance) {
                min_distance = d;
                n_dcell.obs.clear();
                n_dcell.add_obs(nnDCell.obs[0]);
                n_dcell.distance = min_distance;
              } else if (d == min_distance && !n_dcell.has_obs(nnDCell.obs[0])) {
                n_dcell.add_obs(nnDCell.obs[0]);
              }
            }
            next_dqueue.push(DistPos(min_distance, np));

            if (sourceState == Critical && nCState != Frontier) {
              continue;
            }
            full_dqueue.push(DistPos(min_distance, np));
          }
        }
      }
    }
    dqueue = next_dqueue;
    next_dqueue = DistPosQueue();
  }
  return boost::make_tuple(dgrid, full_dqueue);
}
