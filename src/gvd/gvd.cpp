#include "gvd.h"
#include <math.h>
#include <cfloat>

/*
 *  dist_cell implementation
 */

void dist_cell::add_obs(int x, int y) {
  obs.push_back(pair<int, int>(x, y));
}

bool dist_cell::has_obs(int x, int y) {
  return find(obs.begin(), obs.end(), pair<int, int>(x, y)) != obs.end();
}

bool dist_cell::operator>(const dist_cell& d) const {
  return this->distance > d.distance;
}

ostream& operator<<(ostream& out, const dist_cell& cell) {
  // out<<"("<< cell.distance<<", "<< cell.obs.size() << " )";
  if (cell.distance != 0) {
    out << cell.obs.size();
  } else {
    out << "=";
  }
  return out;
}

/*
 *  Aux funcs
 */

/* Euclidean distance between two points.*/
inline float dist(float ax, float ay, float bx, float by) {
  return sqrt(pow(ax - bx, 2) + pow(ay - by, 2));
}

/* gets the size of a grid (vector<vector<T>>) */
template <typename T>
pair<int, int> get_grid_size(vector<vector<T>> grid) {
  return pair<int, int>(grid.size(), grid[0].size());
}

/* true if (x,y) is on the grid */
bool on_grid(int x, int y, grid_type grid) {
  pair<int, int> size = get_grid_size(grid);
  return ((x >= 0 && x < size.first) && (y >= 0 && y < size.second));
}

/*
 *  main funcs
 */

/* returns the dist_grid corresponding to the original grid */
dist_grid calculate_distances(grid_type ogrid) {
  // get grid size
  pair<int, int> size = get_grid_size(ogrid);

  // declare distance grid to populate
  dist_grid dgrid;

  // initialize the dgrid and the distance queue (dqueue)
  heap<dist_cell> dqueue;
  for (int x = 0; x < size.first; x++) {
    dgrid.push_back(dist_col());
    for (int y = 0; y < size.second; y++) {
      dgrid[x].push_back(dist_cell());
      if (ogrid[x][y] == Occupied) {
        dgrid[x][y].distance = 0;
        dgrid[x][y].add_obs(x, y);

        dqueue.push(dgrid[x][y]);
      } else {
        dgrid[x][y].distance = FLT_MAX;
      }
    }
  }

  heap<dist_cell> next_dqueue;
  while (!dqueue.empty()) {
    while (!dqueue.empty()) {
      // get the first cell to process
      dist_cell current_cell = dqueue.first();
      dqueue.pop();

      // Look at neighbors to find a new free cell that needs its
      // distance updated
      for (int i = -1; i <= 1; i++) {
        int nx = current_cell.obs[0].first + i;
        for (int j = -1; j <= 1; j++) {
          int ny = current_cell.obs[0].second + j;
          if ((i != 0 || j != 0) && on_grid(nx, ny, ogrid)) {
            if (ogrid[nx][ny] != Occupied &&
                dgrid[nx][ny].distance == FLT_MAX) {
              float min_distance = FLT_MAX;
              // look at neighbors of freecell to find cells whose
              // distance has already been found
              for (int k = -1; k <= 1; k++) {
                int nxk = nx + k;
                for (int l = -1; l <= 1; l++) {
                  int nyl = ny + l;
                  if ((k != 0 || l != 0) && on_grid(nxk, nyl, ogrid)) {
                    if (dgrid[nxk][nyl].obs.size() > 0) {
                      // find distance to neighbor's closest cell
                      // and update the number of obstacles at that distance
                      float d = dist(nx, ny, dgrid[nxk][nyl].obs[0].first,
                                     dgrid[nxk][nyl].obs[0].second);
                      if (d < min_distance) {
                        min_distance = d;
                        dgrid[nx][ny].obs.clear();
                        dgrid[nx][ny].add_obs(dgrid[nxk][nyl].obs[0].first,
                                              dgrid[nxk][nyl].obs[0].second);
                        dgrid[nx][ny].distance = min_distance;
                      } else if (d == min_distance &&
                                 !dgrid[nx][ny].has_obs(
                                     dgrid[nxk][nyl].obs[0].first,
                                     dgrid[nxk][nyl].obs[0].second)) {
                        dgrid[nx][ny].add_obs(dgrid[nxk][nyl].obs[0].first,
                                              dgrid[nxk][nyl].obs[0].second);
                      }
                    }
                  }
                }
              }
              dist_cell me_cell;
              me_cell.add_obs(nx, ny);
              me_cell.distance = min_distance;
              next_dqueue.push(me_cell);
            }
          }
        }
      }
    }
    dqueue = next_dqueue;
    next_dqueue.clear();
  }
  return dgrid;
}

/*falta hacer el mapa de dist a celdas a esa dist para despues poder erosionar bien
  y despues habria que hace algo pra pasar de grid a grafo (o capaz que no sirve)*/


/* returns a boolean matrix, a cell is true if it belongs to the gvd and false
 * otherwise*/
vector<vector<bool>> get_grid_gvd(dist_grid dg) {
  // get sizes
  int grid_size_x = dg.size();
  int grid_size_y = dg[0].size();

  // initialize grid_gvd
  vector<vector<bool>> grid_gvd;
  for (int i = 0; i < grid_size_x; i++) {
    grid_gvd.push_back(vector<bool>());
    for (int j = 0; j < grid_size_y; j++) {
      grid_gvd[i].push_back(true);
    }
  }

  // compute grid gvd
  for (int i = 0; i < grid_size_x; i++) {
    for (int j = 0; j < grid_size_y; j++) {
      grid_gvd[i][j] = dg[i][j].distance > 0 && dg[i][j].obs.size() > 1;
    }
  }
  return grid_gvd;
}
