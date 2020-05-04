#include "gvd.h"

#include <math.h>
#include <cfloat>

/*
 *  dist_cell implementation
 */

void dist_cell::add_obs(pos p) {
  obs.push_back(p);
}

bool dist_cell::has_obs(pos p) {
  return find(obs.begin(), obs.end(), p) != obs.end();
}

bool dist_cell::operator>(const dist_cell& d) const {
  return this->distance > d.distance;
}

bool dist_cell::operator==(const dist_cell& d) const {
  return this->distance == d.distance;
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

pos operator+(const pos& p1, const pos& p2) {
  return pos(p1.first + p2.first, p1.second + p2.second);
}

template <typename T>
T cell(vector<vector<T>> grid, pos p) {
  return grid[p.first][p.second];
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
template <typename T>
bool on_grid(int x, int y, vector<vector<T>> grid) {
  pair<int, int> size = get_grid_size(grid);
  return ((x >= 0 && x < size.first) && (y >= 0 && y < size.second));
}

template <typename T>
bool on_grid(pos p, vector<vector<T>> grid) {
  return on_grid(p.first, p.second, grid);
}

/*
 *  main funcs
 */

/* returns the dist_grid corresponding to the original grid */
pair<dist_grid, priority_queue<dist_pos>> calculate_distances(grid_type ogrid) {
  // get grid size
  pair<int, int> size = get_grid_size(ogrid);

  // declare distance grid to populate
  dist_grid dgrid;

  // initialize the dgrid and the distance queue (dqueue)
  priority_queue<dist_pos> dqueue;
  priority_queue<dist_pos> full_dqueue;
  for (int x = 0; x < size.first; x++) {
    dgrid.push_back(dist_col());
    for (int y = 0; y < size.second; y++) {
      dgrid[x].push_back(dist_cell());
      if (ogrid[x][y] == Occupied) {
        dgrid[x][y].distance = 0;
        dgrid[x][y].add_obs(pos(x, y));

        dqueue.push(dist_pos(0, pos(x, y)));
      } else {
        dgrid[x][y].distance = FLT_MAX;
      }
    }
  }

  priority_queue<dist_pos> next_dqueue;
  while (!dqueue.empty()) {
    while (!dqueue.empty()) {
      // get the first cell to process
      pos current_pos = dqueue.top().second;
      dqueue.pop();

      // Look at neighbors to find a new free cell that needs its
      // distance updated
      for (int i = -1; i <= 1; i++) {
        int nx = current_pos.first + i;
        for (int j = -1; j <= 1; j++) {
          int ny = current_pos.second + j;
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
                        dgrid[nx][ny].add_obs(
                            pos(dgrid[nxk][nyl].obs[0].first,
                                dgrid[nxk][nyl].obs[0].second));
                        dgrid[nx][ny].distance = min_distance;
                      } else if (d == min_distance &&
                                 !dgrid[nx][ny].has_obs(
                                     pos(dgrid[nxk][nyl].obs[0].first,
                                         dgrid[nxk][nyl].obs[0].second))) {
                        dgrid[nx][ny].add_obs(
                            pos(dgrid[nxk][nyl].obs[0].first,
                                dgrid[nxk][nyl].obs[0].second));
                      }
                    }
                  }
                }
              }
              next_dqueue.push(dist_pos(min_distance, pos(nx, ny)));
              full_dqueue.push(dist_pos(min_distance, pos(nx, ny)));
            }
          }
        }
      }
    }
    dqueue = next_dqueue;
    next_dqueue=priority_queue<dist_pos>();
  }
  return pair<dist_grid, priority_queue<dist_pos>>(dgrid, full_dqueue);
}

/*falta hacer el mapa de dist a celdas a esa dist para despues poder erosionar
  bien y despues habria que hace algo para pasar de grid a grafo (o capaz que no
  sirve)*/

vector<pos> P = {pos(-1, -1), pos(-1, 0), pos(-1, 1), pos(0, 1),
                 pos(1, 1),   pos(1, 0),  pos(1, -1), pos(0, -1)};

int A(pos p, vector<vector<bool>> grid_gvd) {
  int res = 0;
  bool prev_np = 0;
  for (int i = 0; i < 8; i++) {
    bool v1 = cell(grid_gvd, p + P[i]);
    bool v2 = cell(grid_gvd, p + P[(i + 1) % 8]);
    if (P[i].first == 0 || P[i].second == 0) {
      bool v3 = cell(grid_gvd, p + P[(i + 2) % 8]);
      res += v1 && !v2 && !v3;
    } else {
      res += v1 && !v2;
    }
  }
  return res;
}

/* returns a boolean matrix, a cell is true if it belongs to the gvd and false
 * otherwise*/
vector<vector<bool>> get_grid_gvd(dist_grid dg, priority_queue<dist_pos> dqueue) {
  // get sizes
  pair<int, int> size = get_grid_size(dg);

  // initialize grid_gvd
  vector<vector<bool>> grid_gvd;
  for (int i = 0; i < size.first; i++) {
    grid_gvd.push_back(vector<bool>());
    for (int j = 0; j < size.second; j++) {
      grid_gvd[i].push_back(dg[i][j].distance != 0);
    }
  }

  // compute grid gvd
  while (!dqueue.empty()) {
    // get the first cell to process
    pos current_pos = dqueue.top().second;
    dqueue.pop();

    int cx = current_pos.first;
    int cy = current_pos.second;

    // Remove from gvd if it does not belongs to the gvd by definition ands does
    // not disconects the gvd
    if (dg[cx][cy].obs.size() <= 1 && A(current_pos, grid_gvd) <= 1) {
      grid_gvd[cx][cy] = false;
    }
  }
  return grid_gvd;
}
