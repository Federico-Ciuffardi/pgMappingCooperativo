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

pos operator-(const pos& p1, const pos& p2) {
  return pos(p1.first - p2.first, p1.second - p2.second);
}

pos operator*(const int c, const pos& p1) {
  return pos(p1.first * c, p1.second * c);
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

/* returns the dist_grid corresponding to the original grid, relative to
 * Occupide or Critical (from_type) */
boost::tuple<dist_grid, dist_pos_queue> calculate_distances(grid_type ogrid, cell_type from_type) {
  // get grid size
  pair<int, int> size = get_grid_size(ogrid);

  // declare distance grid to populate
  dist_grid dgrid;

  // initialize the dgrid and the distance queue (dqueue)
  dist_pos_queue dqueue;
  dist_pos_queue full_dqueue;
  for (int x = 0; x < size.first; x++) {
    dgrid.push_back(dist_row());
    for (int y = 0; y < size.second; y++) {
      dgrid[x].push_back(dist_cell());
      if (ogrid[x][y] == from_type || ogrid[x][y] == Unknown) {
        dgrid[x][y].distance = 0;
        if (ogrid[x][y] == from_type) {
          dgrid[x][y].add_obs(pos(x, y));
          dqueue.push(dist_pos(0, pos(x, y)));
        }
      } else {
        dgrid[x][y].distance = FLT_MAX;
      }
    }
  }

  dist_pos_queue next_dqueue;
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
            if (ogrid[nx][ny] != Unknown && ogrid[nx][ny] != from_type &&
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
                      float d =
                          dist(nx, ny, dgrid[nxk][nyl].obs[0].first, dgrid[nxk][nyl].obs[0].second);
                      if (d < min_distance) {
                        min_distance = d;
                        dgrid[nx][ny].obs.clear();
                        dgrid[nx][ny].add_obs(
                            pos(dgrid[nxk][nyl].obs[0].first, dgrid[nxk][nyl].obs[0].second));
                        dgrid[nx][ny].distance = min_distance;
                      } else if (d == min_distance &&
                                 !dgrid[nx][ny].has_obs(pos(dgrid[nxk][nyl].obs[0].first,
                                                            dgrid[nxk][nyl].obs[0].second))) {
                        dgrid[nx][ny].add_obs(
                            pos(dgrid[nxk][nyl].obs[0].first, dgrid[nxk][nyl].obs[0].second));
                      }
                    }
                  }
                }
              }
              next_dqueue.push(dist_pos(min_distance, pos(nx, ny)));
              if (from_type == Critical && ogrid[nx][ny] != Frontier) {
                continue;
              }
              full_dqueue.push(dist_pos(min_distance, pos(nx, ny)));
            }
          }
        }
      }
    }
    dqueue = next_dqueue;
    next_dqueue = dist_pos_queue();
  }
  return boost::make_tuple(dgrid, full_dqueue);
}
/*falta hacer el mapa de dist a celdas a esa dist para despues poder erosionar
  bien y despues habria que hace algo para pasar de grid a grafo (o capaz que no
  sirve)*/

vector<pos> P = {pos(-1, -1), pos(-1, 0), pos(-1, 1), pos(0, 1),
                 pos(1, 1),   pos(1, 0),  pos(1, -1), pos(0, -1)};

int A(pos p, grid_gvd ggvd) {
  int res = 0;
  bool prev_np = 0;
  for (int i = 0; i < 8; i++) {
    bool v1 = cell(ggvd, p + P[i]);
    bool v2 = cell(ggvd, p + P[(i + 1) % 8]);
    if (P[i].first == 0 || P[i].second == 0) {
      bool v3 = cell(ggvd, p + P[(i + 2) % 8]);
      res += v1 && !v2 && !v3;
    } else {
      res += v1 && !v2;
    }
  }
  return res;
}

/* returns a boolean matrix, a cell is true if it belongs to the gvd and false
 * otherwise*/
grid_gvd get_grid_gvd(dist_grid dg, dist_pos_queue dqueue) {
  // get sizes
  pair<int, int> size = get_grid_size(dg);

  // initialize grid_gvd
  grid_gvd grid_gvd;
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

/*
 *  gvd implementation
 */

boost::tuple<gvd::Vertex, bool> gvd::add_v(pos p) {
  NameVertexMap::iterator pos_it;
  bool inserted;
  Vertex u;
  boost::tie(pos_it, inserted) = positions.insert(std::make_pair(p, Vertex()));
  if (inserted) {
    u = add_vertex(g);
    g[u] = gvd_vertex(p, false);
    pos_it->second = u;
  } else {
    u = pos_it->second;
  }
  return boost::make_tuple(u, inserted);
}

pair<gvd::Edge, bool> gvd::add_e(Vertex u, Vertex v) {
  return add_edge(u, v, g);
}

gvd::gvd(grid_gvd ggvd) {
  pair<int, int> size = get_grid_size(ggvd);

  // initialize grid_gvd
  for (int x = 0; x < size.first; x++) {
    for (int y = 0; y < size.second; y++) {
      if (!ggvd[x][y])
        continue;

      // insert (x,y) to the graph
      Vertex u;
      bool inserted;
      boost::tie(u, inserted) = add_v(pos(x, y));

      // for each neighbor (nx,ny) of (i,j)
      for (int i = -1; i <= 1; i++) {
        int nx = x + i;
        for (int j = -1; j <= 1; j++) {
          int ny = y + j;
          if ((i != 0 || j != 0) && on_grid(nx, ny, ggvd) && ggvd[nx][ny]) {
            // add it to the graph
            Vertex v;
            bool inserted;
            boost::tie(v, inserted) = add_v(pos(nx, ny));
            // and also add an edge connecting them
            graph_traits<Graph>::edge_descriptor e;
            add_e(u, v);
          }
        }
      }
    }
  }
}

// could be of less order, maybe using trees
map<pos, bool> get_local_mins(dist_grid dg, gvd& GVD) {
  map<pos, bool> lmins;
  for (auto vp = vertices(GVD.g); vp.first != vp.second; ++vp.first) {
    bool auxmin = true;
    pos current_pos = GVD.g[*vp.first].p;
    bool not_processed = lmins.find(current_pos) == lmins.end();
    if (not_processed) {
      for (auto ad = adjacent_vertices(*vp.first, GVD.g); ad.first != ad.second; ++ad.first) {
        pos adj_pos = GVD.g[*ad.first].p;
        bool is_min = cell(dg, current_pos).distance < cell(dg, adj_pos).distance;
        if (is_min) {
          lmins[adj_pos] = false;
        }
        auxmin = is_min && auxmin;
      }
      lmins[current_pos] = auxmin;
    }
  }
  return lmins;
}

void collapse_vertices(gvd& GVD, map<pos, bool> lmins){

  /*// Remove all the vertices. This is OK.
  graph_traits<gvd::Graph>::vertex_iterator vi, vi_end, next;
  tie(vi, vi_end) = vertices(GVD.g);
  for (next = vi; vi != vi_end; vi = next) {
    ++next;
    remove_vertex(*vi, GVD.g);
  }*/

  //list<gvd::Vertex> remove_aux;
  for (auto vp = vertices(GVD.g); vp.first != vp.second;) {
    auto vp_aux = vp.first; 
    ++vp.first;
    pos current_pos = GVD.g[*vp_aux].p;
    bool is_min = lmins[current_pos];
    if(!is_min && out_degree(*vp_aux, GVD.g) == 2){
      auto adj = adjacent_vertices(*vp_aux, GVD.g);
      auto adj1 = GVD.g[*adj.first];
      pos adj1_aux = adj1.p - current_pos;
      auto adj_aux = adj;
      ++adj.first;
      auto adj2 = GVD.g[*adj.first];
      pos adj2_aux = (-1)*(adj2.p - current_pos);
      if(adj1_aux == adj2_aux){
        //cout<<"( "<< current_pos.first <<" , " <<current_pos.second<< " )"<<endl;
        GVD.add_e(*adj_aux.first, *adj.first);
        GVD.add_e(*adj.first, *adj_aux.first);
        clear_vertex(*vp_aux, GVD.g);
        remove_vertex(*vp_aux, GVD.g);
       
      }
    }
  }
}

int degree_constraint(grid_type& ogrid, gvd& GVD) {
  ofstream outfile;
  int criticals_count = 0;
  for (auto vp = vertices(GVD.g); vp.first != vp.second; ++vp.first) {
    pos current_pos = GVD.g[*vp.first].p;
    //bool is_min = lmins[current_pos];
    if (out_degree(*vp.first, GVD.g) == 2) {
      for (auto ad = adjacent_vertices(*vp.first, GVD.g); ad.first != ad.second; ++ad.first) {
        if (out_degree(*ad.first, GVD.g) >= 3) {
          ogrid[current_pos.first][current_pos.second] = Critical;
          criticals_count++;
          break;
        }
      }
    }
  }
  return criticals_count;
}

map<pos, dist_pos> unknown_dist_constraint(grid_type ogrid, gvd& GVD, int criticals_count) {
  dist_grid dgrid;
  dist_pos_queue dqueue;
  map<pos, dist_pos> critical_with_frontier;
  boost::tie(dgrid, dqueue) = calculate_distances(ogrid, Critical);
  while (criticals_count > 0 && !dqueue.empty()) {
    dist_pos frontier = dqueue.top();
    pos current_pos = frontier.second;
    dqueue.pop();
    pos critical_pos = cell(dgrid, current_pos).obs[0];
    gvd::Vertex v = GVD.positions[critical_pos];
    if (!GVD.g[v].is_critical) {
      GVD.g[v].is_critical = true;
      critical_with_frontier[critical_pos] = frontier;
      criticals_count--;
    }
  }
  return critical_with_frontier;
}

map<pos, dist_pos> get_critical_points(grid_type ogrid, dist_grid dg, gvd& GVD) {
  map<pos, bool> local_mins = get_local_mins(dg, GVD);
  collapse_vertices(GVD, local_mins);
  int criticals_count = degree_constraint(ogrid, GVD);
  //cout << criticals_count << endl;
  map<pos, dist_pos> critical_with_frontier = unknown_dist_constraint(ogrid, GVD, criticals_count);
  return critical_with_frontier;
  //return map<pos, dist_pos>();
}

void print_grid(grid_gvd ggvd, grid_type grid){
  ofstream outfile;
  outfile.open("/home/fede/catkin_ws/src/tscf_exploration/src/gvd/map.txt", ios::out | std::ofstream::app);
  int grid_size_x = grid.size();
  int grid_size_y = grid[0].size();
  //freopen( "../src/gvd/maps.txt", "rw", stdout );
  outfile << "gvd grid :" << endl;
  for (int x = 0; x < grid_size_x; x++) {
    for (int y = 0; y < grid_size_y; y++) {
      if (ggvd[x][y]) {
        outfile << "*|";
      } else if (grid[x][y] == Occupied) {
        outfile << "=|";
      } else if (grid[x][y] == Unknown) {
        outfile << "?|";
      } else {
        outfile << " |";
      }
    }
    outfile << endl;
  }
  outfile.close();
  //fclose(stdout);
}

set<pos> get_points_of_interest(grid_type ogrid){
  set<pos> res;
  dist_grid dgrid;
  dist_pos_queue dqueue;
  boost::tie(dgrid, dqueue) = calculate_distances(ogrid, Occupied);
  grid_gvd ggvd = get_grid_gvd(dgrid, dqueue);
  //print_grid(ggvd, ogrid);
  gvd GVD(ggvd);
  map<pos, dist_pos> cf = get_critical_points(ogrid, dgrid, GVD);
  for(auto it = cf.begin(); it!=cf.begin(); ++it){
    res.insert(it->second.second);
  }
  return res;
}