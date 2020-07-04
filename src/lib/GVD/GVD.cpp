#include "GVD.h"

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

/*
 *  pos implementation
 */

pos operator+(const pos& p1, const pos& p2) {
  return pos(p1.first + p2.first, p1.second + p2.second);
}

pos operator-(const pos& p1, const pos& p2) {
  return pos(p1.first - p2.first, p1.second - p2.second);
}

pos operator-(const pos& p1) {
  return pos(-p1.first, -p1.second);
}

pos operator*(const int c, const pos& p1) {
  return pos(p1.first * c, p1.second * c);
}

pos operator/(const pos& p1, const float c) {
  return pos(p1.first / c, p1.second / c);
}

ostream& operator<<(ostream& out, const pos& p) {
  // out<<"("<< cell.distance<<", "<< cell.obs.size() << " )";
  out << "( " << p.first << " , " << p.second << " )";
  return out;
}

/* Euclidean distance between two points.*/
float dist(pos p1, pos p2) {
  return sqrt(pow(p1.first - p2.first, 2) + pow(p1.second - p2.second, 2));
}

/* Module of the vector */
float lenght(pos p) {
  return dist(pos(0, 0), p);
}

/* Module of the vector, (round by c++, from float to int) */
pos normalized(pos p) {
  return p / lenght(p);
}

/*
 *  Aux funcs
 */

/* Euclidean distance between two points.*/
float dist(float ax, float ay, float bx, float by) {
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

/* true if pos is on the grid */
template <typename T>
bool on_grid(pos p, vector<vector<T>> grid) {
  return on_grid(p.first, p.second, grid);
}

bool cell(vector<vector<bool>> grid, pos p) {
  return grid[p.first][p.second];
}

template <typename T>
T& cell(vector<vector<T>>& grid, pos p) {
  return grid[p.first][p.second];
}

void print(pos p) {
  ofstream outfile;
  outfile.open("/home/fede/catkin_ws/src/tscf_exploration/src/GVD/map.txt",
               ios::out | std::ofstream::app);
  outfile << p << endl;
  outfile.close();
}

void print(string s) {
  ofstream outfile;
  outfile.open("/home/fede/catkin_ws/src/tscf_exploration/src/GVD/map.txt",
               ios::out | std::ofstream::app);
  outfile << s << endl;
  outfile.close();
}

/*
 *  main funcs
 */

vector<pos> neighbor = {pos(-1, -1), pos(-1, 0), pos(-1, 1), pos(0, 1),
                        pos(1, 1),   pos(1, 0),  pos(1, -1), pos(0, -1)};

/* returns the dist_grid corresponding to the original grid, relative to
 * Occupide or Critical (from_type) */
boost::tuple<dist_grid, dist_pos_queue> calculate_distances(grid_type ogrid, cell_type from_type) {
  // get grid size
  pair<int, int> size = get_grid_size(ogrid);

  // initialize the dgrid and the distance queues
  dist_grid dgrid;
  dist_pos_queue dqueue, full_dqueue;
  for (int x = 0; x < size.first; x++) {
    dgrid.push_back(dist_row());
    for (int y = 0; y < size.second; y++) {
      pos p = pos(x, y);
      cell_type ctype = cell(ogrid, p);

      dist_cell dcell;
      if (ctype == from_type || ctype == Unknown) {  // TODO es asi porque sirve para una funcion
                                                     // posterior pero esta semanticamente mal
        dcell.distance = 0;
        if (ctype == from_type) {
          dcell.add_obs(pos(x, y));
          dqueue.push(dist_pos(0, pos(x, y)));
        }
      } else {
        dcell.distance = FLT_MAX;
      }
      dgrid[x].push_back(dcell);
    }
  }

  dist_pos_queue next_dqueue;
  while (!dqueue.empty()) {
    while (!dqueue.empty()) {
      // get the first cell to process
      pos p = dqueue.top().second;
      dqueue.pop();

      // Look at neighbors to find a new free cell that needs its distance updated
      for (int i = 0; i <= neighbor.size(); i++) {
        pos np = p + neighbor[i];
        if (on_grid(np, ogrid)) {
          cell_type n_ctype = cell(ogrid, np);
          dist_cell& n_dcell = cell(dgrid, np);
          bool is_traversable = n_ctype != Unknown && n_ctype != Occupied;
          if (is_traversable && n_dcell.distance == FLT_MAX) {
            float min_distance = FLT_MAX;

            // look at neighbors of freecell to find cells whose distance has already been found
            for (int i = 0; i <= neighbor.size(); i++) {
              pos nnp = np + neighbor[i];
              if (on_grid(nnp, ogrid)) {
                cell_type nn_ctype = cell(ogrid, nnp);
                dist_cell& nn_dcell = cell(dgrid, nnp);
                if (nn_dcell.obs.size() > 0) {
                  // find distance to neighbor's closest cell and update the number of obstacles at
                  // that distance
                  float d = dist(np, nn_dcell.obs[0]);
                  if (d < min_distance) {
                    min_distance = d;
                    n_dcell.obs.clear();
                    n_dcell.add_obs(nn_dcell.obs[0]);
                    n_dcell.distance = min_distance;
                  } else if (d == min_distance && !n_dcell.has_obs(nn_dcell.obs[0])) {
                    n_dcell.add_obs(nn_dcell.obs[0]);
                  }
                }
              }
            }
            next_dqueue.push(dist_pos(min_distance, np));

            if (from_type == Critical && n_ctype != Frontier) {
              continue;
            }
            full_dqueue.push(dist_pos(min_distance, np));
          }
        }
      }
    }
    dqueue = next_dqueue;
    next_dqueue = dist_pos_queue();
  }
  return boost::make_tuple(dgrid, full_dqueue);
}

boost::tuple<boost::unordered_map<pos, pos>, pos> find_paths_to_gvd(grid_type ogrid,
                                                                    VecGVD gvd,
                                                                    pos p_pos) {
  // get grid size
  pair<int, int> size = get_grid_size(ogrid);

  // initialize the dgrid and the distance queues
  dist_grid dgrid;
  dist_pos_queue dqueue, full_dqueue;
  for (int x = 0; x < size.first; x++) {
    dgrid.push_back(dist_row());
    for (int y = 0; y < size.second; y++) {
      pos p = pos(x, y);
      cell_type ctype = cell(ogrid, p);

      dist_cell dcell;
      if (p == p_pos || ctype == Unknown) {  // TODO es asi porque sirve para una funcion
                                             // posterior pero esta semanticamente mal
        dcell.distance = 0;
        if (p == p_pos) {
          dcell.add_obs(pos(x, y));
          dqueue.push(dist_pos(0, pos(x, y)));
        }
      } else {
        dcell.distance = FLT_MAX;
      }
      dgrid[x].push_back(dcell);
    }
  }
  boost::unordered_map<pos, pos> v_predecessor;
  dist_pos_queue next_dqueue;
  while (!dqueue.empty()) {
    while (!dqueue.empty()) {
      // get the first cell to process
      pos p = dqueue.top().second;
      dqueue.pop();

      // Look at neighbors to find a new free cell that needs its distance updated
      for (int i = 0; i <= neighbor.size(); i++) {
        pos np = p + neighbor[i];
        if (on_grid(np, ogrid)) {
          cell_type n_ctype = cell(ogrid, np);
          dist_cell& n_dcell = cell(dgrid, np);
          bool is_traversable = n_ctype != Unknown && n_ctype != Occupied;
          if (is_traversable && n_dcell.distance == FLT_MAX) {
            float min_distance = FLT_MAX;

            // look at neighbors of freecell to find cells whose distance has already been found
            for (int i = 0; i <= neighbor.size(); i++) {
              pos nnp = np + neighbor[i];
              if (on_grid(nnp, ogrid)) {
                cell_type nn_ctype = cell(ogrid, nnp);
                dist_cell& nn_dcell = cell(dgrid, nnp);
                if (nn_dcell.obs.size() > 0) {
                  // find distance to neighbor's closest cell and update the number of obstacles at
                  // that distance
                  float d = dist(np, nnp) + nn_dcell.distance;
                  if (d < min_distance) {
                    min_distance = d;
                    n_dcell.obs.clear();
                    n_dcell.add_obs(nn_dcell.obs[0]);
                    n_dcell.distance = min_distance;
                    v_predecessor[np] = nnp;
                  } else if (d == min_distance && !n_dcell.has_obs(nn_dcell.obs[0])) {
                    n_dcell.add_obs(nn_dcell.obs[0]);
                    v_predecessor[np] = nnp;
                  }
                }
              }
            }
            next_dqueue.push(dist_pos(min_distance, np));

            if (gvd.positions.find(np) != gvd.positions.end()) {
              return boost::make_tuple(v_predecessor, np);
            }
          }
        }
      }
    }
    dqueue = next_dqueue;
    next_dqueue = dist_pos_queue();
  }
  return boost::make_tuple(v_predecessor, pos());
}

/* given a graph for the neighbors of a pos return the number of conex components if pos would be
 * removed */
int A(pos p, grid_gvd ggvd) {
  int res = 0;
  bool prev_np = 0;
  for (int i = 0; i < 8; i++) {
    bool v1 = cell(ggvd, p + neighbor[i]);
    bool v2 = cell(ggvd, p + neighbor[(i + 1) % 8]);
    if (neighbor[i].first == 0 || neighbor[i].second == 0) {
      bool v3 = cell(ggvd, p + neighbor[(i + 2) % 8]);
      res += v1 && !v2 && !v3;
    } else {
      res += v1 && !v2;
    }
    if (res > 1)
      break;
  }
  return res;
}

/* returns a boolean matrix, a cell is true if it belongs to the GVD and false
 * otherwise*/
grid_gvd get_grid_gvd(dist_grid dg, dist_pos_queue dqueue) {
  // get sizes
  pair<int, int> size = get_grid_size(dg);

  // initialize grid_gvd
  grid_gvd grid_gvd;
  for (int i = 0; i < size.first; i++) {
    grid_gvd.push_back(vector<bool>());
    for (int j = 0; j < size.second; j++) {
      grid_gvd[i].push_back(dg[i][j].distance != 0);  // is not a wall or unknown
    }
  }

  // compute grid GVD
  while (!dqueue.empty()) {
    // get the first cell to process
    pos current_pos = dqueue.top().second;
    dqueue.pop();

    int cx = current_pos.first;
    int cy = current_pos.second;

    // Remove from GVD if it does not belongs to the GVD by definition ands does
    // not disconects the GVD
    if (dg[cx][cy].obs.size() <= 1 && A(current_pos, grid_gvd) <= 1) {
      grid_gvd[cx][cy] = false;
    }
  }
  return grid_gvd;
}

// could be of less order, maybe using trees
boost::unordered_map<pos, bool> get_local_mins(dist_grid dg, GVD& gvd) {
  boost::unordered_map<pos, bool> lmins;

  for (auto vp = vertices(gvd.g); vp.first != vp.second; ++vp.first) {
    bool is_min_e = true;
    bool has_greater = false;
    pos current_pos = gvd.g[*vp.first].p;
    bool not_processed = lmins.find(current_pos) == lmins.end();

    if (not_processed) {
      for (auto ad = adjacent_vertices(*vp.first, gvd.g); ad.first != ad.second; ++ad.first) {
        pos adj_pos = gvd.g[*ad.first].p;
        bool auxmin = cell(dg, current_pos).distance <= cell(dg, adj_pos).distance;
        bool adj_greater = cell(dg, current_pos).distance < cell(dg, adj_pos).distance;

        if (adj_greater) {
          has_greater = true;
          lmins[adj_pos] = false;
        }
        is_min_e = is_min_e && auxmin;
        if (!is_min_e)
          break;
      }
      lmins[current_pos] = is_min_e && has_greater;
    }
  }
  return lmins;
}

bool same_direcction(pos p1, pos p2) {
  return normalized(p1) == -normalized(p2);
}

void collapse_vertices(GVD& gvd, boost::unordered_map<pos, bool> lmins) {
  for (auto vp = vertices(gvd.g); vp.first != vp.second;) {
    auto vp_aux = vp.first;
    ++vp.first;
    pos current_pos = gvd.g[*vp_aux].p;

    bool is_min = lmins[current_pos];
    if (!is_min && out_degree(*vp_aux, gvd.g) == 2) {
      auto adj = adjacent_vertices(*vp_aux, gvd.g);

      auto adj1 = gvd.g[*adj.first];
      pos adj1_aux = adj1.p - current_pos;
      auto adj_aux = adj;

      ++adj.first;
      auto adj2 = gvd.g[*adj.first];
      pos adj2_aux = (adj2.p - current_pos);

      if (same_direcction(adj1_aux, adj2_aux)) {
        gvd.add_e(*adj_aux.first, *adj.first);
        gvd.add_e(*adj.first, *adj_aux.first);
        clear_vertex(*vp_aux, gvd.g);
        remove_vertex(*vp_aux, gvd.g);
      }
    }
  }
}

void clean_up(GVD& gvd, dist_grid dgrid, int min_deg) {
  GVD::VertexIterator v_it, v_it_end;
  for (tie(v_it, v_it_end) = vertices(gvd.g); v_it != v_it_end;) {
    GVD::VertexIterator v_it_aux = v_it;
    v_it++;

    GVD::Vertex max_deg_v = *v_it_aux;
    int max_deg = out_degree(*v_it_aux, gvd.g);

    if (max_deg >=
        min_deg) {  // just for not recalculating out_degree(*v_it_aux, gvd.g) it does not have
                    // anything to do with the max deg itself
      GVD::AdjacencyIterator av_it, av_it_end;
      for (boost::tie(av_it, av_it_end) = boost::adjacent_vertices(*v_it_aux, gvd.g);
           av_it != av_it_end; ++av_it) {
        int current_degree = out_degree(*av_it, gvd.g);
        if (current_degree >=
            max_deg) {  // >= max clean up, > just when there is just one max_degree
          max_deg_v = *av_it;
          max_deg = current_degree;
        }
      }

      if (max_deg_v == *v_it_aux)
        continue;

      for (tie(av_it, av_it_end) = adjacent_vertices(*v_it_aux, gvd.g); av_it != av_it_end;) {
        GVD::AdjacencyIterator av_it_aux = av_it;
        ++av_it;
        if (max_deg_v == *av_it_aux)
          continue;
        if (edge(*av_it_aux, max_deg_v, gvd.g).second) {
          remove_edge(*av_it_aux, *v_it_aux, gvd.g);

          if (out_degree(*av_it_aux, gvd.g) == 1) {
            clear_vertex(*av_it_aux, gvd.g);
            break;
          } else {
            remove_edge(*v_it_aux, *av_it_aux, gvd.g);
            tie(av_it, av_it_end) = adjacent_vertices(*v_it_aux, gvd.g);
          }
        }
      }
    }
  }
  for (tie(v_it, v_it_end) = vertices(gvd.g); v_it != v_it_end;) {
    GVD::VertexIterator v_it_aux = v_it;
    v_it++;
    if (out_degree(*v_it_aux, gvd.g) == 0) {
      remove_vertex(*v_it_aux, gvd.g);
    }
  }
}

int degree_constraint(grid_type& ogrid, GVD& gvd, boost::unordered_map<pos, bool> local_mins) {
  int criticals_count = 0;
  pos current_pos;
  for (auto vp = vertices(gvd.g); vp.first != vp.second; ++vp.first) {
    current_pos = gvd.g[*vp.first].p;
    if (out_degree(*vp.first, gvd.g) == 2 && local_mins[current_pos]) {
      for (auto ad = adjacent_vertices(*vp.first, gvd.g); ad.first != ad.second; ++ad.first) {
        if (out_degree(*ad.first, gvd.g) >= 3) {
          ogrid[current_pos.first][current_pos.second] = Critical;
          criticals_count++;
          break;
        }
      }
    }
  }
  // No real critical point found, create an artificial one representing the hole space as one
  // segment
  if (criticals_count == 0) {
    int max = -1;
    pos max_pos;

    for (auto vp = vertices(gvd.g); vp.first != vp.second; ++vp.first) {
      current_pos = gvd.g[*vp.first].p;
      int current_deg = out_degree(*vp.first, gvd.g);
      if (current_deg > max || max < 0) {
        max = current_deg;
        max_pos = current_pos;
      }
    }
    ogrid[max_pos.first][max_pos.second] = Critical;
    criticals_count++;
  }
  return criticals_count;
}

/*boost::unordered_map<pos, dist_pos> unknown_dist_constraint(grid_type ogrid, GVD& gvd, int
criticals_count) { dist_grid dgrid; dist_pos_queue dqueue; boost::unordered_map<pos, dist_pos>
critical_with_frontier; boost::tie(dgrid, dqueue) = calculate_distances(ogrid, Critical); while
(criticals_count > 0 && !dqueue.empty()) { dist_pos frontier_dp = dqueue.top(); dqueue.pop(); pos
critical_pos = cell(dgrid, frontier_dp.second).obs[0]; GVD::Vertex cv = gvd.positions[critical_pos];
    if (!gvd.g[cv].is_critical) {
      // print("Encontre un critico:");
      // print(critical_pos);
      // print("Y la frontera es:");
      // print(frontier_dp.second);
      gvd.g[cv].is_critical = true;
      critical_with_frontier[critical_pos] = frontier_dp;
      criticals_count--;
    }
  }
  return critical_with_frontier;
}*/

// Returns two maps : criticals -> frontiers, criticals-> min dis to frontier, segments gvd
criticals_info unknown_dist_constraint2(grid_type ogrid, GVD& gvd) {
  dist_grid dgrid;
  dist_pos_queue dqueue;
  criticals_info res;
  cout << "calculate_distances" << endl;
  boost::tie(dgrid, dqueue) = calculate_distances(ogrid, Critical);  // dqueue = frontier queue
  cout << "set each frontier to a critical" << endl;
  while (!dqueue.empty()) {
    dist_pos frontier_dp = dqueue.top();
    dqueue.pop();
    pos frontier = frontier_dp.second;

    vector<pos>& frontier_crits = cell(dgrid, frontier).obs;

    for (int i = 0; i < frontier_crits.size(); i++) {
      pos critical_pos = frontier_crits[i];
      GVD::Vertex cv = gvd.positions[critical_pos];
      if (!gvd.g[cv].is_critical) {
        gvd.g[cv].is_critical = true;
        // gvd.g[cv].segment = critical_pos;
        res[critical_pos].mind_f = frontier_dp.first;
        res[critical_pos].frontiers.push_back(frontier);
        // frontier_crits.clear();
        frontier_crits[0] = critical_pos;
        break;
      }
      if (i == (frontier_crits.size() - 1)) {  // i is the last one
        res[critical_pos].frontiers.push_back(frontier);
        // frontier_crits.clear();
        frontier_crits[0] = critical_pos;
      }
    }
  }
  cout << "set the segment for every vertex" << endl;
  GVD::VertexIterator v_it, v_it_end;
  for (tie(v_it, v_it_end) = vertices(gvd.g); v_it != v_it_end; v_it++) {
    gvd_vertex& v = gvd.g[*v_it];
    if (cell(dgrid, v.p).obs.size() > 0) {
      v.segment = cell(dgrid, v.p).obs[0];
    } else {
      v.segment = pos(INT_MIN, INT_MIN);
      cout << "warning vertex without segment" << endl;
    }
  }
  return res;
}

criticals_info get_critical_points(grid_type ogrid, dist_grid dg, GVD& gvd) {
  boost::unordered_map<pos, bool> local_mins = get_local_mins(dg, gvd);
  cout << "debug :: clean_up" << endl;
  // TODO clean_up and collapse_vertices can be merged into one function
  clean_up(gvd, dg, 3);
  clean_up(gvd, dg, 2);
  GVD gvd_copy = gvd;
  cout << "debug :: collapse_vertices" << endl;
  collapse_vertices(gvd_copy, local_mins);

  // TODO borrar el criticals count no se usa
  // Quick Fix, pass the local_mins to the degree constraint, this should be mark on the gvd so
  // there is no need to passed the map of local mins
  cout << "debug :: degree_constraint" << endl;
  int criticals_count = degree_constraint(ogrid, gvd_copy, local_mins);
  // cout << criticals_count << endl;
  // boost::unordered_map<pos, dist_pos> critical_with_frontier = unknown_dist_constraint(ogrid,
  // gvd, criticals_count);
  cout << "debug :: unknown_dist_constraint2" << endl;
  criticals_info cis = unknown_dist_constraint2(ogrid, gvd);
  // return critical_with_frontier;
  return cis;
}

/*void print_grid(grid_gvd ggvd,
                grid_type grid,
                boost::unordered_map<pos, dist_pos> cf = boost::unordered_map<pos, dist_pos>(),
                boost::unordered_map<pos, bool> frontier_aux = boost::unordered_map<pos, bool>()) {
  ofstream outfile;
  outfile.open("/home/fede/catkin_ws/src/tscf_exploration/src/GVD/map.txt",
               ios::out | std::ofstream::app);
  int grid_size_x = grid.size();
  int grid_size_y = grid[0].size();
  outfile << "GVD grid :" << endl;
  for (int x = 0; x < grid_size_x; x++) {
    for (int y = 0; y < grid_size_y; y++) {
      if (frontier_aux[pos(x, y)]) {
        outfile << "x|";
        continue;
      }
      if (ggvd[x][y]) {
        if (cf.find(pos(x, y)) != cf.end()) {
          outfile << "o|";
          continue;
        }
        outfile << "*|";
      } else if (grid[x][y] == Occupied) {
        outfile << "=|";
      } else if (grid[x][y] == Unknown) {
        outfile << "?|";
      } else {
        if (grid[x][y] == Frontier) {
          outfile << "+|";
          continue;
        }
        outfile << " |";
      }
    }
    outfile << endl;
  }
  outfile.close();
  // fclose(stdout);
}*/

boost::tuple<criticals_info, GVD> get_points_of_interest(grid_type ogrid) {
  boost::unordered_set<pos> res;
  dist_grid dgrid;
  dist_pos_queue dqueue;
  cout << "debug :: calculate_distances" << endl;
  boost::tie(dgrid, dqueue) = calculate_distances(ogrid, Occupied);
  cout << "debug :: get_grid_gvd" << endl;
  grid_gvd ggvd = get_grid_gvd(dgrid, dqueue);
  cout << "debug :: ggvd" << endl;
  GVD gvd(ggvd);
  // boost::unordered_map<pos,bool> frontier_aux;
  criticals_info cis = get_critical_points(ogrid, dgrid, gvd);
  // cis = get_critical_points(ogrid, dgrid, gvd);
  // return boost::make_tuple(res, gvd)
  // for (auto it = cf.begin(); it != cf.end(); ++it) {
  // res.insert(it->second.second);
  // frontier_aux[it->second.second] = true;
  //}
  // print(to_string(res.size()));
  // print_grid(ggvd, ogrid, cf, frontier_aux);
  // return boost::make_tuple(res, gvd);
  cout << "debug :: make_tuple" << endl;
  return boost::make_tuple(cis, gvd);
}

// A* single

/// euclidean distance heuristic
template <class Graph, class CostType, class LocMap>
class single_astar_distance_heuristic : public astar_heuristic<Graph, CostType> {
 public:
  typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
  single_astar_distance_heuristic(LocMap l, Vertex goal) : m_location(l), m_goal(goal) {}
  CostType operator()(Vertex u) { return dist(m_location[u].p, m_location[m_goal].p); }

 private:
  LocMap m_location;
  Vertex m_goal;
};

struct found_goal {};  // exception for termination

/// visitor that terminates when we find the goal
template <class Vertex>
class single_astar_goal_visitor : public boost::default_astar_visitor {
 public:
  single_astar_goal_visitor(Vertex goal) : m_goal(goal) {}
  template <class Graph>
  void examine_vertex(Vertex u, Graph& g) {
    if (u == m_goal)
      throw found_goal();
  }

 private:
  Vertex m_goal;
};

/// get the shortestpath and the cost of reaching the goal
boost::tuple<list<VecGVD::Vertex>, float> get_single_path(VecGVD gvd, pos from, pos to) {
  VecGVD::Graph g = gvd.g;
  VecGVD::Vertex start = gvd.positions[from];
  VecGVD::Vertex goal = gvd.positions[to];
  typedef VecGVD::Graph mygraph_t;
  typedef VecGVD::Vertex vertex;
  typedef float cost;

  vector<mygraph_t::vertex_descriptor> p(num_vertices(g));
  vector<cost> d(num_vertices(g));
  list<vertex> shortest_path;
  try {
    // call astar named parameter interface
    astar_search_tree(g, start,
                      single_astar_distance_heuristic<mygraph_t, cost, VecGVD::Graph>(gvd.g, goal),
                      predecessor_map(make_iterator_property_map(p.begin(), get(vertex_index, g)))
                          .distance_map(make_iterator_property_map(d.begin(), get(vertex_index, g)))
                          .visitor(single_astar_goal_visitor<vertex>(goal)));
  } catch (found_goal fg) {  // found a path to the goal

    for (vertex v = goal;; v = p[v]) {
      shortest_path.push_front(v);
      if (p[v] == v)
        break;
    }
    // cout << "Shortest path from " << from << " to " << to << ": ";

    list<vertex>::iterator spi = shortest_path.begin();
    // cout << from;
    // for (++spi; spi != shortest_path.end(); ++spi)
    //  cout << " -> " << gvd.g[*spi].p;

    // cout << endl << "Total travel time: " << d[goal] << endl;
  }
  return boost::make_tuple(shortest_path, d[goal]);
}

// Multi A*

/// euclidean distance heuristic
template <class Graph, class CostType>
class multi_astar_distance_heuristic : public astar_heuristic<Graph, CostType> {
 public:
  typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
  multi_astar_distance_heuristic(Graph g, pos_set goals) : m_graph(g), m_goals(goals) {}
  CostType operator()(Vertex u) {
    // cout<<"heuristica antes"<<m_goals.size()<<endl;
    pos current_target = *(m_goals.begin());
    CostType distance = dist(m_graph[u].p, current_target);
    // cout<<distance<<endl;
    auto it = m_goals.find(m_graph[u].p);
    if (it != m_goals.end() && m_goals.size() > 1) {
      m_goals.erase(it);
    }
    pos new_target = *(m_goals.begin());
    // cout<<"heuristica desopues"<<m_goals.size()<<endl;
    return dist(m_graph[u].p, new_target);
  }

 private:
  Graph m_graph;
  pos_set m_goals;
};

struct found_goals {};  // exception for termination

/// visitor that terminates when we find the goal
template <class Vertex>
class multi_astar_goal_visitor : public boost::default_astar_visitor {
 public:
  multi_astar_goal_visitor(pos_set goals) : m_goals(goals) {}
  template <class Graph>
  void examine_vertex(Vertex u, Graph g) {
    // cout<<"visitor antess"<<m_goals.size()<<endl;
    auto it = m_goals.find(g[u].p);
    if (it != m_goals.end()) {
      if (m_goals.size() > 1) {
        m_goals.erase(it);
      } else {
        throw found_goals();
      }
    }
    // cout<<"visitor despues"<<m_goals.size()<<endl;
    if (m_goals.size() == 0) {
      throw found_goals();
    }
  }

 private:
  pos_set m_goals;
};

/// get the shortestpath and the cost of reaching the goal
boost::tuple<boost::unordered_map<pos, list<VecGVD::Vertex>>, boost::unordered_map<pos, float>>
get_multi_path(VecGVD gvd, pos start, pos_set goals) {
  VecGVD::Graph g = gvd.g;

  vector<VecGVD::Vertex> p(num_vertices(g));
  vector<float> d(num_vertices(g));
  boost::unordered_map<pos, list<VecGVD::Vertex>> shortest_paths;
  boost::unordered_map<pos, float> shortest_paths_costs;
  try {
    // call astar named parameter interface
    astar_search_tree(g, gvd.positions[start],
                      multi_astar_distance_heuristic<VecGVD::Graph, float>(gvd.g, goals),
                      predecessor_map(make_iterator_property_map(p.begin(), get(vertex_index, g)))
                          .distance_map(make_iterator_property_map(d.begin(), get(vertex_index, g)))
                          .visitor(multi_astar_goal_visitor<VecGVD::Vertex>(goals)));
  } catch (found_goals fg) {  // found a path to the goal
    // cout<<"comienzo del termino!"<<endl;
    for (auto it = goals.begin(); it != goals.end(); it++) {
      list<VecGVD::Vertex> shortest_path;
      pos goal = *it;
      VecGVD::Vertex goal_v = gvd.positions[goal];

      for (VecGVD::Vertex v = goal_v;; v = p[v]) {
        shortest_path.push_front(v);
        if (p[v] == v)
          break;
      }

      shortest_paths[goal] = shortest_path;

      shortest_paths_costs[goal] = d[goal_v];
    }

    for (auto it = goals.begin(); it != goals.end(); it++) {
      pos goal = *it;
      // cout << "Shortest path from " << start << " to " << goal << ": ";

      list<VecGVD::Vertex>::iterator spi = shortest_paths[goal].begin();
      /*cout << start;
      for (++spi; spi != shortest_paths[goal].end(); ++spi)
        cout << " -> " << gvd.g[*spi].p;

      cout << endl << "Total travel time: " << d[gvd.positions[goal]] << endl;*/
    }
  }
  // cout<<"fin del termino!"<<endl;
  return boost::make_tuple(shortest_paths, shortest_paths_costs);
}