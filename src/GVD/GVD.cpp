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
  out<<"( "<< p.first <<" , " <<p.second<< " )"<<endl;
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

void print(pos p){
  ofstream outfile;
  outfile.open("/home/fede/catkin_ws/src/tscf_exploration/src/GVD/map.txt",
               ios::out | std::ofstream::app);
  outfile << p <<endl;
  outfile.close();
}

void print(string s){
  ofstream outfile;
  outfile.open("/home/fede/catkin_ws/src/tscf_exploration/src/GVD/map.txt",
               ios::out | std::ofstream::app);
  outfile << s <<endl;
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
      if (ctype == from_type || ctype == Unknown) { //FIXME es asi porque sirve para una funcion posterior pero esta semanticamente mal
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

/* given a graph for the neighbors of a pos return the number of conex components if pos would be removed */
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
      grid_gvd[i].push_back(dg[i][j].distance != 0);// is not a wall or unknown
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

/*
 *  GVD implementation
 */

boost::tuple<GVD::Vertex, bool> GVD::add_v(pos p) {
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

pair<GVD::Edge, bool> GVD::add_e(Vertex u, Vertex v) {
  return add_edge(u, v, g);
}

GVD::GVD(grid_gvd ggvd) {
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
map<pos, bool> get_local_mins(dist_grid dg, GVD& gvd) {
  map<pos, bool> lmins;
  for (auto vp = vertices(gvd.g); vp.first != vp.second; ++vp.first) {
    bool auxmin = true;
    pos current_pos = gvd.g[*vp.first].p;
    bool not_processed = lmins.find(current_pos) == lmins.end();
    if (not_processed) {
      for (auto ad = adjacent_vertices(*vp.first, gvd.g); ad.first != ad.second; ++ad.first) {
        pos adj_pos = gvd.g[*ad.first].p;
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

bool same_direcction(pos p1, pos p2) {
  return normalized(p1) == -normalized(p2);
}

void collapse_vertices(GVD& gvd, map<pos, bool> lmins) {
  /*// Remove all the vertices. This is OK.
  graph_traits<GVD::Graph>::vertex_iterator vi, vi_end, next;
  tie(vi, vi_end) = vertices(gvd.g);
  for (next = vi; vi != vi_end; vi = next) {
    ++next;
    remove_vertex(*vi, gvd.g);
  }*/

  // list<GVD::Vertex> remove_aux;
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
        // cout<<"( "<< current_pos.first <<" , " <<current_pos.second<< " )"<<endl;
        gvd.add_e(*adj_aux.first, *adj.first);
        gvd.add_e(*adj.first, *adj_aux.first);
        clear_vertex(*vp_aux, gvd.g);
        remove_vertex(*vp_aux, gvd.g);
      }
    }
  }
}

void clean_up(GVD& gvd) {
  /*GVD::VertexIterator v_it, v_it_end;
  for (tie(v_it, v_it_end) = vertices(gvd.g); v_it != v_it_end; v_it++) {
    GVD::Vertex max_deg_v = *v_it; 
    int max_deg = out_degree(*v_it, gvd.g);
    if(max_deg > 2){ // just for not recalculating out_degree(*v_it, gvd.g) it does not have anything to do with the max deg itself
      GVD::VertexIterator av_it, av_it_end;
      for (tie(av_it, av_it_end) = adjacent_vertices(*v_it, gvd.g); av_it != av_it_end; ++av_it) {
        if(out_degree(*av_it,gvd.g)>max_deg){
          max_deg_v = *av_it;
        }
      }

      for (tie(av_it, av_it_end) = adjacent_vertices(max_deg_v, gvd.g); av_it != av_it_end; ++av_it) {
        if(out_degree(*av_it,gvd.g)>max_deg){
          max_deg_v = *av_it;
        }
      }

    }
  }*/
}

int degree_constraint(grid_type& ogrid, GVD& gvd) {
  int criticals_count = 0;
  for (auto vp = vertices(gvd.g); vp.first != vp.second; ++vp.first) {
    pos current_pos = gvd.g[*vp.first].p;
    // bool is_min = lmins[current_pos];
    if (out_degree(*vp.first, gvd.g) == 2) {
      for (auto ad = adjacent_vertices(*vp.first, gvd.g); ad.first != ad.second; ++ad.first) {
        if (out_degree(*ad.first, gvd.g) >= 3) {
          ogrid[current_pos.first][current_pos.second] = Critical;
          criticals_count++;
          break;
        }
      }
    }
  }
  return criticals_count;
}

map<pos, dist_pos> unknown_dist_constraint(grid_type ogrid, GVD& gvd, int criticals_count) {
  dist_grid dgrid;
  dist_pos_queue dqueue;
  map<pos, dist_pos> critical_with_frontier;
  boost::tie(dgrid, dqueue) = calculate_distances(ogrid, Critical);
  while (criticals_count > 0 && !dqueue.empty()) {
    dist_pos frontier_dp = dqueue.top();
    dqueue.pop();
    pos critical_pos = cell(dgrid, frontier_dp.second).obs[0];
    GVD::Vertex cv = gvd.positions[critical_pos];
    if (!gvd.g[cv].is_critical) {
      //print("Encontre un critico:");
      //print(critical_pos);
      //print("Y la frontera es:");
      //print(frontier_dp.second);
      gvd.g[cv].is_critical = true;
      critical_with_frontier[critical_pos] = frontier_dp;
      criticals_count--;
    }
  }
  return critical_with_frontier;
}

map<pos, dist_pos> get_critical_points(grid_type ogrid, dist_grid dg, GVD& gvd) {
  map<pos, bool> local_mins = get_local_mins(dg, gvd);
  collapse_vertices(gvd, local_mins);
  clean_up(gvd);
  int criticals_count = degree_constraint(ogrid, gvd);
  // cout << criticals_count << endl;
  map<pos, dist_pos> critical_with_frontier = unknown_dist_constraint(ogrid, gvd, criticals_count);
  return critical_with_frontier;
  // return map<pos, dist_pos>();
}

void print_grid(grid_gvd ggvd, grid_type grid, map<pos, dist_pos> cf = map<pos, dist_pos>(), map<pos,bool> frontier_aux = map<pos,bool>()) {
  ofstream outfile;
  outfile.open("/home/fede/catkin_ws/src/tscf_exploration/src/GVD/map.txt",
               ios::out | std::ofstream::app);
  int grid_size_x = grid.size();
  int grid_size_y = grid[0].size();
  outfile << "GVD grid :" << endl;
  for (int x = 0; x < grid_size_x; x++) {
    for (int y = 0; y < grid_size_y; y++) {
      if(frontier_aux[pos(x,y)]){
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
        if(grid[x][y] == Frontier){
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
}

boost::tuple<set<pos>, GVD> get_points_of_interest(grid_type ogrid) {
  set<pos> res;
  dist_grid dgrid;
  dist_pos_queue dqueue;
  boost::tie(dgrid, dqueue) = calculate_distances(ogrid, Occupied);
  grid_gvd ggvd = get_grid_gvd(dgrid, dqueue);
  GVD gvd(ggvd);
  //map<pos,bool> frontier_aux;
  map<pos, dist_pos> cf = get_critical_points(ogrid, dgrid, gvd);
  for (auto it = cf.begin(); it != cf.end(); ++it) {
    res.insert(it->second.second);
    //frontier_aux[it->second.second] = true;
  }
  print(to_string(res.size()));
  //print_grid(ggvd, ogrid, cf, frontier_aux);
  return boost::make_tuple(res, gvd);
}