#ifndef GVD_H
#define GVD_H

#include <bits/stdc++.h>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <iostream>
#include "data/Vector2.h"

using namespace std;

/*
 *  position
 */
typedef Vector2 Pos;
typedef pair<float, Pos> DistPos;
typedef priority_queue<DistPos, vector<DistPos>, greater<DistPos>> DistPosQueue;

/*
 *  Misc
 */

template <typename T>
Pos get_grid_size(vector<vector<T>> grid);

template <typename T>
bool on_grid(int x, int y, vector<vector<T>> grid);
/*
 *  ocupancy grid
 */
enum cell_type { Occupied, Unknown, Free, Critical, Frontier };
typedef vector<cell_type> row_type;
typedef vector<row_type> grid_type;

/*
 *  Pos implementation
 */

/* Pos operator+(const Pos& p1, const Pos& p2); */

/* Pos operator-(const Pos& p1, const Pos& p2); */

/* Pos operator-(const Pos& p1); */

/* Pos operator*(const int c, const Pos& p1); */

/* Pos operator/(const Pos& p1, const float c); */

/* ostream& operator<<(ostream& out, const Pos& p); */


/*
 *  critical info 
 */
struct critical_info{
  float mind_f; //min distance to frontier
  vector<Pos> frontiers;
};
typedef boost::unordered_map<Pos, critical_info> criticals_info;


/*
 *  dist cell, col and grid
 */
struct dist_cell {
  vector<Pos> obs;
  float distance;

  dist_cell(){};

  void add_obs(Pos);
  bool has_obs(Pos);

  bool operator>(const dist_cell& d) const;
  bool operator==(const dist_cell& p) const;
  friend ostream& operator<<(ostream& out, const dist_cell& cell);
};

typedef vector<dist_cell> dist_row;
typedef vector<dist_row> dist_grid;

/*
 *  GVD
 */
typedef vector<vector<bool>> grid_gvd;

using namespace boost;

struct gvd_vertex {
  Pos p;
  bool is_critical;
  Pos segment;
  gvd_vertex(){};
  gvd_vertex(Pos p, bool is_critical, Pos segment) {
    this->p = p;
    this->is_critical = is_critical;
    this->segment = Pos(-9999,-9999);
  }
};

template<typename graph>
struct genericGVD {
  typedef graph Graph;
  typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
  typedef typename graph_traits<Graph>::edge_descriptor Edge;

  typedef typename boost::unordered_map<Pos, Vertex> NameVertexMap;
  typedef typename NameVertexMap::iterator NameVertexMapIterator;
  typedef typename graph_traits<Graph>::vertex_iterator VertexIterator;
  typedef typename graph_traits<Graph>::adjacency_iterator AdjacencyIterator;

  typedef typename graph_traits<Graph>::edge_iterator EdgeIterator;
  // typedef graph_traits<Graph>::adjacency_iterator adjacency_iterator;

  Graph g;
  NameVertexMap positions;
  genericGVD(){};
  genericGVD(grid_gvd ggvd){
    Pos size = get_grid_size(ggvd);

    // initialize grid_gvd
    for (int x = 0; x < size.x; x++) {
      for (int y = 0; y < size.y; y++) {
        if (!ggvd[x][y])
          continue;

        // insert (x,y) to the graph
        Vertex u;
        bool inserted;
        boost::tie(u, inserted) = add_v(Pos(x, y));

        // for each neighbor (nx,ny) of (i,j)
        for (int i = -1; i <= 1; i++) {
          int nx = x + i;
          for (int j = -1; j <= 1; j++) {
            int ny = y + j;
            if ((i != 0 || j != 0) && on_grid(nx, ny, ggvd) && ggvd[nx][ny]) {
              // add it to the graph
              Vertex v;
              bool inserted;
              boost::tie(v, inserted) = add_v(Pos(nx, ny));
              // and also add an edge connecting them
              Edge e;
              add_e(u, v);
            }
          }
        }
      }
    }
  };

  boost::tuple<Vertex, bool> add_v(Pos p) {
    NameVertexMapIterator pos_it;
    bool inserted;
    Vertex u;
    boost::tie(pos_it, inserted) = positions.insert(std::make_pair(p, Vertex()));
    if (inserted) {
      u = add_vertex(g);
      g[u] = gvd_vertex(p, false, Pos(-1, -1));
      pos_it->second = u;
    } else {
      u = pos_it->second;
    }
    return boost::make_tuple(u, inserted);
  }

  pair<Edge, bool> add_e(Vertex u, Vertex v, float w = -1) {
    if (w == -1) {
      return add_edge(u, v, g);
    }
    return add_edge(u, v, w, g);
  }
};

typedef genericGVD<adjacency_list<listS, listS, bidirectionalS, gvd_vertex,property<edge_weight_t, float>>> GVD;
typedef genericGVD<adjacency_list<vecS, vecS, bidirectionalS, gvd_vertex,property<edge_weight_t, float>>> VecGVD;

using namespace std;

/*
 *  functions
 */
boost::tuple<dist_grid, DistPosQueue> calculate_distances(grid_type ogrid, cell_type from_type);

boost::tuple<boost::unordered_map<Pos,Pos>, Pos> find_paths_to_gvd(grid_type ogrid, VecGVD gvd, Pos p_pos);

grid_gvd get_grid_gvd(dist_grid dg, DistPosQueue);

//boost::unordered_map<Pos, DistPos> get_critical_points(grid_type ogrid, dist_grid dg, GVD& gvd);
// boost::unordered_map<Pos,bool> get_local_mins(dist_grid dg, GVD gvd);

boost::tuple<criticals_info, GVD> get_points_of_interest(grid_type g);

boost::tuple<list<VecGVD::Vertex>,float> get_single_path(VecGVD gvd, Pos from, Pos to);

typedef boost::unordered_set<Pos> pos_set;

boost::tuple<boost::unordered_map<Pos,list<VecGVD::Vertex>> , boost::unordered_map<Pos,float>> get_multi_path(VecGVD gvd, Pos start, pos_set goals);

float dist(Pos p1, Pos p2);

#endif
