#ifndef GVD_H
#define GVD_H

#include <bits/stdc++.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <iostream>

using namespace std;

/*
 *  ocupancy grid
 */
enum cell_type { Occupied, Unknown, Free, Critical, Frontier };
typedef vector<cell_type> row_type;
typedef vector<row_type> grid_type;

/*
 *  position
 */
typedef pair<int, int> pos;
typedef pair<float, pos> dist_pos;
typedef priority_queue<dist_pos, vector<dist_pos>, greater<>> dist_pos_queue;

/*
 *  dist cell, col and grid
 */
struct dist_cell {
  vector<pos> obs;
  float distance;

  dist_cell(){};

  void add_obs(pos);
  bool has_obs(pos);

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
  pos p;
  bool is_critical;
  gvd_vertex(){};
  gvd_vertex(pos p, bool is_critical) {
    this->p = p;
    this->is_critical = is_critical;
  }
};

struct GVD {
  typedef adjacency_list<listS, listS, bidirectionalS, gvd_vertex> Graph; // maybe using list is not the most eficient way
  typedef graph_traits<Graph>::vertex_descriptor Vertex;
  typedef std::map<pos, Vertex> NameVertexMap;
  typedef graph_traits<Graph>::edge_descriptor Edge;
  // typedef graph_traits<Graph>::adjacency_iterator adjacency_iterator;

  Graph g;
  NameVertexMap positions;
  GVD(){};
  GVD(grid_gvd);
  boost::tuple<GVD::Vertex, bool> add_v(pos p);
  pair<GVD::Edge, bool> add_e(Vertex a, Vertex b);
};

using namespace std;

/*
 *  functions
 */
boost::tuple<dist_grid, dist_pos_queue> calculate_distances(grid_type ogrid, cell_type from_type);

grid_gvd get_grid_gvd(dist_grid dg, dist_pos_queue);

map<pos, dist_pos> get_critical_points(grid_type ogrid, dist_grid dg, GVD& gvd);
// map<pos,bool> get_local_mins(dist_grid dg, GVD gvd);


boost::tuple<set<pos>,GVD> get_points_of_interest(grid_type g);

#endif