#ifndef GVD_H
#define GVD_H

#include <bits/stdc++.h>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
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
typedef priority_queue<dist_pos, vector<dist_pos>, greater<dist_pos>> dist_pos_queue;

/*
 *  pos implementation
 */

pos operator+(const pos& p1, const pos& p2);

pos operator-(const pos& p1, const pos& p2);

pos operator-(const pos& p1);

pos operator*(const int c, const pos& p1);

pos operator/(const pos& p1, const float c);

ostream& operator<<(ostream& out, const pos& p);


/*
 *  critical info 
 */
struct critical_info{
  float mind_f; //min distance to frontier
  vector<pos> frontiers;
};
typedef map<pos, critical_info> criticals_info;


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
  pos segment;
  gvd_vertex(){};
  gvd_vertex(pos p, bool is_critical, pos segment) {
    this->p = p;
    this->is_critical = is_critical;
    this->segment;
  }
};

struct GVD {
  /*typedef adjacency_list_traits< listS, listS, bidirectionalS >::vertex_descriptor vertex_descriptor;
  typedef adjacency_list<listS, listS, bidirectionalS, property< vertex_index_t, int,
            property< vertex_name_t, char,
                property< vertex_distance_t, int,
                    property< vertex_predecessor_t, vertex_descriptor,gvd_vertex > > > > ,property<edge_weight_t, float>>*/

  typedef adjacency_list<listS, listS, bidirectionalS, gvd_vertex,property<edge_weight_t, float>>
      Graph;  // maybe using list is not the most eficient way

  typedef graph_traits<Graph>::vertex_descriptor Vertex;
  typedef graph_traits<Graph>::edge_descriptor Edge;

  typedef std::map<pos, Vertex> NameVertexMap;

  typedef graph_traits<Graph>::vertex_iterator VertexIterator;
  typedef graph_traits<Graph>::adjacency_iterator AdjacencyIterator;

  typedef graph_traits<Graph>::edge_iterator EdgeIterator;
  // typedef graph_traits<Graph>::adjacency_iterator adjacency_iterator;

  Graph g;
  NameVertexMap positions;
  GVD(){};
  GVD(grid_gvd);

  boost::tuple<GVD::Vertex, bool> add_v(pos p);

  pair<GVD::Edge, bool> add_e(Vertex u, Vertex v, float w);
};

struct VecGVD {

  typedef adjacency_list<vecS, vecS, bidirectionalS, gvd_vertex,property<edge_weight_t, float>>
      Graph;  // maybe using list is not the most eficient way

  typedef graph_traits<Graph>::vertex_descriptor Vertex;
  typedef graph_traits<Graph>::edge_descriptor Edge;

  typedef std::map<pos, Vertex> NameVertexMap;

  typedef graph_traits<Graph>::vertex_iterator VertexIterator;
  typedef graph_traits<Graph>::adjacency_iterator AdjacencyIterator;

  typedef graph_traits<Graph>::edge_iterator EdgeIterator;
  // typedef graph_traits<Graph>::adjacency_iterator adjacency_iterator;

  Graph g;
  NameVertexMap positions;
  VecGVD(){};
  VecGVD(grid_gvd);

  boost::tuple<VecGVD::Vertex, bool> add_v(pos p);

  pair<VecGVD::Edge, bool> add_e(Vertex u, Vertex v, float w);
};


using namespace std;

/*
 *  functions
 */
boost::tuple<dist_grid, dist_pos_queue> calculate_distances(grid_type ogrid, cell_type from_type);

grid_gvd get_grid_gvd(dist_grid dg, dist_pos_queue);

//map<pos, dist_pos> get_critical_points(grid_type ogrid, dist_grid dg, GVD& gvd);
// map<pos,bool> get_local_mins(dist_grid dg, GVD gvd);

boost::tuple<criticals_info, GVD> get_points_of_interest(grid_type g);

boost::tuple<list<VecGVD::Vertex>,float> get_path(VecGVD gvd, pos from, pos to);

float dist(pos p1, pos p2);

#endif