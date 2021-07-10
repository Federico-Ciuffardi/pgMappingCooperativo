#pragma once

#include <bits/stdc++.h>
#include <iostream>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <limits>

#include <boost/unordered_set.hpp>

#include "Grid.h"
#include "Pos.h"


using namespace boost;

// Boost Graph wrapper
template<typename graph, typename vertex_id>
struct Graph {
  typedef          graph GraphType;
  typedef          vertex_id VertexId;

  typedef typename graph_traits<GraphType>::vertex_descriptor Vertex;
  typedef typename GraphType::vertex_property_type VertexProperty;
  typedef typename graph_traits<GraphType>::edge_descriptor Edge;

  typedef typename boost::unordered_map<VertexId, Vertex> NameVertexMap;
  typedef typename NameVertexMap::iterator NameVertexMapIterator;
  typedef typename graph_traits<GraphType>::vertex_iterator VertexIterator;
  typedef typename graph_traits<GraphType>::adjacency_iterator AdjacencyIterator;

  typedef typename graph_traits<GraphType>::edge_iterator EdgeIterator;
  // typedef graph_traits<Graph>::adjacency_iterator adjacency_iterator;

  GraphType g;
  NameVertexMap vertices;

  Graph(){};
  // construct graph based on al boolean grid (true = belongs to GvdGraph)
  Graph(Grid<bool> boolGrid){
    pair<int, int> size = boolGrid.size();

    // initialize grid_gvd
    for (int x = 0; x < size.first; x++) {
      for (int y = 0; y < size.second; y++) {
        if (!boolGrid.cell(x,y))
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
            if ((i != 0 || j != 0) && boolGrid.inside(nx, ny) && boolGrid.cell(nx,ny)) {
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

  // Get node info asociated with p
  VertexProperty& operator[](Vertex v){
    return g[v];
  }
  // Get node info asociated with p
  VertexProperty& operator[](VertexId vid){
    return g[vertices[vid]];
  }

  // add vertex asociated with p
  boost::tuple<Vertex, bool> add_v(Pos p) {
    NameVertexMapIterator pos_it;
    bool inserted;
    Vertex u;
    boost::tie(pos_it, inserted) = vertices.insert(std::make_pair(p, Vertex()));
    if (inserted) {
      u = add_vertex(g);
      g[u] = VertexProperty(p);
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

// Boost Graph wrapper
// Focused on Graphs where the vertices are identified by a spatial location (repesented with the type Pos)
// graph (template parameter) VertexProperty must have a attribute p with time Pos and a constructor `VertexProperty(p)`
// Adds A* funtions (Pos is hardcoded but in theory any class with a distance_to method could be used)
template<typename graph>
struct PosGraph : public Graph<graph,Pos> {
  // keep updated if the parent class changes
  typedef Graph<graph,Pos> ParentClass; 
  using Graph<graph,Pos>::Graph; // inherit constructors (not usign ParentClass becouse intellisense bug)

  // inherit typedefs
  using typename ParentClass::GraphType;

  using typename ParentClass::VertexId;

  using typename ParentClass::Vertex;
  using typename ParentClass::VertexProperty;
  using typename ParentClass::Edge;

  using typename ParentClass::NameVertexMap;
  using typename ParentClass::NameVertexMapIterator;
  using typename ParentClass::VertexIterator;
  using typename ParentClass::AdjacencyIterator;

  using typename ParentClass::EdgeIterator;


  // pathfinding
  /// A* single
  //// euclidean distance heuristic
  template <class GraphType, class CostType, class LocMap>
  class single_astar_distance_heuristic : public astar_heuristic<GraphType, CostType> {
   public:
    typedef typename graph_traits<GraphType>::vertex_descriptor Vertex;
    single_astar_distance_heuristic(LocMap l, Vertex goal) : m_location(l), m_goal(goal) {}
    CostType operator()(Vertex u) { return m_location[u].p.distance_to(m_location[m_goal].p); }

   private:
    LocMap m_location;
    Vertex m_goal;
  };

  struct found_goal {};  // exception for termination

  //// visitor that terminates when we find the goal
  template <class Vertex>
  class single_astar_goal_visitor : public boost::default_astar_visitor {
   public:
    single_astar_goal_visitor(Vertex goal) : m_goal(goal) {}
    template <class GraphType>
    void examine_vertex(Vertex u, GraphType& g) {
      if (u == m_goal)
        throw found_goal();
    }

   private:
    Vertex m_goal;
  };
  //// get the shortestpath and the cost of reaching the goal
  boost::tuple<list<Vertex>, float> getSinglePath(Pos from, Pos to) {
    Vertex start = this->vertices[from];
    Vertex goal = this->vertices[to];
    typedef float cost;

    vector<Vertex> p(num_vertices(this->g));
    vector<cost> d(num_vertices(this->g));
    list<Vertex> shortest_path;
    try {
      // call astar named parameter interface
      astar_search_tree(this->g, start,
                        single_astar_distance_heuristic<GraphType, cost, GraphType>(this->g, goal),
                        predecessor_map(make_iterator_property_map(p.begin(), get(vertex_index, this->g)))
                            .distance_map(make_iterator_property_map(d.begin(), get(vertex_index, this->g)))
                            .visitor(single_astar_goal_visitor<Vertex>(goal)));
    } catch (found_goal fg) {  // found a path to the goal

      for (Vertex v = goal;; v = p[v]) {
        shortest_path.push_front(v);
        if (p[v] == v)
          break;
      }
      // cout << "Shortest path from " << from << " to " << to << ": ";

      /* list<Vertex>::iterator spi = shortest_path.begin(); */
      // cout << from;
      // for (++spi; spi != shortest_path.end(); ++spi)
      //  cout << " -> " << gvd.g[*spi].p;

      // cout << endl << "Total travel time: " << d[goal] << endl;
    }
    return boost::make_tuple(shortest_path, d[goal]);
  }

  /// Multi A*
  //// euclidean distance heuristic
  template <class GraphType, class CostType>
  class multi_astar_distance_heuristic : public astar_heuristic<GraphType, CostType> {
   public:
    typedef typename graph_traits<GraphType>::vertex_descriptor Vertex;
    multi_astar_distance_heuristic(GraphType g, PosSet goals) : m_graph(g), m_goals(goals) {}
    CostType operator()(Vertex u) {
      // cout<<"heuristica antes"<<m_goals.size()<<endl;
      Pos current_target = *(m_goals.begin());
      CostType distance = m_graph[u].p.distance_to(current_target);
      // cout<<distance<<endl;
      auto it = m_goals.find(m_graph[u].p);
      if (it != m_goals.end() && m_goals.size() > 1) {
        m_goals.erase(it);
      }
      Pos new_target = *(m_goals.begin());
      // cout<<"heuristica desopues"<<m_goals.size()<<endl;
      return m_graph[u].p.distance_to(new_target);
    }

   private:
    GraphType m_graph;
    PosSet m_goals;
  };

  struct found_goals {};  // exception for termination

  //// visitor that terminates when we find the goal
  template <class Vertex>
  class multi_astar_goal_visitor : public boost::default_astar_visitor {
   public:
    multi_astar_goal_visitor(PosSet goals) : m_goals(goals) {}
    template <class GraphType>
    void examine_vertex(Vertex u, GraphType g) {
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
    PosSet m_goals;
  };
  //// get the shortestpath and the cost of reaching the goal
  boost::tuple<boost::unordered_map<Pos, list<Vertex>>, boost::unordered_map<Pos, float>>
  getMultiPath(Pos start, PosSet goals) {
    vector<Vertex> p(num_vertices(this->g));
    vector<float> d(num_vertices(this->g));
    boost::unordered_map<Pos, list<Vertex>> shortest_paths;
    boost::unordered_map<Pos, float> shortest_paths_costs;
    try {
      // call astar named parameter interface
      astar_search_tree(this->g, this->vertices[start],
                        multi_astar_distance_heuristic<GraphType, float>(this->g, goals),
                        predecessor_map(make_iterator_property_map(p.begin(), get(vertex_index, this->g)))
                            .distance_map(make_iterator_property_map(d.begin(), get(vertex_index, this->g)))
                            .visitor(multi_astar_goal_visitor<Vertex>(goals)));
    } catch (found_goals fg) {  // found a path to the goal
      // cout<<"comienzo del termino!"<<endl;
      for (auto it = goals.begin(); it != goals.end(); it++) {
        list<Vertex> shortest_path;
        Pos goal = *it;
        Vertex goal_v = this->vertices[goal];

        for (Vertex v = goal_v;; v = p[v]) {
          shortest_path.push_front(v);
          if (p[v] == v)
            break;
        }

        shortest_paths[goal] = shortest_path;

        shortest_paths_costs[goal] = d[goal_v];
      }

      /* for (auto it = goals.begin(); it != goals.end(); it++) { */
        /* Pos goal = *it; */
        // cout << "Shortest path from " << start << " to " << goal << ": ";

        /* list<GvdVecGraph::Vertex>::iterator spi = shortest_paths[goal].begin(); */
        /*cout << start;
        for (++spi; spi != shortest_paths[goal].end(); ++spi)
          cout << " -> " << gvd.g[*spi].p;

        cout << endl << "Total travel time: " << d[gvd.vertices[goal]] << endl;*/
      /* } */
    }
    // cout<<"fin del termino!"<<endl;
    return boost::make_tuple(shortest_paths, shortest_paths_costs);
  }
  // Find path to Graph
  template<typename CellType>
  boost::tuple<boost::unordered_map<Pos, Pos>, Pos> findPath(Pos source, Grid<CellType> grid, vector<CellType> notTraversable) {
    Float inf = numeric_limits<Float>::max(); 

    // initialize the dgrid and the distance queues
    Grid<Float> distGrid(grid.size(),inf);

    DistPosQueue dqueue, full_dqueue;
    dqueue.push(DistPos(0,source));
    distGrid[source] = 0;

    // Find path from p to graph
    boost::unordered_map<Pos, Pos> predecessor;
    while (!dqueue.empty()) {
      Pos p = dqueue.top().second;
      dqueue.pop();

      // Look at neighbors to explore compute its distances
      for (Pos np : grid.adj(p,notTraversable)) {
        if (distGrid[np] != inf) continue; // already processed

        // Compute np distance
        for (Pos nnp : grid.adj(np)) {
          if (distGrid[nnp] == inf) continue; // invalid distance, can safely avoid computation

          // find distance to neighbor's closest cell
          // that distance
          float d = np.distance_to(nnp) + distGrid[nnp];
          if (d < distGrid[np]) {
            distGrid[np] = d;
            predecessor[np] = nnp;
          }
        }

        // if np is on the graph then a path to the graph was found!
        if (this->vertices.find(np) != this->vertices.end()) {
          return boost::make_tuple(predecessor, np);
        }

        // Add neighbor to prossess
        dqueue.push(DistPos(distGrid[np], np));

      }
    }
    return boost::make_tuple(predecessor, NULL_POS);
  }
};
