#pragma once

#include <bits/stdc++.h>
#include <iostream>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>

#include <boost/unordered_set.hpp>
#include <iterator>
#include <vector>

#include "Grid.h"
#include "Pos.h"

#include "../utils.h"

using namespace boost;

// README
// Copies of the graph definded here may not work.
//
// I think this is caused by the use of maps and vertex_descriptors to access
// vertices properties.
//
// The vertex_descriptors are stored in a map and then can be retrieved with an ID 
// I think vertex_descriptors are invalidated when the structure is copied.
//
// If you need to copy a graph the default copy constructor and the the default
// operator= were overrided to amend this issue. I tested both and currently
// use them but I'm not sure if they are 100% correct.
//
// TODO avoid duplicate code regarding operator= and copy constructor between them
// and withing classes

// Generic Boost Graph wrapper
template<typename graph, typename vertex_id>
struct Graph {
  typedef          graph GraphType;
  typedef          vertex_id VertexId;

  typedef typename graph_traits<GraphType>::vertex_descriptor Vertex;
  typedef typename GraphType::vertex_property_type VertexProperty;
  typedef typename graph_traits<GraphType>::edge_descriptor Edge;

  typedef typename boost::unordered_map<VertexId, Vertex> IdVertexMap;
  typedef typename IdVertexMap::iterator IdVertexMapIterator;

  typedef typename boost::unordered_map<Vertex, VertexId> VertexIdMap;
  typedef typename VertexIdMap::iterator VertexIdMapIterator;

  typedef typename graph_traits<GraphType>::vertex_iterator VertexIterator;
  typedef typename graph_traits<GraphType>::adjacency_iterator AdjacencyIterator;

  typedef typename graph_traits<GraphType>::edge_iterator EdgeIterator;

  GraphType g;
  VertexIdMap vertexIdMap;
  IdVertexMap idVertexMap;

  Graph(){}
  Graph(Graph& other){
    /* for(auto it : other.idVertexMap){ */ // not working properly
    /*   VertexId vId = it.first; */
    for(Vertex v : other){
      VertexId vId = other.vertexIdMap[v];
      /* cout<<vId<<endl; */

      Vertex thisV;
      bool inserted;
      tie(thisV, inserted) = this->addV(vId,other[v]);

      for(Vertex vN : other.adj(v)){
        VertexId vIdN = other.vertexIdMap[vN];

        // add it to the graph
        Vertex thisVN;
        bool inserted;
        tie(thisVN, inserted) = this->addV(vIdN,other[vN]);

        // and also add an edge connecting them
        this->addE(thisV, thisVN);
      }
    }
  } 

  Graph& operator=(Graph&& other) {
    this->idVertexMap = IdVertexMap();
    this->vertexIdMap = VertexIdMap();
    this->g           = GraphType();
    /* for(auto it : other.idVertexMap){ */ // not working properly
    /*   VertexId vId = it.first; */
    for(Vertex v : other){
      VertexId vId = other.vertexIdMap[v];
      /* cout<<vId<<endl; */

      Vertex thisV;
      bool inserted;
      tie(thisV, inserted) = this->addV(vId,other[v]);

      for(Vertex vN : other.adj(v)){
        VertexId vIdN = other.vertexIdMap[vN];

        // add it to the graph
        Vertex thisVN;
        bool inserted;
        tie(thisVN, inserted) = this->addV(vIdN,other[vN]);

        // and also add an edge connecting them
        this->addE(thisV, thisVN);
      }
    }
    return *this;
  }

  // Get node info associated with p
  VertexProperty& operator[](Vertex v){
    return g[v];
  }
  // Get node info associated with p
  VertexProperty& operator[](VertexId vid){
    return g[idVertexMap[vid]];
  }

  // add vertex associated with p
  boost::tuple<Vertex, bool> addV(VertexId vId) {
    return addV(vId,VertexProperty(vId));
  }
  boost::tuple<Vertex, bool> addV(VertexId vId, VertexProperty vP) {
    Vertex v;

    auto it = idVertexMap.find(vId);
    bool inserted = it == idVertexMap.end();
    /* bool inserted = !is_elem(vId,idVertexMap); */
    if (inserted) {
      v = add_vertex(vP,g);
      /* g[v] = VertexProperty(vId); */
      idVertexMap[vId] = v;
      vertexIdMap[v] = vId;
    } else {
      /* v = idVertexMap[vId]; */
      v = it->second;
    }
    return boost::make_tuple(v, inserted);
  }

  // remove vertex associated with vId (also the edges related with vId)
  void removeV(VertexId vId) {
    IdVertexMapIterator it = idVertexMap.find(vId);
    if (it != idVertexMap.end()) {
      Vertex v = it->second;

      clear_vertex(v, g);
      remove_vertex(v, g);

      vertexIdMap.erase(v);
      idVertexMap.erase(vId);
    }
  }
  // remove vertex associated with v (also the edges related with vId)
  void removeV(Vertex v) {
    VertexIdMapIterator it = vertexIdMap.find(v);
    if (it != vertexIdMap.end()) {
      VertexId vId = it->second;

      clear_vertex(v, g);
      remove_vertex(v, g);

      vertexIdMap.erase(v);
      idVertexMap.erase(vId);
    }
  }

  // add Edge associated to u and v
  pair<Edge, bool> addE(Vertex u, Vertex v, Float w = -1) {
    if (w == -1) {
      return add_edge(u, v, g);
    }
    return add_edge(u, v, w, g);
  }
  pair<Edge, bool> addE(VertexId vId, VertexId uId) {
    return addE(idVertexMap[vId], idVertexMap[uId]);
  }

  /* // add Edge associated to uId and vId */
  void removeE(VertexId vId, VertexId uId) { 
    return remove_edge(idVertexMap[vId], idVertexMap[uId], g);
  }
  void removeE(Vertex v, Vertex u) { 
    return remove_edge(v, u, g);
  }

  vector<Vertex> adj(Vertex v){
    vector<Vertex> res;
    AdjacencyIterator vIt, vItEnd;

    for (tie(vIt,vItEnd) = adjacent_vertices(v, g); vIt != vItEnd; ++vIt) {
      res.push_back(*vIt);
    }
    return res;
  }

  vector<VertexId> adj(VertexId vId){
    vector<VertexId> res;
    Vertex v = idVertexMap[vId];

    AdjacencyIterator vIt, vItEnd;
    for (tie(vIt,vItEnd) = adjacent_vertices(v, g); vIt != vItEnd; ++vIt) {
      res.push_back(vertexIdMap[*vIt]);
    }
    return res;
  }

  Int degree(Vertex v){
    return out_degree(v, g);
  }

  // check if vertex or vertexIdMap exists
  bool has(Vertex v){
    return is_elem(v,vertexIdMap);
  }

  bool has(VertexId vId){
    return is_elem(vId,idVertexMap);
  }

  VertexIterator begin(){
    VertexIterator vIt;
    VertexIterator vItEnd;
    boost::tie(vIt,vItEnd) = vertices(g);
    return vIt;
  }
  VertexIterator end(){
    VertexIterator vIt;
    VertexIterator vItEnd;
    boost::tie(vIt,vItEnd) = vertices(g);
    return vItEnd;
  }
};

// Boost Graph wrapper Focused on Graphs where the vertices are identified by a
// spatial location (repesented with the type Pos)
//
// graph (template parameter) VertexProperty must have a attribute p with time
// Pos and a constructor `VertexProperty(p)` 
//
// Adds A* funtions (Pos is hardcoded but in theory any class with a
// distanceTo method could be used)
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

  using typename ParentClass::VertexIdMap;
  using typename ParentClass::VertexIdMapIterator;

  using typename ParentClass::IdVertexMap;
  using typename ParentClass::IdVertexMapIterator;

  using typename ParentClass::VertexIterator;
  using typename ParentClass::AdjacencyIterator;

  using typename ParentClass::EdgeIterator;

  PosGraph(){}
  PosGraph(PosGraph& other){
    /* for(auto it : other.idVertexMap){ */ // not working properly
    /*   VertexId vId = it.first; */
    for(Vertex v : other){
      VertexId vId = other.vertexIdMap[v];
      /* cout<<vId<<endl; */

      Vertex thisV;
      bool inserted;
      tie(thisV, inserted) = this->addV(vId,other[v]);

      for(Vertex vN : other.adj(v)){
        VertexId vIdN = other.vertexIdMap[vN];

        // add it to the graph
        Vertex thisVN;
        bool inserted;
        tie(thisVN, inserted) = this->addV(vIdN,other[vN]);

        // and also add an edge connecting them
        Edge e;
        tie(e, inserted) = edge(v,vN,other.g); 
        this->addE(thisV, thisVN, get(edge_weight_t(), other.g, e));
      }
    }
  } 

  PosGraph& operator=(PosGraph&& other) {
    this->idVertexMap = IdVertexMap();
    this->vertexIdMap = VertexIdMap();
    this->g           = GraphType();
    /* for(auto it : other.idVertexMap){ */ // not working properly
    /*   VertexId vId = it.first; */
    for(Vertex v : other){
      VertexId vId = other.vertexIdMap[v];
      /* cout<<vId<<endl; */

      Vertex thisV;
      bool inserted;
      tie(thisV, inserted) = this->addV(vId,other[v]);

      for(Vertex vN : other.adj(v)){
        VertexId vIdN = other.vertexIdMap[vN];

        // add it to the graph
        Vertex thisVN;
        bool inserted;
        tie(thisVN, inserted) = this->addV(vIdN,other[vN]);

        Edge e;
        tie(e, inserted) = edge(v,vN,other.g); 
        this->addE(thisV, thisVN, get(edge_weight_t(), other.g, e));
      }
    }
    return *this;
  }

  // construct graph based on al boolean grid (true = belongs to GvdGraph)
  template<typename CellType>
  PosGraph(Grid<bool>& boolGrid, Grid<CellType> grid, vector<CellType> invalidTypes){
    // initialize grid_gvd
    for(Pos p : boolGrid){
      if (!boolGrid[p]) continue; // skip false

      // insert (x,y) to the graph
      Vertex u;
      bool inserted;
      tie(u, inserted) = this->addV(p);

      // for each neighbor (nx,ny) of (i,j)
      for(Pos pN : grid.adj(p, invalidTypes)){
        if (!boolGrid[pN]) continue; // skip false

        // add it to the graph
        Vertex v;
        bool inserted;
        tie(v, inserted) = this->addV(pN);
        // and also add an edge connecting them
        this->addE(u, v);
      }
    }
  };

  // pathfinding
  /// A* single
  //// euclidean distance heuristic
  template <class GraphType, class CostType, class LocMap>
  class single_astar_distance_heuristic : public astar_heuristic<GraphType, CostType> {
   public:
    typedef typename graph_traits<GraphType>::vertex_descriptor Vertex;
    single_astar_distance_heuristic(LocMap l, Vertex goal) : m_location(l), m_goal(goal) {}
    CostType operator()(Vertex u) { return m_location[u].p.distanceTo(m_location[m_goal].p); }

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
    Vertex start = this->idVertexMap[from];
    Vertex goal = this->idVertexMap[to];
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
    multi_astar_distance_heuristic(GraphType &g, PosSet goals) : m_graph(g), m_goals(goals) {}
    CostType operator()(Vertex u) {
      // cout<<"heuristica antes"<<m_goals.size()<<endl;
      Pos current_target = *(m_goals.begin());
      CostType distance = m_graph[u].p.distanceTo(current_target);
      // cout<<distance<<endl;
      auto it = m_goals.find(m_graph[u].p);
      if (it != m_goals.end() && m_goals.size() > 1) {
        m_goals.erase(it);
      }
      Pos new_target = *(m_goals.begin());
      // cout<<"heuristica desopues"<<m_goals.size()<<endl;
      return m_graph[u].p.distanceTo(new_target);
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
    void examine_vertex(Vertex u, GraphType &g) {
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

    filter(goals,[this](Pos goal){
                   return !is_elem(goal, this->idVertexMap);
                 });

    boost::unordered_map<Pos, list<Vertex>> shortest_paths;
    boost::unordered_map<Pos, float> shortest_paths_costs;

    vector<Vertex> p(num_vertices(this->g));
    vector<float>  d(num_vertices(this->g));

    Vertex startVertex = this->idVertexMap[start];

    try {
      // call astar named parameter interface
      astar_search_tree(this->g, startVertex,
                        multi_astar_distance_heuristic<GraphType, float>(this->g, goals),
                        predecessor_map(make_iterator_property_map(p.begin(), get(vertex_index, this->g)))
                            .distance_map(make_iterator_property_map(d.begin(), get(vertex_index, this->g)))
                            .visitor(multi_astar_goal_visitor<Vertex>(goals)));
    } catch (found_goals fg) {} // found a path to the goal

    // cout<<"comienzo del termino!"<<endl;
    for (Pos goal : goals) {
      Vertex goalVertex = this->idVertexMap[goal];

      list<Vertex> shortest_path;
      for (Vertex v = goalVertex ; p[v] != v ; v = p[v]) {
        shortest_path.push_front(v);
      }

      if(!shortest_path.empty() || goalVertex == startVertex){
        shortest_path.push_front(startVertex);
        shortest_paths[goal] = shortest_path;
        shortest_paths_costs[goal] = d[goalVertex];
      }
    }
    return boost::make_tuple(shortest_paths, shortest_paths_costs);
  }

  // Find path to Graph
  template<typename CellType>
  list<Pos> findPath(Pos source, Grid<CellType> &grid, vector<CellType> notTraversables) {
    // Initialize the distance grid
    Grid<Float> distGrid(grid.size(),INF);
    distGrid[source] = 0;

    // Initialize the distance queues 
    DistPosQueue openQueue;
    openQueue.push(DistPos(0,source));

    // Initialize the predecessor map
    boost::unordered_map<Pos, Pos> predecessor;
    predecessor[source] = NULL_POS;

    // Initialize empty path 
    list<Pos> path;

    // Find path from p to graph
    while (!openQueue.empty()) {
      Pos pos = openQueue.top().second;
      openQueue.pop();

      // if np is on the graph then a path to the graph was found!
      if (this->has(pos)) {

        // construct a vector with the nodes of the path
        Pos prevPos;
        do{
          // store the current position in the path list
          path.push_back(pos);

          // get the next position in the path from the graph to source
          prevPos = pos;
          pos = predecessor[pos];

          // check for error
          if(prevPos == pos)  FAIL("Loop on path. This is a Bug, halting execution.");

        } while (pos != NULL_POS);

        // check for error
        if(prevPos != source) FAIL("Path does not reach the source. This is a Bug, halting execution.");

        return path;
      }

      // Update neighbors distances
      for (Pos np : grid.adj(pos,notTraversables)) {
        // Compute np distance
        float d = distGrid[pos] + pos.distanceTo(np);
        if (d < distGrid[np]) {
          distGrid[np] = d;
          predecessor[np] = pos;
          openQueue.push(DistPos(distGrid[np], np));
        }
      }

    }
   
    // return empty pos
    return path;
  }

};
