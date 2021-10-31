#include "Gvd.h"

#include <math.h>
#include <cfloat>
#include <utility>
#include "DistMap.h"
#include "Map.h"
#include "data/Pos.h"

///////////////////
// Aux Functions //
///////////////////


// increase the sparseness a the graph by removing redundant edges:
//
//   * in this case redundant edges are the ones that from v lead to a neighbor
//     vN1 that can be reached from another neighbor of v vN2, meaning there is a
//     path v - vN1 and a path v - vN2 - vN1, so v - vN1 can be safely removed
//
//   * v is only considered for cleanUp if it has a neighbor with a greater or
//     equal degree one of those neighbors of greater or equal degree (then v and
//     v's neighbors) will keep the connection with v
void Gvd::cleanUp(Pos p, GvdGraph &graph, Int simplification, Int vertexRemoval) {
  if (simplification <= 0) return;

  GvdGraph::Vertex v = graph.idVertexMap[p];
 
  // get the vertex with max degree from the v and its neighbors
  GvdGraph::Vertex maxDegV = v;
  int maxDeg = graph.degree(v);
  for (GvdGraph::Vertex vN : graph.adj(v)) {
    int vNdegree = graph.degree(vN);
    if (vNdegree > maxDeg || (simplification > 1 && vNdegree == maxDeg)) {
      maxDegV = vN;
      maxDeg = vNdegree;
    }
  }
 
  if (maxDegV == v) return;
                              
  GvdGraph::AdjacencyIterator avIt, avItEnd;
  for (boost::tie(avIt, avItEnd) = adjacent_vertices(v, graph.g); avIt != avItEnd;) {
    GvdGraph::Vertex vN = *(avIt++);
 
    if (maxDegV == vN) continue; //skip max deg vertex
 
    if (edge(vN, maxDegV, graph.g).second) {
      graph.removeE(vN, v);
      graph.removeE(v,vN);
 
      if(graph.degree(vN)==1 && vertexRemoval){
        graph.removeE(vN,maxDegV);
        graph.removeE(maxDegV,vN);
      }
 
      if(graph.degree(v)==1 && vertexRemoval){
        graph.removeE(v,maxDegV);
        graph.removeE(maxDegV,v);
      }
 
      boost::tie(avIt, avItEnd) = adjacent_vertices(v, graph.g);
    }
  }
}

/* If removing `p` from  disconects the graph represented with the Grid then  */
template<typename CellType>
bool disconnectsOnRemoval(Pos p, Grid<CellType>& gridGraph) {
  vector<Pos> neighbor = gridGraph.neighborDisplacement;

  int res = 0;

  for (int i = 0; i < 8; i++) {
    Pos pN1 = p + neighbor[i];
    bool v1 = gridGraph.inside(pN1) && gridGraph[pN1];

    Pos pN2 =  p + neighbor[(i + 1) % 8];
    bool v2 = gridGraph.inside(pN2) && gridGraph[pN2];

    // if non diagonal
    if (neighbor[i].x == 0 || neighbor[i].y == 0) {
      Pos pN3 =   p + neighbor[(i + 2) % 8];
      bool v3 = gridGraph.inside(pN3) && gridGraph[pN3];
      res += v1 && !v2 && !v3;
    } else {
      res += v1 && !v2;
    }

    if (res > 1) break;
  }
  return res > 1;
}

// p has at least 2 obstacles as basis points  
bool isObstacleGenerated(Pos p, DistMap& distMap, Map& map){ 
  for(Pos bp1 : distMap.basisPoints(p)){
    if(map[bp1] == Unknown) return false;
  }
  return true;
}

// filter condition specific to the connectivity method applied
bool connectivityAux(Pos p, Map &map, DistMap &distMap){
  switch (GvdConfig::get()->connectivityMethod) {
    case 1:
      return !isObstacleGenerated(p,distMap,map);
      break;
    case 2:
    case 3:
      if (map[p] == Unknown) return true;
      return false;
      break;
  }
  return false;
}

bool Gvd::isConnectivityAux(Pos p){
  return connectivityAux(p, map, *distMap);
}

void Gvd::updateBase(PosSet &candidates) {
  PosSet toClean;

  // Set the candidates to belong to the GVD
  DistPosQueue gvdCandidatesQueue;
  for (Pos p : candidates){
    gridGvd[p] = true;
    gvdCandidatesQueue.push(make_pair((*distMap)[p].distance, p));
  }

  // Construct the gvd
  while (!gvdCandidatesQueue.empty()) {
    Float d = gvdCandidatesQueue.top().first;
    Pos p = gvdCandidatesQueue.top().second;
    gvdCandidatesQueue.pop();

    switch (GvdConfig::get()->vertexSimplificationMethod) {
      case 0:
        gridGvd[p] = true;
        break;
      case 1:
        // ver 1.6 (works and should work on corridors of pair width greater than 4 (and less than 4 too))
        int neighbors = 0;
        bool inNarrowPassage = true;
        for(Pos pN : map.adj(p,nonTraversables)){
          neighbors += gridGvd[pN];
          inNarrowPassage = inNarrowPassage && (*distMap)[pN].distance <= (*distMap)[p].distance + 0.5;
        }
        gridGvd[p] = ( ( (inNarrowPassage && neighbors == 2) || neighbors == 1) && existsNonAdjacent(distMap->basisPoints(p)) )  ||
                     existsNonAdjacent((*distMap)[p].sources)                                                                    || 
                     disconnectsOnRemoval(p, gridGvd);
        break;
    }

    if(gridGvd[p]){
      GvdGraph::Vertex v;
      bool inserted;
      tie(v,inserted) = graphGvd.addV(p);
      if(!inserted){
        graphGvd.removeV(v);
        tie(v,inserted) = graphGvd.addV(p);
      }

      toClean.insert(p);

      for(Pos pN : map.adj(p, nonTraversables)){
        if(graphGvd.has(pN)){
          /* toClean.insert(pN); */
          GvdGraph::Vertex vN  = graphGvd.idVertexMap[pN];
          graphGvd.addE(v,vN);
          graphGvd.addE(vN,v);
        }
      }
    }else {
      toClean.erase(p);
      graphGvd.removeV(p);

    }
  }

  for (Pos p : toClean){
    cleanUp(p, graphGvd, GvdConfig::get()->edgeSimplificationMethod, GvdConfig::get()->edgeSimplificationAllowVertexRemoval);
  }

  if(GvdConfig::get()->edgeSimplificationAllowVertexRemoval){
    for (Pos p : toClean){
      GvdGraph::Vertex v = graphGvd.idVertexMap[p];
      if (graphGvd.degree(v) == 0) {
        graphGvd.removeV(v);
        gridGvd[p] = false;
      }
    }
  }
}

/////////
// API //
/////////

// also updates its distMap
void Gvd::update(){
  chrono::steady_clock::time_point begin = std::chrono::steady_clock::now(); // start the timer

  // update dist map
  cout << "debug :: Update distMap" << endl;
  distMap->update();

  // pre update
  /// clean previous
  gridGvd = GridGvd(distMap->size(), false);
  graphGvd = GvdGraph();

  // update base
  updateBase(distMap->waveCrashes);

  // stop timer and set update time
  chrono::steady_clock::time_point end = chrono::steady_clock::now();
  float secOnNanosec = 1000000000;
  updateTime = (chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count())/secOnNanosec;

}

void Gvd::update(MapUpdatedCells &mapUpdatedCells){
  chrono::steady_clock::time_point begin = std::chrono::steady_clock::now(); // start the timer

  if(GvdConfig::get()->connectivityMethod==1){
      PosSet unknownBorder;
      for(auto it : mapUpdatedCells){
        Pos p = it.first;
        for(Pos pN : map.adj(p)){
          if(map[pN] == Unknown){
            unknownBorder.insert(pN);
          }
        }
      }
      for(Pos p :unknownBorder){
        mapUpdatedCells[p] = Free;
      }
  }

  // update dist map
  cout << "debug :: Update distMap" << endl;
  distMap->update(mapUpdatedCells);

  // pre update
  /// remove the vertices of the modified region 
  for (Pos p : distMap->modified){
    gridGvd[p] = false;
    graphGvd.removeV(p);
  }

  /// Set the wave crashes on the distance map which are candidates be added to the gvd
  PosSet candidates;
  for (Pos p : distMap->waveCrashes){
    candidates.insert(p);
    for(Pos pN : map.adj(p, nonTraversables)){ // allow the removal or edge clean of non useful neighbors
      if(gridGvd[pN]) candidates.insert(pN);
    }
  }

  // update base
  updateBase(candidates);

  // stop timer and set update time
  chrono::steady_clock::time_point end = chrono::steady_clock::now();
  float secOnNanosec = 1000000000;
  updateTime = (chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count())/secOnNanosec;
}

//////////////////
// Constructors //
//////////////////

Gvd::Gvd(MapType& map) : map(map){
  gridGvd = GridGvd (map.size(), false);

  switch (GvdConfig::get()->connectivityMethod) {
    case 0:
      sources         = {Occupied};
      nonTraversables = {Occupied,Unknown};

      break;
    case 1:
      sources         = {Occupied,Unknown};
      nonTraversables = {Occupied,Unknown};
      break;
    case 2:
    case 3:
      sources         = {Occupied};
      nonTraversables = {Occupied};
      break;
  }
  this->distMap = new DistMap(map, sources, nonTraversables);
}

/////////////////
// Destructors //
/////////////////

Gvd::~Gvd(){
  delete distMap;
}
