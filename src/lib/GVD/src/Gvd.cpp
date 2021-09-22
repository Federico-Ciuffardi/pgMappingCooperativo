#include "Gvd.h"

#include <math.h>
#include <cfloat>
#include <utility>
#include "DistMap.h"
#include "Map.h"
#include "data/Pos.h"

/////////
// Aux //
/////////

template<typename CellType>
int neighbors(Pos p, Grid<CellType>& gridGraph) {
  int res = 0;
  for(Pos pN : gridGraph.adj(p)){
    res += gridGraph[pN];
  }
  return res;
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
  int obstacleBasis = 0;
  for(Pos bp : basisPoints(p, distMap)){
    obstacleBasis += map[bp] == Occupied;
    if(obstacleBasis > 1) return true;
  }
  return false;
}

// filter condition specific to the connectivity method applied
bool connectivityAux(Pos p, Map &map, DistMap &distMap){
  switch (GvdConfig::get()->connectivityMethod) {
    case 1:
      return !isObstacleGenerated(p,distMap,map);
      break;
    case 2:
    case 3:
      // p is not unknown or a neighbor of unknown
      if (map[p] == Unknown) return true;
      for (Pos pn : map.adj(p)){
        if (map[pn] == Unknown) return true;
      } 
      return false;
      break;
  }
  return false;
}

bool Gvd::isConnectivityAux(Pos p){
  return connectivityAux(p, map, *distMap);
}

// returns a boolean matrix, a cell is true if it should belong to the GvdGraph
// and false otherwise
GridGvd getGridGvd(DistMap& distMap, Map& map, Int simplification) {
  // Initialize grid GVD
  GridGvd gridGvd(distMap.size(), false);

  // Set the candidates to belong to the GVD
  DistPosQueue gvdCandidatesQueue;
  for (Pos p : distMap.waveCrashPoss){
    gvdCandidatesQueue.push(make_pair(distMap[p].distance, p));
    gridGvd[p] = true;
  }

  // Simplify GVD
  while (!gvdCandidatesQueue.empty() && simplification > 0) {
    Float d = gvdCandidatesQueue.top().first;
    Pos p = gvdCandidatesQueue.top().second;
    gvdCandidatesQueue.pop();

    switch (simplification) {
      case 0:
        gridGvd[p] = true;
        break;
      case 1:
        gridGvd[p] =  distMap[p].sources.size() > 1 || disconnectsOnRemoval(p, gridGvd);
        break;
      case 2:
        gridGvd[p] =  (!connectivityAux(p,map,distMap) && distMap[p].sources.size() > 1) || disconnectsOnRemoval(p, gridGvd);
        break;
      case 3:
        gridGvd[p] = disconnectsOnRemoval(p, gridGvd); 
        break;
    }
  }
  return gridGvd;
}

void Gvd::updateGvdGraph(Int simplification) {

  for (Pos p : distMap->modified){
    gridGvd[p] = false;
    graphGvd->removeV(p);
  }

  // Set the adjacent to the waveCrashPoss that belong to the gvd to be removed if not needed
  for (Pos p : distMap->waveCrashPoss){
    for(Pos pN : map.adj(p,{Occupied})){
      if(gridGvd[pN]) distMap->waveCrashPoss.insert(pN);
    }
  }

  // Set the candidates to belong to the GVD
  DistPosQueue gvdCandidatesQueue;
  for (Pos p : distMap->waveCrashPoss){
    gridGvd[p] = true;
    gvdCandidatesQueue.push(make_pair((*distMap)[p].distance, p));
  }

  // Simplify GVD
  while (!gvdCandidatesQueue.empty()) {
    Float d = gvdCandidatesQueue.top().first;
    Pos p = gvdCandidatesQueue.top().second;
    gvdCandidatesQueue.pop();

    switch (simplification) {
      case 0:
        gridGvd[p] = true;
        break;
      case 1:
        gridGvd[p] =  existsNonAdjacent((*distMap)[p].sources) || disconnectsOnRemoval(p, gridGvd);
        break;
      case 2:
        gridGvd[p] = (!connectivityAux(p,map,*distMap) && (*distMap)[p].sources.size() > 1 && existsNonAdjacent(basisPoints(p,*distMap))) || disconnectsOnRemoval(p, gridGvd);
        break;
      case 3:
        gridGvd[p] = disconnectsOnRemoval(p, gridGvd); 
        break;
    }

    if(gridGvd[p]){
      GvdGraph::Vertex v;
      bool inserted;
      tie(v,inserted) = graphGvd->addV(p);
      if(inserted){
        for(Pos pN : map.adj(p)){
          if(graphGvd->has(pN)){
            GvdGraph::Vertex vN  = graphGvd->idVertexMap[pN];
            graphGvd->addE(v,vN);
            graphGvd->addE(vN,v);
          }
        }
      }
    }
  }
}


// increase the sparseness of the graph by removing redundant edges:
//
//   * in this case redundant edges are the ones that from v lead to a neighbor
//     vN1 that can be reached from another neighbor of v vN2, meaning there is a
//     path v - vN1 and a path v - vN2 - vN1, so v - vN1 can be safely removed
//
//   * v is only considered for cleanUp if it has a neighbor with a greater or
//     equal degree one of those neighbors of greater or equal degree (then v and
//     v's neighbors) will keep the connection with v
void cleanUp(GvdGraph& gvd, Int simplification, Int vertexRemoval) {
  if (simplification <= 0) return;

  for (GvdGraph::VertexIterator vIt = gvd.begin(); vIt != gvd.end();) {
    GvdGraph::Vertex v = *(vIt++);

    // get the vertex with max degree from the v and its neighbors
    GvdGraph::Vertex maxDegV = v;
    int maxDeg = gvd.degree(v);
    for (GvdGraph::Vertex vN : gvd.adj(v)) {
      int vNdegree = gvd.degree(vN);
      if (vNdegree > maxDeg || (simplification > 1 && vNdegree == maxDeg)) {
        maxDegV = vN;
        maxDeg = vNdegree;
      }
    }

    if (maxDegV == v) continue;
                                
    GvdGraph::AdjacencyIterator avIt, avItEnd;
    for (boost::tie(avIt, avItEnd) = adjacent_vertices(v, gvd.g); avIt != avItEnd;) {
      GvdGraph::Vertex vN = *(avIt++);

      if (maxDegV == vN) continue; //skip max deg vertex

      if (edge(vN, maxDegV, gvd.g).second) {
        gvd.removeE(vN, v);
        gvd.removeE(v,vN);

        if(gvd.degree(vN)==1 && vertexRemoval){
          gvd.removeE(vN,maxDegV);
          gvd.removeE(maxDegV,vN);
        }

        if(gvd.degree(v)==1 && vertexRemoval){
          gvd.removeE(v,maxDegV);
          gvd.removeE(maxDegV,v);
        }

        boost::tie(avIt, avItEnd) = adjacent_vertices(v, gvd.g);
      }
    }
  }
  if(vertexRemoval){
    for (GvdGraph::VertexIterator vIt = gvd.begin(); vIt != gvd.end();) {
      GvdGraph::Vertex v = *(vIt++);
      if (gvd.degree(v) == 0) {
        gvd.removeV(v);
      }
    }
  }
}

/////////
// Gvd //
/////////
Gvd::Gvd(DistMap* distMap) : map(distMap->map){
  this->distMap = distMap;
}

Gvd::Gvd(MapType& map) : map(map){
  cout<<"connectivityMethod: "<<GvdConfig::get()->connectivityMethod<<endl;
  switch (GvdConfig::get()->connectivityMethod) {
    case 0:
      this->distMap = new DistMap(map,{Occupied},{Occupied,Unknown});
      break;
    case 1:
      this->distMap = new DistMap(map,{Occupied,Unknown},{Occupied,Unknown});
      break;
    case 2:
    case 3:
      this->distMap = new DistMap(map,{Occupied},{Occupied});
      break;
  }
  gridGvd = GridGvd (distMap->size(), false);
}

// also updates its distMap
void Gvd::update(){
  // if connectivityMethod is 2 fill boders with obstacles
  if(GvdConfig::get()->connectivityMethod == 2){
    pair<Int,Int> size = map.size();
    for(int x = 0; x < size.first; x++){
      map[x][0] = Occupied;
      map[x][size.second - 1] = Occupied;
    }
    for(int y = 0; y < size.second; y++){
      map[0][y] = Occupied;
      map[size.first - 1][y] = Occupied;
    }
  }

  // Clean old result
  if(graphGvd) delete graphGvd;

  // Get new result / replace old
  cout << "debug :: Update distMap" << endl;
  distMap->update();
  cout << "debug :: Generate gridGVD" << endl;
  gridGvd = getGridGvd(*distMap, map, GvdConfig::get()->vertexSimplificationMethod);
  cout << "debug :: Generate graphGVD" << endl;
  graphGvd = new GvdGraph(gridGvd, map, distMap->nonTraversables);
  cout << "debug :: cleanUp" << endl;

  cleanUp(*graphGvd, GvdConfig::get()->edgeSimplificationMethod, GvdConfig::get()->edgeSimplificationAllowVertexRemoval);

}

void Gvd::update(MapUpdatedCells mapUpdatedCells){
  if(!graphGvd){
    graphGvd = new GvdGraph();
  }

  // Get new result / replace old
  cout << "debug :: Update distMap" << endl;
  distMap->update(mapUpdatedCells);
  cout << "debug :: updateGvdGraph" << endl;
  updateGvdGraph(GvdConfig::get()->vertexSimplificationMethod);

  cleanUp(*graphGvd, GvdConfig::get()->edgeSimplificationMethod, GvdConfig::get()->edgeSimplificationAllowVertexRemoval);

}

Gvd::~Gvd(){
  delete distMap;
}
