#include "TopoMap.h"
#include <iostream>
#include <vector>
#include "DistMap.h"
#include "Gvd.h"
#include "Map.h"
#include "utils.h"

// Sets to true the isLocalMin attribute of all the distance local minimum vertices.
//
// Local minimum here means:
//  * There is no neighbor with less distance
//  * There is at least one neighbor with greater distance
void setLocalMins(DistMap& dg, GvdGraph& gvd) {
  PosSet notLocalMins;

  for (auto it : gvd.idVertexMap) {
    Pos p = it.first;
    GvdGraph::Vertex v = it.second;

    if (gvd[p].isLocalMin || is_elem(p,notLocalMins)) continue; // already processed

    bool isMin = true;
    bool hasGreater = false;

    for (Pos pN : gvd.adj(p)) {

      // p must have at least one neighbor with greater distance
      if (dg[p].distance < dg[pN].distance) { 
        hasGreater = true; // p has a neighbor with greater distance
        notLocalMins.insert(pN); // pN is not a min
      }

      // neighbor of p must be grater or equal
      isMin = isMin && dg[p].distance <= dg[pN].distance; 

      if (!isMin) break;
    }
    if (isMin && hasGreater){
      gvd[p].isLocalMin = true;
    } // no need to add to notLocalMins, as this is only used to avoid extra processing 
    
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
void cleanUp(GvdGraph& gvd) {
  for (GvdGraph::VertexIterator vIt = gvd.begin(); vIt != gvd.end();) {
    GvdGraph::Vertex v = *(vIt++);

    // get the vertex with max degree from the v and its neighbors
    GvdGraph::Vertex maxDegV = v;
    int maxDeg = gvd.degree(v);
    for (GvdGraph::Vertex vN : gvd.adj(v)) {
      int vNdegree = gvd.degree(vN);
      if (vNdegree >= maxDeg) {
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

        if(gvd.degree(vN)==1){
          gvd.removeE(vN,maxDegV);
          gvd.removeE(maxDegV,vN);
        }

        if(gvd.degree(v)==1){
          gvd.removeE(v,maxDegV);
          gvd.removeE(maxDegV,v);
        }

        boost::tie(avIt, avItEnd) = adjacent_vertices(v, gvd.g);
      }
    }
  }
  for (GvdGraph::VertexIterator vIt = gvd.begin(); vIt != gvd.end();) {
    GvdGraph::Vertex v = *(vIt++);
    if (gvd.degree(v) == 0) {
      gvd.removeV(v);
    }
  }
}


bool sameDirecction(Pos p1, Pos p2) {
  return p1.normalize() == -p2.normalize();
}

// Simplify the graph without removing spatial information.
//
//   Remove all the non critical candidates of degree 2 that lie within a
//   straight line of vertices and are not the ends. The removed vertices are
//   represented with an edge connecting the preserved ends of the line.
void collapseVertices(GvdGraph& gvd) {
  for (GvdGraph::VertexIterator vIt = gvd.begin(); vIt != gvd.end();) {
    GvdGraph::Vertex v = *(vIt++);

    Pos p = gvd[v].p;

    bool isMin = gvd[v].isLocalMin;
    if (!isMin && gvd.degree(v) == 2) {
      vector<GvdGraph::Vertex> adj = gvd.adj(v);

      auto adj1 = gvd[adj[0]];
      Pos pToAdj1 = adj1.p - p;

      auto adj2 = gvd[adj[1]];
      Pos pToAdj2 = (adj2.p - p);

      if (sameDirecction(pToAdj1, pToAdj2)) {
        gvd.addE(adj[0], adj[1]);
        gvd.addE(adj[1], adj[0]);
        gvd.removeV(v);
      }
    }
  }
}

// Return true if there is a path from `prevV` that:
// * Includes `v` and reaches a
// * Ends on a vertex of degree 3 
// * Does not contain a critial vertex candidate meaning a local min vertex
//   (see setLocalMins for local min definition) 
bool degreeConstraintAux(GvdGraph& gvd,GvdGraph::Vertex& prevV, GvdGraph::Vertex& v){
  if (gvd.degree(v) >= 3){ // neighbor of degree 3 or greater
    return true;
  } else if (gvd.degree(v) == 1 || gvd[v].isLocalMin){ // path end or another candidate
    return false;
  } else{
    for (GvdGraph::Vertex vN : gvd.adj(v)) {
      if(vN != prevV){
        return degreeConstraintAux(gvd,v,vN);
      }
    }
  }
  cout<< "WARNING: degreeConstraintAux bad base case reached"<<endl;
  return false;
}

// Sets to true the degreeConstrain attribute of all the vertices that satisfy it.
// The degree constrain is satisfied if:
// * A vertex has degree 2 
// * Is a local min
// * has a path with the characteristics described on the `degreeConstraintAux`
//   function
void degreeConstraint(GvdGraph& gvd) {
  int criticalsCount = 0;

  for (auto it : gvd.idVertexMap) {
    Pos p = it.first;
    GvdGraph::Vertex v = it.second;

    // skip does not complies with some of the requirmets already
    if (gvd.degree(v) != 2 || !gvd[p].isLocalMin) continue;

    // check if it has a neighbor of degree 3 
    for (GvdGraph::Vertex vN : gvd.adj(v)) {
      if(degreeConstraintAux(gvd,v,vN)){
        gvd[v].degreeConstrain = true;
        criticalsCount++;
        break;
      }
    }
  }

  if (criticalsCount > 0) return;

  // No real critical point found, create an artificial one representing the
  // hole space as one segment

  int maxDeg = -1;
  Pos maxPos = NULL_POS;

  for (auto it : gvd.idVertexMap) {
    Pos p = it.first;
    GvdGraph::Vertex v = it.second;

    int currentDegree = gvd.degree(v);
    if (currentDegree > maxDeg) {
      maxDeg = currentDegree;
      maxPos = p;
    }
  }

  if(maxPos != NULL_POS){
     gvd[maxPos].degreeConstrain = true;
  }

}

// Returns two maps : criticals -> frontiers, criticals-> min dis to frontier, segments gvd
CriticalInfos unknownDistConstraint(StateGrid& stateGrid, GvdGraph& gvd, DistMap& dg) {
  CriticalInfos res;
  cout << "debug :: Get the nearest frontiers to each critical" << endl;

  DistMap distMap(stateGrid,{Critical},{Unknown,Occupied}, {Frontier});
  distMap.update();

  cout << "debug :: set each frontier to a critical" << endl;
  while (!distMap.objectiveDQueue.empty()) {
    DistPos frontier_dp = distMap.objectiveDQueue.top();
    distMap.objectiveDQueue.pop();
    Pos frontier = frontier_dp.second;

    vector<Pos> frontierCrits = toVec(distMap[frontier].sources);

    for (int i = 0; i < frontierCrits.size(); i++) {
      Pos critical_Pos = frontierCrits[i];
      GvdVertexProperty& c = gvd[critical_Pos];
      if (!c.is_critical) {
        c.is_critical = true;
        // c.segment = critical_Pos;
        res[critical_Pos].mindToF = frontier_dp.first;
        res[critical_Pos].frontiers.push_back(frontier);
        // frontier_crits.clear();
        frontierCrits[0] = critical_Pos;
        break;
      }
      if (i == (frontierCrits.size() - 1)) {  // i is the last one
        res[critical_Pos].frontiers.push_back(frontier);
        // frontier_crits.clear();
        frontierCrits[0] = critical_Pos;
      }
    }
  }
  cout << "set the segment for every vertex" << endl;
  GvdGraph::VertexIterator v_it, v_it_end;
  for (tie(v_it, v_it_end) = vertices(gvd.g); v_it != v_it_end; v_it++) {
    GvdVertexProperty& v = gvd[*v_it];
    if (distMap[v.p].sources.size() > 0) {
      v.segment = *distMap[v.p].sources.begin();
    } else {
      v.segment = Pos(INT_MIN, INT_MIN);
      cout << "warning vertex without segment" << endl;
    }
  }
  cout << "Done" << endl;

  return res;
}

void setCriticals(StateGrid& stateGrid, GvdGraph& gvd, DistMap& dg){
}

CriticalInfos get_critical_points(StateGrid& stateGrid, DistMap& dg, GvdGraph& gvd) {
  cout << "debug :: cleanUp" << endl;
  cleanUp(gvd);

  cout << "debug :: setLocalMins" << endl;
  setLocalMins(dg, gvd);

  // collapsedGvd graph (used to save some time on the degreeConstrain
  cout << "debug :: copy gvd" << endl;
  GvdGraph collapsedGvd(gvd);

  cout << "debug :: collapseVertices" << endl;
  collapseVertices(collapsedGvd);

  cout << "debug :: degreeConstraint" << endl;
  degreeConstraint(collapsedGvd);

  // pass info to complete gvd 
  // collapsedGvd ⊂ gvd
  for(auto it : collapsedGvd.idVertexMap){
    Pos p = it.first;
    GvdGraph::Vertex v = it.second;
    gvd[p] = collapsedGvd[v]; 
  }

  // Set the critical points on the state grid
  for(Pos p : stateGrid){
    if(gvd.has(p) && gvd[p].isLocalMin && gvd[p].degreeConstrain && isObstacleGenerated(p,dg,stateGrid)){
      stateGrid[p] = Critical;
    }
  }

  cout << "debug :: unknownDistConstraint" << endl;
  CriticalInfos cis = unknownDistConstraint(stateGrid, gvd, dg);

  setCriticals(stateGrid,gvd,dg);

  return cis;
}

TopoMap::TopoMap(Gvd* gvd) : map(gvd->map){
  this->gvd = gvd;
  this->distMap = gvd->distMap;
}

TopoMap::TopoMap(MapType& map) : map(map) {
  this->gvd = new Gvd(map);
  this->distMap = gvd->distMap;
}

/* boost::tuple<criticals_info, GvdGraph> get_points_of_interest(StateGrid stateGrid) { */
void TopoMap::update(){
  // Clean old result
  cis.clear();

  // Get new result / replace old
  this->gvd->update(); // also updates the shared distMap
  cout << "debug :: Calculate ciritical points" << endl;
  this->cis = get_critical_points(map, *distMap, *gvd->graphGvd);
}

TopoMap::~TopoMap(){
  delete distMap;
  if(gvd)
    gvd->distMap = NULL;
  delete gvd;
}
