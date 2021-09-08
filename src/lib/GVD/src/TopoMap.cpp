#include "TopoMap.h"
#include <iostream>
#include <vector>
#include "DistMap.h"
#include "Gvd.h"
#include "Map.h"
#include "utils.h"

/////////
// Aux //
/////////

// Sets to true the isLocalMin attribute of all the distance local minimum vertices.
//
// Local minimum here means:
//  * There is no neighbor with less distance
//  * There is at least one neighbor with greater distance
void setLocalMins(DistMap& distMap, GvdGraph& gvd) {
  PosSet notLocalMins;

  for (auto it : gvd.idVertexMap) {
    Pos p = it.first;
    GvdGraph::Vertex v = it.second;

    if (gvd[p].isLocalMin || is_elem(p,notLocalMins)) continue; // already processed

    bool isMin = true;
    bool hasGreater = false;

    for (Pos pN : gvd.adj(p)) {

      // p must have at least one neighbor with greater distance
      if (distMap[p].distance < distMap[pN].distance) { 
        hasGreater = true; // p has a neighbor with greater distance
        notLocalMins.insert(pN); // pN is not a min
      }

      // neighbor of p must be grater or equal
      switch (GvdConfig::get()->criticalConditionMin){
        case 0: isMin = isMin && distMap[p].distance < distMap[pN].distance;
        case 1: isMin = isMin && distMap[p].distance <= distMap[pN].distance;
      }

      if (!isMin) break;
    }
    if (isMin && hasGreater){
      gvd[p].isLocalMin = true;
    } // no need to add to notLocalMins, as this is only used to avoid extra processing 
    
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

/////////////
// TopoMap //
/////////////
TopoMap::TopoMap(Gvd* gvd) : map(gvd->map){
  this->gvd = gvd;
  this->distMap = gvd->distMap;
  this->segmenter = new ConnectedComponents(map, {Occupied,Unknown,Critical,CriticalLine});
}

TopoMap::TopoMap(MapType& map) : map(map){
  this->gvd = new Gvd(map);
  this->distMap = gvd->distMap;
  this->segmenter = new ConnectedComponents(map, {Occupied,Unknown,Critical,CriticalLine});
}

// also updates its distMap and gvd
void TopoMap::update(){
  // Clean old result
  criticalInfos.clear();

  // Get new result / replace old

  cout << "debug :: Update gvd" << endl;
  this->gvd->update(); // also updates the shared distMap

  cout << "debug :: Check local min constrain" << endl;
  setLocalMins(*distMap, *gvd->graphGvd);

  // Check degree constraint: optimize this using a collapsedGvd
  cout << "debug :: Check degree constraint" << endl;

  GvdGraph collapsedGvd(*gvd->graphGvd); // copy graph to collapse to avoid modifying the original

  collapseVertices(collapsedGvd); // collapse the copy

  degreeConstraint(collapsedGvd); // check the degree constrain in the collapsed gvd

  // set the collapsedGvd info in the complete gvd 
  // the following makes use of the fact that: collapsedGvd ⊂ gvd
  for(auto it : collapsedGvd.idVertexMap){
    Pos p = it.first;
    GvdGraph::Vertex v = it.second;
    (*gvd->graphGvd)[p] = collapsedGvd[v]; 
  }

  cout << "debug :: Set the critical vertices and its information" << endl;
  for(Pos p : map){
    // skip non critical vertices
    if(map[p] != Free || !(*gvd->graphGvd).has(p) || !(*gvd->graphGvd)[p].isLocalMin || !(*gvd->graphGvd)[p].degreeConstrain || connectivityAux(p,map,*distMap))
      continue;

    // Set critical in map
    map[p] = Critical;

    // Set criticals info
    CriticalInfo criticalInfo;
    for(Pos bp : basisPoints(p,*distMap)){
      // Set critical lines
      for(Pos linePos : discretizeLine(p,bp)){
        if(map[linePos] == Free){
          map[linePos] = CriticalLine;
          criticalInfo.criticalLines.insert(linePos);
        }
      }
    }
    criticalInfos[p] = criticalInfo;
  }

  segmenter->update();
}

TopoMap::~TopoMap(){
  delete distMap;
  if(gvd)
    gvd->distMap = NULL;
  delete gvd;
  delete segmenter;
}
