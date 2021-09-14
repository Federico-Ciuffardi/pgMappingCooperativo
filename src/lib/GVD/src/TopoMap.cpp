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

// retruns true if the vertex associated with p is a distance local minimum vertex.
//
// Local minimum here means:
//  * There is no neighbor with less distance
//  * There is at least one neighbor with greater distance
bool isLocalMin(DistMap& distMap, GvdGraph& gvd, Pos p) {
  bool isMin = true;
  bool hasGreater = false;

  for (Pos pN : gvd.adj(p)) {

    // p must have at least one neighbor with greater distance
    if (distMap[p].distance < distMap[pN].distance) { 
      hasGreater = true; // p has a neighbor with greater distance
    }

    // neighbor of p must be grater or equal
    switch (GvdConfig::get()->criticalConditionMin){
      case 0: isMin = isMin && distMap[p].distance < distMap[pN].distance;
      case 1: isMin = isMin && distMap[p].distance <= distMap[pN].distance;
    }

    if (!isMin) break;
  }
  return isMin && hasGreater;
}

// Return true if there is a path from `prevV` that:
// * Includes `v` and reaches a
// * Ends on a vertex of degree 3 
// * Does not contain a critial vertex candidate meaning a local min vertex
//   (see setLocalMins for local min definition) 
bool degreeConstraintAux(GvdGraph& gvd, GvdGraph::Vertex& prevV, GvdGraph::Vertex& v){
  if (gvd.degree(v) >= 3){ // neighbor of degree 3 or greater
    return true;
  } else if (gvd.degree(v) == 1 || gvd[v].isLocalMin){ // path end or another candidate
    return false;
  } else{
    for (GvdGraph::Vertex vN : gvd.adj(v)) {
      if(vN != prevV){
        return degreeConstraintAux(gvd, v, vN);
      }
    }
  }
  cout<< "WARNING: degreeConstraintAux bad base case reached"<<endl;
  return false;
}

// retruns true if the vertex associated with p satisfies the degreeConstrain and false otherwise
// The degree constrain is satisfied if:
// * A vertex has degree 2 
// * Is a local min
// * has a path with the characteristics described on the `degreeConstraintAux`
//   function
// Precondition: gvd has isLocalMin stablished for each vertex
bool satisfiesDegreeConstraint(GvdGraph& gvd, Pos p) {
  int criticalsCount = 0;

  GvdGraph::Vertex v = gvd.idVertexMap[p];

  // Vertex must have a degree of 2
  if (gvd.degree(v) != 2) return false;

  // The vertex must have a neighbor of degree 3 
  for (GvdGraph::Vertex vN : gvd.adj(v)) {
    if(degreeConstraintAux(gvd, v, vN)) return true;
  }

  return false;
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
  gvd->update(); // also updates the shared distMap
  GvdGraph &graphGvd = *gvd->graphGvd;
  
  cout << "debug :: Calculate isLocalMin for each vertex gvd" << endl;
  for (GvdGraph::Vertex v : graphGvd) {
    auto &vertexInfo = graphGvd[v];
    vertexInfo.isLocalMin = isLocalMin(*distMap, graphGvd, vertexInfo.p);
  }

  cout << "debug :: Set the critical vertices and its information" << endl;
  for(Pos p : map){
    // skip non critical vertices
    if(map[p] != Free || !(*gvd->graphGvd).has(p) || !graphGvd[p].isLocalMin || connectivityAux(p,map,*distMap) || !satisfiesDegreeConstraint(*gvd->graphGvd, p))
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
