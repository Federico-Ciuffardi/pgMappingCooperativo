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

bool isValid(CellState cellState){
  return cellState != Frontier && cellState != Occupied && cellState != Unknown;
}

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
bool satisfiesDegreeConstraint(GvdGraph& gvd, GvdGraph::Vertex v) {
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
  for( auto it : criticalInfos ){
    Pos critical = it.first;
    CriticalInfo criticalInfo = it.second;
    map[critical] = Free;
    for(Pos p : criticalInfo.criticalLines){
      map[p] = Free;
    }
  }
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
  for(GvdGraph::Vertex v : graphGvd){
    Pos p = graphGvd[v].p;

    // skip vertices that:
    if(!isValid(map[p])                              || // cellState invalid
       !graphGvd[v].isLocalMin                       || // are not local min
        connectivityAux(p,map,*distMap)              || // are connectivityAux
       !satisfiesDegreeConstraint(*gvd->graphGvd, v)  ) // do not satisfy degreeConstrain
      continue;

    // Filter critical lines
    CriticalInfo criticalInfo;
    boost::unordered_set<Pos> pBasisPoints     = distMap->basisPoints(p);
    boost::unordered_set<Pos> criticalLineEnds = pBasisPoints;

    for(Pos bp1 : pBasisPoints){
      if(map[bp1] == Unknown){ 
        criticalLineEnds.erase(bp1);
        continue;
      }
      for(Pos bp2 : pBasisPoints){
        if(bp2 == bp1) continue;

        Pos p_bp1 = bp1-p;
        Pos p_bp2 = bp2-p;

        if(map[bp2] == Unknown || p_bp1.angle_to(p_bp2) < M_PI/1.5){
          criticalLineEnds.erase(bp2);
          continue;
        } 
      }
    }

    // Skip if less than 2 critical lines left
    if(criticalLineEnds.size() < 2) continue;

    for(Pos endPoint : criticalLineEnds){
      for(Pos linePos : discretizeLine(p,endPoint)){
        if(isValid(map[linePos])){
          map[linePos] = CriticalLine;
          criticalInfo.criticalLines.insert(linePos);
        }
      }
    }

    criticalInfos[p] = criticalInfo;

    // Set critical in map
    map[p] = Critical;

  }


  segmenter->update();
}

void TopoMap::update(MapUpdatedCells mapUpdatedCells){
  // Clean old result
  for( auto it : criticalInfos ){
    Pos critical = it.first;
    CriticalInfo criticalInfo = it.second;
    map[critical] = Free;
    for(Pos p : criticalInfo.criticalLines){
      map[p] = Free;
    }
  }
  criticalInfos.clear();

  // Get new result / replace old
  cout << "debug :: Update gvd" << endl;
  gvd->update(mapUpdatedCells); // also updates the shared distMap
  GvdGraph &graphGvd = *gvd->graphGvd;
  
  cout << "debug :: Calculate isLocalMin for each vertex gvd" << endl;
  for (GvdGraph::Vertex v : graphGvd) {
    auto &vertexInfo = graphGvd[v];
    vertexInfo.isLocalMin = isLocalMin(*distMap, graphGvd, vertexInfo.p);
  }

  cout << "debug :: Set the critical vertices and its information" << endl;
  for(GvdGraph::Vertex v : graphGvd){
    Pos p = graphGvd[v].p;

    // skip vertices that:
    if(!isValid(map[p])                              || // cellState invalid
       !graphGvd[v].isLocalMin                       || // are not local min
        connectivityAux(p,map,*distMap)              || // are connectivityAux
       !satisfiesDegreeConstraint(*gvd->graphGvd, v)  ) // do not satisfy degreeConstrain
      continue;

    // Filter critical lines
    CriticalInfo criticalInfo;
    boost::unordered_set<Pos> pBasisPoints     = distMap->basisPoints(p);
    boost::unordered_set<Pos> criticalLineEnds = pBasisPoints;

    for(Pos bp1 : pBasisPoints){
      if(map[bp1] == Unknown){ 
        criticalLineEnds.erase(bp1);
        continue;
      }
      for(Pos bp2 : pBasisPoints){
        if(bp2 == bp1) continue;

        Pos p_bp1 = bp1-p;
        Pos p_bp2 = bp2-p;

        if(map[bp2] == Unknown || p_bp1.angle_to(p_bp2) < M_PI/1.5){
          criticalLineEnds.erase(bp2);
          continue;
        } 
      }
    }

    // Skip if less than 2 critical lines left
    if(criticalLineEnds.size() < 2) continue;

    for(Pos endPoint : criticalLineEnds){
      for(Pos linePos : discretizeLine(p,endPoint)){
        if(isValid(map[linePos])){
          map[linePos] = CriticalLine;
          criticalInfo.criticalLines.insert(linePos);
        }
      }
    }

    criticalInfos[p] = criticalInfo;

    // Set critical in map
    map[p] = Critical;

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
