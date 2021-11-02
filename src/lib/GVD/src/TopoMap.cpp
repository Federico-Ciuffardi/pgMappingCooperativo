#include "TopoMap.h"
#include <iostream>
#include <utility>
#include <vector>
#include "DistMap.h"
#include "Gvd.h"
#include "Map.h"
#include "utils.h"

///////////////////
// Aux Functions //
///////////////////

bool isValid(CellState cellState){
  return cellState != Frontier && cellState != Occupied && cellState != Unknown;
}

bool TopoMap::minAngleConstrain(DistMap& distMap, Pos p, boost::unordered_map<Pos,pair<Pos,Pos>> &criticalLine) {

  Float maxAngle = minCriticalLineAngle;

  Pos endPoint1 = NULL_POS;
  Pos endPoint2 = NULL_POS;

  PosSet pBasisPoints = distMap.basisPoints(p);
  for(Pos bp1 : pBasisPoints){
    if(map[bp1] == Unknown) continue;

    for(Pos bp2 : pBasisPoints){
      if(bp2 == bp1 || map[bp2] == Unknown) continue;

      Pos p_bp1 = bp1-p;
      Pos p_bp2 = bp2-p;

      Float angle = p_bp1.angle_to(p_bp2);
      if(angle >= maxAngle){
        maxAngle = angle;
        endPoint1 = bp1;
        endPoint2 = bp2;
      } 
    }
  }

  if( endPoint1 != NULL_POS ){
    criticalLine[p] = make_pair(endPoint1,endPoint2);
    return true;
  }else{
    return false;
  }
}

// retruns true if the vertex associated with p is a distance local minimum vertex.
//
// The definition of local minimum depends on the criticalConditionMin value (check GvdConfig.h)
bool localMinConstrain(DistMap& distMap, GvdGraph& gvd, Pos p) {
  bool isMin = true;
  bool hasGreater = false;

  for (Pos pN : gvd.adj(p)) {

    // p must have at least one neighbor with greater distance
    if (distMap[p].distance < distMap[pN].distance) { 
      hasGreater = true; // p has a neighbor with greater distance
    }

    // Is local min minimum
    switch (GvdConfig::get()->criticalConditionMin){
      case 0: isMin = distMap[p].distance < distMap[pN].distance;
      case 1: isMin = distMap[p].distance <= distMap[pN].distance;
    }

    if (!isMin) break;
  }
  return isMin && hasGreater;
}

// Return true if there is a path from `prevV` that:
// * Includes `v`
// * Ends on a vertex of degree 3 
// * Does not contain a critical vertex candidate (excluding `v`)
bool degreeConstraintAux(GvdGraph& gvd, GvdGraph::Vertex prevV, GvdGraph::Vertex v){
  while(true){
    int degree = gvd.degree(v);
    if (degree >= 3){ // neighbor of degree 3 or greater
      return true;
    } else if (degree == 1 || gvd[v].isCandidate){ // path end or another candidate
      return false;
    } else{
      for (GvdGraph::Vertex vN : gvd.adj(v)) {
        if(vN != prevV){
          prevV = v;
          v = vN;
          break;
        }
      }
    }
  }
}

// retruns true if the vertex associated with p satisfies the degreeConstrain and false otherwise
// The degree constrain is satisfied if:
// * A vertex has degree 2 (precondition for this function)
// * has a path with the characteristics described on the `degreeConstraintAux` function
// Preconditions: (1) gvd has isCandidate stablished for each vertex, (2) the vertex v has degree 2
bool satisfiesDegreeConstraint(GvdGraph& gvd, GvdGraph::Vertex v) {

  // The vertex must have a neighbor of degree 3 
  for (GvdGraph::Vertex vN : gvd.adj(v)) {
    if(degreeConstraintAux(gvd, v, vN)) return true;
  }

  return false;
}

void TopoMap::updateBase(PosSet &preCandidates){
  GvdGraph &graphGvd = gvd->graphGvd;

  boost::unordered_map<Pos,pair<Pos,Pos>> criticalLine;
  PosSet candidates;
  cout << "debug :: Calculate which vertices are candidates" << endl;
  for( Pos p : preCandidates ){
    if(!graphGvd.has(p)) continue; // skip if p is not in the gvd

    GvdGraph::Vertex v = graphGvd.idVertexMap[p];
    auto &vertexInfo = graphGvd[v];

    vertexInfo.isCandidate = isValid(map[p])                              && // cellState valid
                             graphGvd.degree(v) == 2                      && // has degree 2
                             !connectivityAux(p,map,*distMap)             && // are not connectivityAux
                             minAngleConstrain(*distMap, p, criticalLine) && // satisfies minAngleConstrain
                             localMinConstrain(*distMap, graphGvd, p)      ; // satisfies localMinConstrain 

    if(vertexInfo.isCandidate){
      candidates.insert(p);
    }
  }

  cout << "debug :: Set the critical vertices and its information" << endl;
  for( Pos p : candidates ){
    GvdGraph::Vertex v = graphGvd.idVertexMap[p];

    // this loop is separated from the previous one to check the degreeConstrain after all the candidates were determined 
    // TODO make it work incrementally 
    /* if(!satisfiesDegreeConstraint(gvd->graphGvd, v)) */ 
    /*   continue; */

    // set criticalLines
    boost::unordered_set<Pos> criticalLineEnds;

    criticalLineEnds.insert(criticalLine[p].first);
    criticalLineEnds.insert(criticalLine[p].second);

    CriticalInfo criticalInfo;
    for(Pos endPoint : criticalLineEnds){
      for(Pos linePos : discretizeLine(p,endPoint)){
        if(linePos == p || linePos == endPoint) continue;

        if(isValid(map[linePos])){
          map[linePos] = CriticalLine;
        }

        if(!is_elem(map[linePos], distMap->sources)){
          criticalInfo.criticalLines.insert(linePos);
          criticalLines[linePos]++;
        }
      }
    }

    criticalInfos[p] = criticalInfo;

    // Set critical in map
    map[p] = Critical;
  }

  cout << "debug :: Set segments" << endl;
  segmenter->update();
}

/////////
// API //
/////////

void TopoMap::update(){
  // update the gvd and the dist map
  cout << "debug :: Update gvd" << endl;
  gvd->update(); 

  // get the candidate cells to change their critical status
  PosSet candidates;
  for(auto &it : gvd->graphGvd.idVertexMap){
    candidates.insert(it.first);
  }

  // Clean old result
  for( auto it : criticalInfos ){
    Pos critical = it.first;
    CriticalInfo criticalInfo = it.second;
    if(map[critical] == Critical) map[critical] = Free;
    for(Pos linePos : criticalInfo.criticalLines){
      if(map[linePos] == CriticalLine) map[linePos] = Free;
      criticalLines.erase(linePos);
    }
  }
  criticalInfos.clear();

  // update base
  updateBase(candidates);
}

void TopoMap::update(MapUpdatedCells &mapUpdatedCells){
  // update the gvd and the dist map
  cout << "debug :: Update gvd" << endl;
  gvd->update(mapUpdatedCells); 
  
  // get the candidate cells to change their critical status
  PosSet candidates;
  for(Pos p : distMap->modified){
    candidates.insert(p);
    for( Pos pN : map.adj(p)){
      candidates.insert(pN);
    }
  }

  for(auto &it : mapUpdatedCells){
    Pos p = it.first;
    candidates.insert(p);

    // if is valid then restore the value
    if(isValid(map[p])){
      if(is_elem(p, criticalInfos)){
        map[p] = Critical;
      }else if(is_elem(p, criticalLines)){
        map[p] = CriticalLine;
      }
    }
  }

  // Clean old result
  for( Pos pos : candidates ){
    if(is_elem(pos,criticalInfos)){
      CriticalInfo criticalInfo = criticalInfos[pos];
      if(map[pos] == Critical) map[pos] = Free;
      for(Pos linePos : criticalInfo.criticalLines){
        criticalLines[linePos]--;
        if(criticalLines[linePos] <= 0){
          if(map[linePos] == CriticalLine) map[linePos] = Free;
          criticalLines.erase(linePos);
        }
      }
      criticalInfos.erase(pos);
    }
  }

  // update base
  updateBase(candidates);
}

//////////////////
// Constructors //
//////////////////

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

/////////////////
// Destructors //
/////////////////

TopoMap::~TopoMap(){
  delete distMap;
  if(gvd)
    gvd->distMap = NULL;
  delete gvd;
  delete segmenter;
}
