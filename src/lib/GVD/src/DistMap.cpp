#include "DistMap.h"
#include <boost/unordered/unordered_set_fwd.hpp>
#include <cmath>
#include <limits>
#include "Map.h"
#include "data/Pos.h"
#include "utils.h"

/////
/// DistCell
//

///////////////
// Operators //
///////////////

bool DistMap::DistCell::operator>(const DistMap::DistCell& d) const {
  return this->distance > d.distance;
}

bool DistMap::DistCell::operator==(const DistMap::DistCell& d) const {
  return this->distance == d.distance;
}

ostream& operator<<(ostream& out, const DistMap::DistCell& cell) {
  // out<<"("<< cell.distance<<", "<< cell.sources.size() << " )";
  if(cell.distance == INF){
    out << " ∞,"<<cell.sources.size();
  }else if (cell.distance != 0) {
    int intDist = floor(cell.distance);
    if(cell.distance == intDist){
      out << " " << intDist<<","<<cell.sources.size();
    }else{
      out << "~" << intDist<<","<<cell.sources.size();
    }
  } else {
    out << "████";
  }
  return out;
}

DistMap::DistMapType::reference DistMap::operator[](Pos p){
  return distMap.cell(p);
}

//////////////
// Funcions //
//////////////

void DistMap::DistCell::clear() {
  sources.clear();
  pseudoSources.clear();
  distance = INF;

  bool toRaise = false;
  bool isCleared = true;
}

/////
/// DistMap
//

///////////////////
// Aux functions //
///////////////////

void DistMap::removeSource(Pos p) {
  distMap[p].clear();

  distMap[p].toRaise = true;

  open.push(DistPos(0, p));
}

void DistMap::setSource(Pos p) {
  distMap[p].clear();

  distMap[p].isCleared = false;
  distMap[p].sources.insert(p);
  distMap[p].distance = 0;

  open.push(DistPos(0, p));
}

void DistMap::updateBase(){
  // Clean old result
  preWaveCrashes.clear();
  waveCrashes.clear();
  modified.clear();

  // update distMap
  while (!open.empty()) {
    Pos p = open.top().second;
    open.pop();

    if (distMap[p].toRaise) {
      processRaise(p);
    } else if (hasBasisPoint(p)) {
      processLower(p);
    }
  }
  // Process waveCrashes
  for (auto preWaveCrash: preWaveCrashes){
    Pos p  = preWaveCrash.first;
    Pos pN = preWaveCrash.second;

    // set pseudoSources
    setPseudoSourcesFromWave(p, pN);
    if (isWaveCrash(pN)) waveCrashes.insert(pN);
  }
}

// Consistent wave
void DistMap::processLower(Pos p) {
  distMap[p].pseudoSources.clear();
  modified.insert(p);

  for (Pos pN : map.adj(p,nonTraversables)) {
    if (distMap[pN].toRaise) continue; // optimization avoids unnecesary operations

    hasBasisPoint(pN); // to clean invalid basis

    Float minD;
    PosSet minDSources;
    tie(minD,minDSources) = closests(pN,distMap[p].sources);
    if (minD < distMap[pN].distance) {
      distMap[pN].distance = minD;
      distMap[pN].sources = minDSources;
      distMap[pN].pseudoSources.clear();
      distMap[pN].isCleared = false;
      open.push(DistPos(distMap[pN].distance, pN));
    } else {
      if (minD == distMap[pN].distance){
        accum(distMap[pN].sources, minDSources);

        if(!is_elem(pN,modified)){
          open.push(DistPos(distMap[pN].distance, pN));
          modified.insert(pN);
        }
      }
      preWaveCrashes.insert(make_pair(p,pN));
      preWaveCrashes.insert(make_pair(pN,p));
    }
  }
}

// Inconsistent wave
void DistMap::processRaise(Pos p) {
  Float minD = INF;
  for (Pos pN : map.adj(p)) {
    if (distMap[pN].isCleared || distMap[pN].toRaise) continue;

    open.push(DistPos(distMap[pN].distance, pN));
    // if n does not have any valid obstacle then its distance is invalid
    // it and should propagate the process raise
    if (!hasBasisPoint(pN)) {
      distMap[pN].clear();
      distMap[pN].toRaise = true;
    } 
  }
  distMap[p].toRaise = false;
}

// return if p has VALID basisPoints and remove the invalid ones
bool DistMap::hasBasisPoint(Pos p){
  filter(distMap[p].sources,       [this](Pos s){return !is_elem(map[s],sources);});
  filter(distMap[p].pseudoSources, [this](Pos s){return !is_elem(map[s],sources);});

  if(distMap[p].sources.empty()){
    distMap[p].pseudoSources.clear();
  }

  return !distMap[p].sources.empty();
}

bool DistMap::isWaveCrash(Pos p){
  return existsNonAdjacent(basisPoints(p));
}

void DistMap::setPseudoSourcesFromWave(Pos waveP, Pos p){
  Float minD;
  PosSet minDSources;
  tie(minD,minDSources) = closests(p,distMap[waveP].sources);

  if (minD < distMap[waveP].distance) return; // there is no wave crash

  for ( Pos candidatePseudoSource : minDSources){
    Float candidateDist = p.distanceTo(candidatePseudoSource);

    // Skip if the difference of distance of the source and candidatePseudoSource is 
    // grater than 1 a pseudo source should be a rounding error caused by discretization
    if(abs(candidateDist - distMap[p].distance) > 1) continue;

    // skip candidate if it is adjacent to a source (belongs to the same obstacle)
    if( exist( distMap[p].sources, [&](Pos p){ return p.adjacent(candidatePseudoSource); } )) continue;

    // skip if adjacent to a pseudoSource of p or replace it if the candidate is closer
    bool skip = false;
    PosSet pseudoSourcesCpy = distMap[p].pseudoSources;
    PosSet toReplace;
    for(Pos currentPseudoSource : pseudoSourcesCpy){
      if(currentPseudoSource.adjacent(candidatePseudoSource)){ // if they belong to the same obstacle keep the closest one
        Float currentPseudoSourceDist = p.distanceTo(currentPseudoSource);
        if(candidateDist <= currentPseudoSourceDist){
          // reaplace
          toReplace.insert(currentPseudoSource);
        }else{
          // skip
          skip = true;
          break;
        }
      }
    } 
    if (skip){
      continue;
    }else{
      for(Pos ps : toReplace){
        distMap[p].pseudoSources.erase(p);
      }
    }

    distMap[p].pseudoSources.insert(candidatePseudoSource);
  }
}

/////////
// API //
/////////

void DistMap::update(MapUpdatedCells &mapUpdatedCells) {
  // Initialize update
  for(auto it : mapUpdatedCells){
    Pos p                 = it.first;
    CellType lastState    = it.second;
    CellType currentState = map[p];
    if(is_elem(currentState,sources)){
      setSource(p);
    }else{ // currentState not source
      if(is_elem(lastState,sources)){
        removeSource(p);
      }
    }
  }

  updateBase();
}

void DistMap::update() {
  // Initialize update
  for (Pos p : map) {
    distMap[p].clear();
    if (is_elem(map[p], sources)) { 
      setSource(p);
    }
  }

  updateBase();
}

// get basisPoints
PosSet DistMap::basisPoints(Pos p){
  PosSet res;

  // sources U pseudoSources
  accum(res,distMap[p].sources);
  accum(res,distMap[p].pseudoSources);

  return res;
}

/////////////////
// Costructors //
/////////////////

pair<Int,Int> DistMap::size(){
  return distMap.size();
}

DistMap::DistMap(MapType& map, vector<CellType> sources, vector<CellType> nonTraversables, vector<CellType> objectives) : map(map){
  this->distMap = DistMapType(map.size()); 
  this->sources = sources;
  this->nonTraversables = nonTraversables;
}

///////////////
// Operators //
///////////////
ostream& operator<<(ostream& out, DistMap& distMap) {
  return out<<distMap.distMap;
}
