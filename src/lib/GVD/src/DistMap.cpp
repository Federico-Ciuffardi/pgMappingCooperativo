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

///////////
// Clear //
///////////

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
  waveCrashes.clear();
  modified.clear();

  // update distMap
  while (!open.empty()) {
    Pos p = open.top().second;
    open.pop();

    if (distMap[p].toRaise) {
      processRaise(p);
    } else if (hasBasisPoint(p)) {
      waveCrashes.erase(p);
      distMap[p].pseudoSources.clear();
      processLower(p);
    }
  }
}

// Consistent wave
void DistMap::processLower(Pos p) {
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

        // fiter the pseudoSources now adjacent to a new source
        filter(distMap[pN].pseudoSources, [this,minDSources](Pos ps){
          for(Pos s : minDSources){
            if (ps.adjacent(s)) return true;
          }
          return false;
        });

        if(!is_elem(pN,modified)) open.push(DistPos(distMap[pN].distance, pN));
      }
      processWaveCrash(p, pN);
    }
    modified.insert(pN);
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
  // version 1
  /* return existsNonAdjacent(basisPoints(p)); */


  // version 2
  // the unknownSources part is to maintain the gvd connected when updating the connectivityAux
  PosSet unknownSources;
  for(Pos p : distMap[p].sources){
    if(map[p] == Unknown){
      unknownSources.insert(p);
    }
  }
  return unknownSources.size() > 1 || existsNonAdjacent(basisPoints(p));
}

void DistMap::processWaveCrash(Pos p, Pos np){
  setPseudoSourcesFromWave(p, np);
  setPseudoSourcesFromWave(np, p);

  if (isWaveCrash(p)) waveCrashes.insert(p);
  if (isWaveCrash(np)) waveCrashes.insert(np);
}

// ver 1
void DistMap::setPseudoSourcesFromWave(Pos p, Pos waveP){
  PosSet pseudoSources = distMap[p].pseudoSources;
  for ( Pos candidatePseudoSource : distMap[waveP].sources){
    Float candidateDist = p.distanceTo(candidatePseudoSource);

    // Skip if the difference of distance of the source and candidatePseudoSource is 
    // grater than 1 a pseudo source should be a rounding error caused by discretization
    if(abs(candidateDist - distMap[p].distance) > 1) continue;

    // Skip if there exist already a pseudo source closer than the candidate
    Float currentDist = INF;
    if(!pseudoSources.empty()){
      currentDist = p.distanceTo(*pseudoSources.begin());
      if (candidateDist > currentDist) continue;
    }

    // skip if adjacent to a source (this could be changed to an abort)
    if( exist( distMap[p].sources, [&](Pos p){ return p.adjacent(candidatePseudoSource); } ))
      return;

    if (candidateDist < currentDist){
      // if candidate distance is less than the current one(s) then remove the current ones
      pseudoSources.clear();
    } else {
      // if the candidate distance is the same that the current one(s) then
      // Skip if adjacent to a pseudo source
      if( exist( pseudoSources, [&](Pos p){ return p.adjacent(candidatePseudoSource); } ))
        continue;
    }
    pseudoSources.insert(candidatePseudoSource);
  }
  distMap[p].pseudoSources = pseudoSources;
}

// ver 2
/* void DistMap::setPseudoSourcesFromWave(Pos p, Pos waveP){ */
/*   PosSet pseudoSources = distMap[p].pseudoSources; */
/*   for ( Pos candidatePseudoSource : distMap[waveP].sources){ */
/*     Float candidateDist = p.distanceTo(candidatePseudoSource); */

/*     // Skip if the difference of distance of the source and candidatePseudoSource is */ 
/*     // grater than 1 a pseudo source should be a rounding error caused by discretization */
/*     if(abs(candidateDist - distMap[p].distance) > 1) continue; */

/*     // skip if adjacent to a source (this could be changed to an abort) */
/*     if( exist( distMap[p].sources, [&](Pos p){ return p.adjacent(candidatePseudoSource); } )) continue; */

/*     // if the candidate distance is the same that the current one(s) then */
/*     // Skip if adjacent to a pseudo source */
/*     if( exist( pseudoSources, [&](Pos p){ return p.adjacent(candidatePseudoSource); } )) continue; */

/*     pseudoSources.insert(candidatePseudoSource); */
/*   } */
/*   distMap[p].pseudoSources = pseudoSources; */
/* } */

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
      /* }else if(lastState == Unknown){ */
      /*   setConsistentBorders(p); */
      }
    }
  }

  updateBase();
}

void DistMap::update() {
  // Get new result / replace old
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
