#include "DistMap.h"
#include <boost/unordered/unordered_set_fwd.hpp>
#include <cmath>
#include <limits>
#include "Map.h"
#include "data/Pos.h"
#include "utils.h"

//////////////
// DistCell //
//////////////
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

void DistMap::DistCell::clear() {
  sources.clear();
  pseudoSources.clear();
  distance = INF;

  bool toRaise = false;
  bool isCleared = true;
}

DistMap::DistMapType::reference DistMap::operator[](Pos p){
  return distMap.cell(p);
}

/////////////
// DistMap //
/////////////

// wave

/// the boolean value indicates if the pseudo sources are modified
bool setPseudoSourcesFromWave(Pos p, Pos waveP, DistMap& distMap){

  PosSet pseudoSources;
  /* for ( Pos candidatePseudoSource : basisPoints(waveP, distMap)){ */
  for ( Pos candidatePseudoSource : distMap[waveP].sources){
    Float candidateDist = p.distanceTo(candidatePseudoSource);

    // Skip if the difference of distance of the source and the pseudo sources is grater than 1
    // a pseudo source should be a rounding error caused by discretization
    if(abs(candidateDist - distMap[p].distance) > 1) continue;

    // Skip if there exist already a pseudo source closer than the candidates
    Float currentDist;
    if(!distMap[p].pseudoSources.empty()){
      currentDist = p.distanceTo(*distMap[p].pseudoSources.begin());
      if (candidateDist > currentDist) continue;
    }

    // Skip if adjacent to a source
    for(Pos pSource : distMap[p].sources){
      if(candidatePseudoSource.adjacent(pSource)) return false;
    }

    if(distMap[p].pseudoSources.empty() || candidateDist < currentDist){
      // if p does not have pseudo sources already or if the distance to
      // the current ones is grater than the candidates set the candidates
      // as the pseudo sources of p 
      pseudoSources.insert(candidatePseudoSource);
    }else{
      // if the current pseudo sources are at the same distance that the
      // candidates only add the non adjacent to the current ones
      bool modified = false;
      bool adjacent = false;
      for(Pos pPseudoSource : distMap[p].pseudoSources){
        adjacent = candidatePseudoSource.adjacent(pPseudoSource);
        if(adjacent) break;
      }
      if(!adjacent){
        pseudoSources.insert(candidatePseudoSource);
      }
    }
  }
  if(pseudoSources.empty()){
    return false;
  }else{
    distMap[p].pseudoSources = pseudoSources;
    return true;
  }
}

void checkWaveCrash(Pos p, Pos np, DistMap& distMap, PosSet& waveCrashPoss) {
  setPseudoSourcesFromWave(p, np, distMap); 
  setPseudoSourcesFromWave(np, p, distMap); 

  if (existsNonAdjacent(basisPoints(p, distMap)))  waveCrashPoss.insert(p);
  if (existsNonAdjacent(basisPoints(np, distMap))) waveCrashPoss.insert(np);
}

// recheck basisPoints of p
bool DistMap::hasBasisPoint(Pos p){
  filter(distMap[p].sources,      [this](Pos s){return !is_elem(map[s],sources);});
  filter(distMap[p].pseudoSources,[this](Pos s){return !is_elem(map[s],sources);});

  if(distMap[p].sources.empty()){
    distMap[p].pseudoSources.clear();
  }

  return !distMap[p].sources.empty();
}

// sources
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

void DistMap::setConsistentBorders(Pos p){
  for (Pos pN : map.adj(p)) {
    if (hasBasisPoint(pN) && !is_elem(consistentBorders, pN)) {
      open.push(DistPos(distMap[pN].distance, pN));
      consistentBorders.insert(pN);
    }
  }
}

// update
void DistMap::processLower(Pos p) {
  modified.insert(p);

  for (Pos pN : map.adj(p,nonTraversables)) {
    if (distMap[pN].toRaise) continue; // optimization avoids unnecesary operations

    modified.insert(pN);

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

        PosSet filteredPseudoSources;
        for(Pos ps : distMap[pN].pseudoSources ){
          bool adjacentToNewSouce = false;
          for(Pos s : minDSources){
            adjacentToNewSouce = ps.adjacent(s);
            if(adjacentToNewSouce) break;
          }
          if(!adjacentToNewSouce) filteredPseudoSources.insert(ps);
        }
        distMap[pN].pseudoSources = filteredPseudoSources;
      }
      if(hasBasisPoint(pN)){
        checkWaveCrash(p, pN, *this, waveCrashPoss);
      }
      /* if(hasBasisPoint(pN)){ */
      /*   if(!is_elem(map[p],nonTraversables)){ */
      /*     setPseudoSourcesFromWave(p, pN, *this); */ 
      /*     if (existsNonAdjacent(basisPoints(p, *this)))  waveCrashPoss.insert(p); */
      /*   } */

      /*   for(Pos pNN : map.adj(pN,nonTraversables)){ */
      /*     if (hasBasisPoint(pNN) && distMap[pNN].distance >= distMap[pN].distance) { */
      /*       setPseudoSourcesFromWave(pN, pNN, *this); */ 
      /*     } */
      /*   } */
      /*   if (existsNonAdjacent(basisPoints(pN, *this))) waveCrashPoss.insert(pN); */
      /* } */
    }
  }
}

void DistMap::processRaise(Pos p) {
  Float minD = INF;
  for (Pos pN : map.adj(p)) {
    if (distMap[pN].isCleared || distMap[pN].toRaise) continue;

    // if n does not have any valid obstacle then its distance is invalid
    // it and should propagate the process raise
    if (!hasBasisPoint(pN)) {
      // cout << "raised: " <<n << endl;
      open.push(DistPos(distMap[pN].distance, pN));
      distMap[pN].clear();
      distMap[pN].toRaise = true;
    } else {
      open.push(DistPos(distMap[pN].distance, pN));
      /* if(is_elem(gvd.full_graph, s)) old_region_border.insert(s); */
      /* if(is_elem(gvd.full_graph, n)) old_region_border.insert(n); */

      // if n has a valid obstacle then it could be used
      // to obtain the disstance of s
      /* Float d; */
      /* PosSet minDSources; */
      /* tie(d,minDSources) = closests(p,distMap[pN].sources); */
      /* if (d < minD) { */
      /*   minD = d; */
      /*   distMap[p].sources = minDSources; */
      /* } else if (d == minD) { */
      /*   accum(distMap[p].sources, minDSources); */
      /* } */
    }
  }

  /* if (minD < INF) { */
  /*   distMap[p].isCleared = false; */
  /*   distMap[p].distance = minD; */
  /*   open.push(DistPos(distMap[p].distance, p)); */
  /* } */

  distMap[p].toRaise = false;
}

// API
void DistMap::update(MapUpdatedCells mapUpdatedCells) {
  // Clean old result
  waveCrashPoss.clear();
  consistentBorders.clear();
  modified.clear();

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

  // Get new result / replace old
  while (!open.empty()) {
    Pos p = open.top().second;
    open.pop();

    if (distMap[p].toRaise) {
      processRaise(p);
    } else if (hasBasisPoint(p)) {
      waveCrashPoss.erase(p);
      distMap[p].pseudoSources.clear();
      processLower(p);
    }
  }
  for(auto it : mapUpdatedCells){
    Pos p                 = it.first;
    if (!is_elem(p,waveCrashPoss)) continue;
    CellType lastState    = it.second;
    CellType currentState = map[p];
    /* cout<<"---------------"<<endl; */
    /* cout<<p<<" state     : "<< lastState << " -> "<< currentState<< endl; */
    /* cout<<p<<" distance  : "<<distMap[p].distance<<endl; */
    /* cout<<p<<" sources   : "<<distMap[p].sources<<endl; */
    /* cout<<p<<" pSources  : "<<distMap[p].pseudoSources<<endl; */
    /* cout<<p<<" toRaise   : "<<distMap[p].toRaise<<endl; */
    /* cout<<p<<" isCleared : "<<distMap[p].isCleared<<endl; */
  }
  /* cout<<map<<endl; */

}

void DistMap::update() {
  // Clean old result
  waveCrashPoss.clear();

  // Get new result / replace old
  for (Pos p : map) {
    DistMap::DistCell dcell;
    if (is_elem(map[p], sources)) { 
      dcell.distance = 0;
      dcell.sources.insert(p);
      if(!map.adj(p,nonTraversables).empty()){
        open.push(DistPos(0, p));
      }
    } else {
      dcell.distance = INF;
    }
    distMap[p] = dcell;
  }

  while (!open.empty()) {
    Pos p = open.top().second;
    open.pop();

    waveCrashPoss.erase(p);
    distMap[p].pseudoSources.clear();

    processLower(p);
  }
}

// Costructor
pair<Int,Int> DistMap::size(){
  return distMap.size();
}

DistMap::DistMap(MapType& map, vector<CellType> sources, vector<CellType> nonTraversables, vector<CellType> objectives) : map(map){
 this->distMap = DistMapType(map.size()); 
 this->sources = sources;
 this->nonTraversables = nonTraversables;
}

// utils
ostream& operator<<(ostream& out, DistMap& distMap) {
  // out<<"("<< cell.distance<<", "<< cell.sources.size() << " )";
  return out<<distMap.distMap;
}
