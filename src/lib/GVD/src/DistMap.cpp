#include "DistMap.h"
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

DistMap::DistMapType::reference DistMap::operator[](Pos p){
  return distMap.cell(p);
}

/////////////
// DistMap //
/////////////

// the boolean value indicates if the pseudo sources are modified
bool setPseudoSourcesFromWave(Pos p, Pos waveP, DistMap& distMap){


  PosSet pseudoSources;
  for ( Pos candidatePseudoSource : basisPoints(waveP, distMap)){
    Float candidateDist = p.distanceTo(candidatePseudoSource);

    // Skip if there exist already a pseudo source closer than the candidates
    Float currentDist;
    if(!distMap[p].pseudoSources.empty()){
      currentDist = p.distanceTo(*distMap[p].pseudoSources.begin());
      if (candidateDist > currentDist) continue;
    }

    // Skip if the difference of distance of the source and the pseudo sources is grater than 1
    // a pseudo source should be a rounding error caused by discretization
    if(abs(candidateDist - distMap[p].distance) > 1) continue;

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

bool existsNonAdjacent(PosSet ps1, PosSet ps2){
  for(Pos p1 : ps1){
    for(Pos p2 : ps2){
        if(!p1.adjacent(p2)) return true;
    }
  }
  return false;
}

bool existsNonAdjacent(PosSet ps){
  return existsNonAdjacent(ps, ps);
}

void checkWaveCrash(Pos p, Pos np, DistMap& distMap, PosSet& waveCrashPoss) {
  // mehtod 3
  setPseudoSourcesFromWave(p, np, distMap); 
  setPseudoSourcesFromWave(np, p, distMap); 

  if (existsNonAdjacent(basisPoints(p, distMap)))  waveCrashPoss.insert(p);
  if (existsNonAdjacent(basisPoints(np, distMap))) waveCrashPoss.insert(np);

  // method 2
  /* if (distMap[p].sources.size()  > 1) waveCrashPoss.insert(p); */ 
  /* if (distMap[np].sources.size() > 1) waveCrashPoss.insert(np); */
  
  /* if(setPseudoSourcesFromWave(p, np, distMap)) waveCrashPoss.insert(p); */ 
  /* if(setPseudoSourcesFromWave(np, p, distMap)) waveCrashPoss.insert(np); */ 


  // method 1
  /* bool are_adj = true; */
  /* for(Pos pSource : distMap[p].sources){ */
  /*   for(Pos npSource : distMap[np].sources){ */
  /*     are_adj = npSource.adjacent(pSource); // better than: `npSource == pSource || is_elem(pSource, distMap.adj(npSource));` */
  /*     if(!are_adj) break; */
  /*   } */
  /*   if(!are_adj) break; */
  /* } */

  /* if(are_adj) return; */

  /* if (distMap[p].distance > 0) { //(dist(s, obst_n) <= dist(n, obst_s)) &&  could add the multi obstacle variation */
  /*   waveCrashPoss.insert(p); */
  /*   distMap[p].crashingWaves.insert(np); */
  /* } */
  /* if (distMap[np].distance > 0) { //(dist(n, obst_s) <= dist(s, obst_n)) && could add the multi obstacle variation */ 
  /*   waveCrashPoss.insert(np); */
  /*   distMap[np].crashingWaves.insert(p); */
  /* } */
}

pair<Int,Int> DistMap::size(){
  return distMap.size();
}

DistMap::DistMap(MapType& map, vector<CellType> sources, vector<CellType> nonTraversables, vector<CellType> objectives) : map(map){
 distMap = DistMapType(map.size()); 
 this->sources = sources;
 this->nonTraversables = nonTraversables;
 this->objectives = objectives;
}

void DistMap::update() {
  // Clean old result
  objectiveDQueue = DistPosQueue();
  waveCrashPoss.clear();

  // Get new result / replace old
  DistPosQueue dqueue; 
  for (Pos p : map) {
    DistMap::DistCell dcell;
    if (is_elem(map[p], sources)) { 
      dcell.distance = 0;
      dcell.sources.insert(p);
      if(!map.adj(p,nonTraversables).empty()){
        dqueue.push(DistPos(0, p));
      }
    } else {
      dcell.distance = INF;
    }
    distMap[p] = dcell;
  }

  while (!dqueue.empty()) {
    Pos p = dqueue.top().second;
    dqueue.pop();

    waveCrashPoss.erase(p);
    distMap[p].pseudoSources.clear();

    // Update neighbors distance
    for (Pos np : map.adj(p,nonTraversables)) {
      Float minD;
      PosSet minDSources;
      tie(minD,minDSources) = closests(np,distMap[p].sources);

      if (minD < distMap[np].distance) {
        distMap[np].distance = minD;
        distMap[np].sources = minDSources;
        dqueue.push(DistPos(distMap[np].distance, np));
      } else {
        if (minD == distMap[np].distance){
          distMap[np].sources.insert(minDSources.begin(), minDSources.end());
        }
        checkWaveCrash(p, np, *this, waveCrashPoss);
      }

      if (objectives.empty() || is_elem(map[np],objectives)){
        objectiveDQueue.push(DistPos(distMap[np].distance, np));
      }

    }
  }
}

ostream& operator<<(ostream& out, DistMap& distMap) {
  // out<<"("<< cell.distance<<", "<< cell.sources.size() << " )";
  return out<<distMap.distMap;
}
