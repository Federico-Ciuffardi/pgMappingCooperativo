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

boost::tuple<Float, PosSet> closestSources(Pos p, PosSet sources) {
  Float minD = INF;
  PosSet minDSources;
  for (Pos source : sources) {
    Float d = p.distanceTo(source);
    if (d < minD) {
      minD = d;
      minDSources.clear();
      minDSources.insert(source);
    } else if (d == minD) {
      minDSources.insert(source);
    }
  }
  return boost::make_tuple(minD, minDSources);
}

// the boolean value indicates if the pseudoSources are modified
bool setPseudoSourcesFromWave(Pos p, Pos waveP, DistMap::DistMapType& distMap){
  Int min;
  PosSet candidatePseudoSources;
  tie(min,candidatePseudoSources) = closestSources(p, distMap[waveP].sources);

  // Skip if there exist already a pseudo source closer than the candidates
  Float distToCurrentPesudoSource;
  if(!distMap[p].pseudoSources.empty()){
    distToCurrentPesudoSource = p.distanceTo(*distMap[p].pseudoSources.begin());
    if (min > distToCurrentPesudoSource) return false;
  }

  // Skip if the difference of distance of the source and the pseudo sources is grater than 1
  // a pseudo source should be a rounding error caused by discretization
  if(abs(min - distMap[p].distance) > 1) return false;

  // Skip if adjacent to a source
  for(Pos candidatePseudoSource : candidatePseudoSources){
    for(Pos pSource : distMap[p].sources){
      if(candidatePseudoSource.distanceToSquared(pSource) <= 4) return false;
    }
  }

  if(distMap[p].pseudoSources.empty() || min < distToCurrentPesudoSource){
    // if p does not have pseudo sources already or if the distance to
    // the current ones is grater than the candidates set the candidates
    // as the pseudo sources of p 
    distMap[p].pseudoSources = candidatePseudoSources;
    return true;
  }else{
    // if the current pseudo sources are at the same distance that the
    // candidates only add the non adjacent to the current ones
    bool modified = false;
    for(Pos candidatePseudoSource : candidatePseudoSources){
      bool adjacent = false;
      for(Pos pPseudoSource : distMap[p].pseudoSources){
        adjacent = candidatePseudoSource.distanceToSquared(pPseudoSource) <= 4;
        if(adjacent) break;
      }
      if(!adjacent){
        distMap[p].pseudoSources.insert(candidatePseudoSource);
        modified = true;
      }
    }
    return modified;
  }
}

void checkWaveCrash(Pos p, Pos np, DistMap::DistMapType& distMap, PosSet& waveCrashPoss) {
  if (distMap[p].sources.size() > 1) {
    waveCrashPoss.insert(p);
  }
  if (distMap[np].sources.size() > 1) {
    waveCrashPoss.insert(np);
  }

  /* bool are_adj = true; */
  /* for(Pos pSource : distMap[p].sources){ */
  /*   for(Pos npSource : distMap[np].sources){ */
  /*     are_adj = npSource.distanceToSquared(pSource) <= 4;//npSource == pSource || is_elem(pSource, distMap.adj(npSource)); */
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
  if(setPseudoSourcesFromWave(p, np, distMap)) waveCrashPoss.insert(p); 
  if(setPseudoSourcesFromWave(np, p, distMap)) waveCrashPoss.insert(np); 
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

    // Update neihbors distance
    for (Pos np : map.adj(p,nonTraversables)) {
      Float minD;
      PosSet minDSources;
      tie(minD,minDSources) = closestSources(np,distMap[p].sources);

      if (minD < distMap[np].distance) {
        distMap[np].distance = minD;
        distMap[np].sources = minDSources;
        dqueue.push(DistPos(distMap[np].distance, np));
      } else {
        if (minD == distMap[np].distance){
          distMap[np].sources.insert(minDSources.begin(), minDSources.end());
        }
        checkWaveCrash(p, np, distMap, waveCrashPoss);
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
