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
  if(cell.distance == inf){
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
  Float minD = inf;
  PosSet minDSources;
  for (Pos source : sources) {
    Float d = p.distance_to(source);
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

void checkWaveCrash(Pos p, Pos np, DistMap::DistMapType& distMap, PosSet& waveCrashPoss) {
  /* if (distMap[p].sources.size() > 1) { */
  /*   waveCrashPoss.insert(p); */
  /* } */
  /* if (distMap[np].sources.size() > 1) { */
  /*   waveCrashPoss.insert(np); */
  /* } */

  bool are_adj = true;
  for(Pos pSource : distMap[p].sources){
    for(Pos npSource : distMap[np].sources){
      are_adj = npSource == pSource || is_elem(pSource, distMap.adj(npSource));
      if(!are_adj) break;
    }
    if(!are_adj) break;
  }

  if(are_adj) return;

  if (distMap[p].distance > 0) { //(dist(s, obst_n) <= dist(n, obst_s)) &&  could add the multi obstacle variation
    waveCrashPoss.insert(p);
    distMap[p].crashingWaves.insert(np);
  }
  if (distMap[np].distance > 0) { //(dist(n, obst_s) <= dist(s, obst_n)) && could add the multi obstacle variation 
    waveCrashPoss.insert(np);
    distMap[np].crashingWaves.insert(p);
  }
}

pair<Int,Int> DistMap::size(){
  return distMap.size();
}

DistMap::DistMap(pair<Int,Int> size, vector<CellState> sources, vector<CellState> nonTraversables, vector<CellState> objectives){
 distMap = DistMapType(size); 
 this->sources = sources;
 this->nonTraversables = nonTraversables;
 this->objectives = objectives;
}

void DistMap::update(StateGrid& grid) {
  // initialize the dgrid and the distance queues
  DistPosQueue dqueue; 
  objectiveDQueue = DistPosQueue(); //clear las full_dqueue

  // clear reset distanceMap
  for (Pos p : grid) {
    CellState cState = grid[p];

    DistMap::DistCell dcell;
    if (is_elem(cState, sources)) { 
      dcell.distance = 0;
      dcell.sources.insert(p);
      if(!grid.adj(p,nonTraversables).empty()){
        dqueue.push(DistPos(0, p));
      }
    } else {
      dcell.distance = inf;
    }
    distMap[p] = dcell;
  }

  while (!dqueue.empty()) {
    Pos p = dqueue.top().second;
    dqueue.pop();

    waveCrashPoss.erase(p);
    distMap[p].crashingWaves.clear();

    // Update neihbors distance
    for (Pos np : grid.adj(p,nonTraversables)) {
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

      if (objectives.empty() || is_elem(grid[np],objectives)){
        objectiveDQueue.push(DistPos(distMap[np].distance, np));
      }

    }
  }
}

ostream& operator<<(ostream& out, DistMap& distMap) {
  // out<<"("<< cell.distance<<", "<< cell.sources.size() << " )";
  return out<<distMap.distMap;
}

