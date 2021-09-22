#include "CentralModule.h"
#include "../lib/affinity_propagation.h"
#include "map_msgs/OccupancyGridUpdate.h"
#include "nav_msgs/OccupancyGrid.h"
#include <cstddef>
#include <vector>
#include "../lib/utils.h"

///////////////
// Variables //
///////////////
vector<CellState> notFrontier = {Occupied, Unknown, Free, Critical, CriticalLine};

//////////////////
// Constructors //
//////////////////
CentralModule::CentralModule() {
  state = WaitingAuction;

  assignmentId = 0;
  auctionId = 0;
}

///////////////
// Get / Set //
///////////////
centralMouleState CentralModule::getState() {
  return state;
};

void CentralModule::setState(centralMouleState newState) {
  state = newState;
};

/////////
// API //
/////////

void CentralModule::updateMap(const OccupancyGridConstPtr& newOccupancyGrid) {
  map.update(newOccupancyGrid);
}

void CentralModule::updateMap(const OccupancyGridUpdateConstPtr& update) {
  map.update(update);
}

// Get the info to start an Auction
Auction CentralModule::getAuctionInfo() {
  // Convert a occupancyGrid and a frontier list to a map
  cout << "debug :: convert a occupancyGrid and a frontier list to a map" << endl;

  // Get the frontiers from the map
  frontiers.clear();
  for(Pos pos : map.map){
    if(map.map[pos] == Frontier){
      frontiers.insert(pos);
    }
  }

  cout << "debug :: get significant frontiers" << endl;
  if(frontierSimplificationMethod > 0){
    // Calculate the connectedComponents of frontiers
    if (!frontierConComps) {
      frontierConComps = new ConnectedComponents(map.map, notFrontier);
    }
    frontierConComps->update();

    // obtain the significativeFrontiers for each connectedComponent with the selected frontierSimplificationMethod
    for ( auto it : frontierConComps->connectedComponents){
      PosSet frontierConCompSet = it.second.members;

      vector<Pos> significativeFrontiers;
      switch (frontierSimplificationMethod) {
        case 1:
        case 2:{
          vector<Pos> frontierConCompVec = toVec(it.second.members);

          int estimatedK = ceil(frontierConCompVec.size()/(sensorRange*2.0));
          int k = estimatedK;

          vector<Pos> centroids = kMeans(frontierConCompVec, k, kMeansMaxIter, kMeansTolerance);
          significativeFrontiers = embed(centroids, frontierConCompVec);

          if(frontierSimplificationMethod == 2){
            if(contains(significativeFrontiers, sensorRange, frontierConCompVec)){
              // decrement k until the kMeans result does not cover all the frontiers (and keep the result of the previous k)
              vector<Pos> oldSignificativeFrontiers;
              do {
                oldSignificativeFrontiers = significativeFrontiers;
                if( k == 1 ) break; // k = 1 is the minimum value possible
                k--;
                // execute kMeans
                centroids = kMeans(frontierConCompVec, k, kMeansMaxIter, kMeansTolerance);
                significativeFrontiers = embed(centroids, frontierConCompVec);
              }while( contains(significativeFrontiers, sensorRange, frontierConCompVec) );
              significativeFrontiers = oldSignificativeFrontiers;
            }else{
              // increment k until the kMeans result covers all the frontiers
              do {
                k++;
                vector<Pos> nonEmbeddedCentroids = kMeans(frontierConCompVec, k, kMeansMaxIter, kMeansTolerance);
                centroids = embed(nonEmbeddedCentroids, frontierConCompVec);
                // execute kMeans
              }while( !contains(centroids, sensorRange, frontierConCompVec) );
            }
          }
        }break;
        case 3:{
          vector<Pos> frontierConCompVec = toVec(it.second.members);
          significativeFrontiers = affinityPropagation(frontierConCompVec, sensorRange);
         }break;
        case 4:{
          significativeFrontiers = getRandomSignificativeFroniers(frontierConCompSet, sensorRange, map.map, {Occupied});
        }break;
        case 5:{
          significativeFrontiers = getSignificativeFroniers(frontierConCompSet, sensorRange, map.map, {Occupied});
        }break;
      }

      // unmark all the forntiers
      for(Pos f : frontierConCompSet) map.map[f] = Free;
     
      // mark only the significant frontiers
      for(Pos f : significativeFrontiers) map.map[f] = Frontier;
    }
  }

  // Get the info for the auction (topological map), and the GVD as a subproduct
  cout << "debug :: get topoMap" << endl;
  if(!topoMap){
    topoMap = new TopoMap(map.map);
  }
  /* topoMap->update(map.updatedCells); */
  topoMap->update(map.updatedCells);

  // Restore frontiers to the non simplified ones
  for(Pos p : frontiers){
    map.map[p] = Frontier;
  }

  // Construct the acution rosmsg
  Auction auctionInfo;

  // Turn the boost GVD to a ros message
  GvdGraph& gvd = *(topoMap->gvd->graphGvd);
  cout << "debug :: gvd to rosmsg" << endl;
  for (GvdGraph::Vertex v : gvd) {
    auctionInfo.gvd.vertices.push_back(toPoint2D(gvd.g[v].p));
    for (GvdGraph::Vertex nv : gvd.adj(v)) {
      Edge e;
      e.from = toPoint2D(gvd.g[v].p);
      e.to = toPoint2D(gvd.g[nv].p);
      auctionInfo.gvd.edges.push_back(e);
    }
  }

  // Turn the frontiers info into a ros message
  cout << "debug :: turn frontiers to rosmsg" << endl;
  for (auto it : topoMap->segmenter->connectedComponents) {
    PosSet &frontiers = it.second.typeMembers[Frontier];

    for (Pos frontier : frontiers) {
      auctionInfo.frontiers.push_back(toPoint2D(frontier));
    }
  }
  cout << "debug :: ended turn frontier to rosmsg" << endl;
  auctionInfo.id = auctionId;

  // Increment the auction ID as a new auction will begin
  auctionId++;

  // clear map updates
  map.updatedCells.clear();

  // return the info of the auction (and the GVD) bundled as a ros message
  return auctionInfo;
}

// Save the bid of a robot (name) for an ongoing auction
// Return true if the bid is valid (for the currently ongoing auction) and false otherwise
bool CentralModule::saveBid(Bid bid, RobotId robotId) {
  // skip if old
  if (assignmentId != bid.id)  return false; 

  // Store the bid
  for (int i = 0; i < bid.frontiers.size(); i++) {
    Pos frontierPos = toPos(bid.frontiers[i]);
    Pos robotPos = toPos(bid.robotPosition);

    // Calculate bid segmentValueComponent
    float segmentValueComponent = 0;
    bool sameSegment = topoMap->segmenter->idGrid[frontierPos] == topoMap->segmenter->idGrid[robotPos];
    switch (bidSegmentValueComponentMode) {
      case 1:
        if(!sameSegment){
          segmentValueComponent = bidSegmentValueComponentCoefficient;
        }
        break;
      case 2:
        if(sameSegment){
          segmentValueComponent = -bidSegmentValueComponentCoefficient;
        }
        break;
    }

    // Calculate bid value
    Float value = bid.pathLength[i] + bid.pathEntryYaw[i] + segmentValueComponent;

    // Store frontier bid
    auctioneer.addBid(robotId, topoMap->segmenter->idGrid[frontierPos] , frontierPos, value);
    bids[robotId][frontierPos] = value;
  }

  return true;
}

// Resolve the auction given the current bids
boost::unordered_map<string, Assignment> CentralModule::assign() {
  // Resolve the auction
  boost::unordered_map<RobotId, Pos> resolution = auctioneer.resolveAuction();

  // Process the auction resolution and bundle it as a ros message
  boost::unordered_map<string, Assignment> resolutionRosMessages;

  for (auto it : resolution) {
    RobotId robotId = it.first;
    Pos frontier = it.second;

    Assignment assignment;
    assignment.id = assignmentId;
    assignment.frontier = toPoint2D(frontier);

    resolutionRosMessages[robotId] = assignment;
  }

  // Increment the assignment ID to set to the next assignment
  assignmentId++;

  // return the info of the assignment bundled as a ros message
  return resolutionRosMessages;
}


////////////////////////////////////
// custom frontier simplification //
////////////////////////////////////

// aux

bool unobstructedLine(Pos p1, Pos p2, Map &map, vector<CellState> obstructedTypes){
  for ( Pos p : discretizeLine(p1,p2)){
    if(is_elem(map[p], obstructedTypes)) return false;
  }
  return true;
}

PosSet accumCircle(PosSet &interior, Pos c, Float radius, PosSet &toCover, Map &map, vector<CellState> nonTraversables){
  Float radiusSquared = radius*radius;

  PosSet circumference;
  PosSet visited;

  std::queue<Pos> toVisit; 
  toVisit.push(c);

  while (!toVisit.empty()) {
    Pos p = toVisit.front(); toVisit.pop();

    if(is_elem(p,visited)) continue;

    visited.insert(p);

    if(is_elem(p,toCover) && unobstructedLine(c, p, map, {Occupied})){
      interior.insert(p);
    }

    Float pDist = p.distanceToSquared(c);
    for (Pos pN : map.adj(p,nonTraversables)) {
      Float pNDist = pN.distanceToSquared(c);

      if (pDist < pNDist){
        if(pNDist < radiusSquared) {
          toVisit.push(pN);
        }else{
          if(is_elem(pN,toCover)) circumference.insert(pN);
        }
      }
    }
  }
  return circumference;
}

// frontier simplification

vector<Pos> getRandomSignificativeFroniers(PosSet &frontiersSet, Float radius, Map& map, vector<CellState> nonTraversables){
  vector<Pos> frontiersVec = toVec(frontiersSet);
  shuffle(std::begin(frontiersVec), std::end(frontiersVec), default_random_engine(2021));

  vector<Pos> significativeFrontiers;
  PosSet coveredFrontiers;

  int i = 0;
  while(coveredFrontiers.size() < frontiersVec.size()){
    if(i>=frontiersVec.size()) FAIL("Out of index: "<<coveredFrontiers.size()<<"/"<<i<<"/"<<frontiersVec.size());

    Pos significativeFrontier = frontiersVec[i];
    i++;

    if(is_elem(significativeFrontier,coveredFrontiers)) continue;

    significativeFrontiers.push_back(significativeFrontier);

    accumCircle(coveredFrontiers, significativeFrontier, radius, frontiersSet, map, nonTraversables);
  }

  return significativeFrontiers;
}

vector<Pos> getSignificativeFroniers(PosSet &frontiersSet, Float radius, Map& map, vector<CellState> nonTraversables){
  PosSet remainingFrontiers = frontiersSet;
  vector<Pos> significativeFrontiers;
  std::queue<Pos> endPoints;

  // define local function to add a significativeFrontier
  auto addSignificativeFrontier = [&](Pos significativeFrontier) { 
    // set the significativeFrontier
    significativeFrontiers.push_back(significativeFrontier);
    PosSet coveredFrontiers;
    PosSet newEndPoints;
    /// set the new coveredFrontiers
    newEndPoints = accumCircle(coveredFrontiers, significativeFrontier, radius + 0.5, remainingFrontiers, map, nonTraversables);
    substract(remainingFrontiers,coveredFrontiers);
    /// get new end points
    for(Pos p : newEndPoints){
      endPoints.push(p);
    }
  };

  // Set the frontiers adjacent to obstacles as the first endPoints
  for(Pos frontier : frontiersSet){
    bool hasObstructedNeighbor = false;
    for(Pos frontierNeighbor : map.adj(frontier)){
      hasObstructedNeighbor = map[frontierNeighbor] == Occupied;
      if(hasObstructedNeighbor){
        endPoints.push(frontier);
        break;
      }
    }
  }

  // if there is no endPoint yet, set the first significativeFrontier to get the first endPoints
  if(endPoints.empty()){
    addSignificativeFrontier(*frontiersSet.begin());
  }

  // Cover current endPoints and discover new ones, until all the frontiers are covered
  while ( !endPoints.empty() ) {
    // Get the oldest endPoint and cover it 
    Pos uncoveredFrontier = endPoints.front(); endPoints.pop();
     
    // skip if already covered
    if (!is_elem(uncoveredFrontier, remainingFrontiers)) continue;

    // get the distance to from the uncoveredFrontier to it farthest frontier (no further than radius*2)
    PosSet circle;
    accumCircle(circle, uncoveredFrontier, radius*2, remainingFrontiers, map, notFrontier);

    Float maxDist = -INF;
    for(Pos p : circle){
      maxDist = max(maxDist,p.distanceTo(uncoveredFrontier));
    }
    maxDist = min(maxDist,radius*2 - 0.5f);

    // get significativeFrontier candidates
    circle.clear();
    PosSet candidates = accumCircle(circle, uncoveredFrontier, maxDist/2 - 1, remainingFrontiers, map, nonTraversables);

    // get the significativeFrontier
    Pos significativeFrontier;
    if(candidates.empty()){
      // if candidates is empty then the uncoveredFrontier is the only remainingFrontier frontier 
      significativeFrontier = uncoveredFrontier;
    }else{
      // else get the farthest candidate from the uncoveredFrontier
      // (TODO get the candidate that has max information gain)
      Float maxDist = -INF;
      Pos maxDistPos = NULL_POS;
      for(Pos p : candidates){
        Float dist = p.distanceTo(uncoveredFrontier);
        if(maxDist < dist ){
          maxDist = dist;
          maxDistPos = p;
        }
      }
      significativeFrontier = maxDistPos;
    }
    
    // set the significativeFrontier
    addSignificativeFrontier(significativeFrontier);
  }
  return significativeFrontiers;
}

/////////////////////////////////////
// k-means frontier simplification //
/////////////////////////////////////

// true if the circles defined by the centrers and the radius, contains all the points
bool contains(vector<Pos> &centers, Float radius, vector<Pos> &points) {
  Float radiusSquared = radius*radius;

  for(Pos point : points){
    // search for a circle that contains the point
    int i = 0;
    for(; i < centers.size() && point.distanceToSquared(centers[i]) > radiusSquared; i++);
    // if no circle contains the point (i is an invalid index)
    if( i == centers.size() ) return false;
  }

  return true;
}

// Embed the pos from `from` to pos on `to`
vector<Pos> embed(vector<Pos> &from, vector<Pos> &to){
  vector<Pos> res;
  for (Pos f : from) {
    Pos embedded = NULL_POS;
    Float minDist = INF;

    for (Pos t : to) {
      Float dist = f.distanceToSquared(t);
      if (dist < minDist) {
        embedded = t;
        minDist = dist;
      }
    }
    res.push_back(embedded);
  }
  return res;
}


// modified version of: 
// http://www.goldsborough.me/c++/python/cuda/2017/09/10/20-32-46-exploring_k-means_in_python,_c++_and_cuda/
vector<Pos> kMeans(const vector<Pos>& data, size_t k, size_t maxIterations, Float tolerance) {
  // Pick random centroids
  /// initialize randomizer
  /* static random_device seed; */
  /* static mt19937 rng(seed()); */
  /* uniform_int_distribution<size_t> indices(0, data.size() - 1); */

  // Pick centroids as random points from the dataset.
  /* vector<Vector2<Float>> means(k); */
  /* for (Vector2<Float>& cluster : means) { */
  /*   cluster = data[indices(rng)]; */
  /* } */

  /// Pick centroids from the first k data points
  vector<Pos> means(k);
  for (int i = 0; i < k; i++) {
    means[i] = data[i];
  }

  std::vector<size_t> assignments(data.size());
  Float maxDistance;
  size_t iteration = 0;
  do{
    // Find assignments.
    for (size_t point = 0; point < data.size(); ++point) {
      Float bestDistance = INF;
      size_t bestCluster = 0;
      for (size_t cluster = 0; cluster < k; ++cluster) {
        const Float distance = data[point].distanceTo(means[cluster]);
        if (distance < bestDistance) {
          bestDistance = distance;
          bestCluster = cluster;
        }
      }
      assignments[point] = bestCluster;
    }

    // Sum up and count points for each cluster.
    vector<Pos> newMeans(k);
    std::vector<size_t> counts(k, 0);
    for (size_t point = 0; point < data.size(); ++point) {
      const size_t cluster = assignments[point];
      newMeans[cluster] += data[point];
      counts[cluster] += 1;
    }

    maxDistance = 0;
    // Divide sums by counts to get new centroids.
    for (size_t cluster = 0; cluster < k; ++cluster) {
     
      const size_t count = std::max<size_t>(1, counts[cluster]); // Avoid zero division.
      newMeans[cluster] = newMeans[cluster] / count;

      maxDistance = max(maxDistance,means[cluster].distanceTo(newMeans[cluster]));
      means[cluster] = newMeans[cluster];
    }

    iteration++;
  } while(maxDistance > tolerance  && iteration < maxIterations);

  if (iteration == maxIterations){
    ROS_INFO_STREAM("Not enough iterations for k-means to converge, last error: "<<maxDistance);
  }

  return means;
}

CentralModule::~CentralModule(){
  delete topoMap;
}
