#include "CentralModule.h"
#include "../lib/affinity_propagation.h"
#include <cstddef>
#include <vector>

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

void CentralModule::updateMap(const MapMergedInfoConstPtr& newMap) {
  // store occGrid
  occupancyGrid = newMap->occupancyGrid;

  // store frontiers
  frontiers = toPosSet(newMap->frontiers, newMap->occupancyGrid.info.width);
}

// Get the info to start an Auction
Auction CentralModule::getAuctionInfo() {
  // Convert a occupancyGrid and a frontier list to a stateGrid
  cout << "debug :: convert a occupancyGrid and a frontier list to a stateGrid" << endl;
  stateGrid = toStateGrid(occupancyGrid, &cellCount);
  /// frontiers
  PosSet filteredFrontiers;
  for(Pos fPos : frontiers){
    // skip if not inside the stateGrid (the map_merger delivers frontiers
    // outside the map boundaries sometimes)
    if ( !stateGrid.inside(fPos) ) continue;

    // The frontiers must have a Unknown neighbor
    /// Currently the frontiers can be detected despite of not having a unknow neighbor in two cases
    ///  * The frontier is on the border of the grid (the current implementation of 
    ///    the map_merger considers that "frontier"  I do not)
    ///  * The only unknows cells neighbor of the "frontier" are diagonal but
    //     the horizontal cells near that diagonal are /    not traversable, so the
    //     unknow cell is not considered reachable from the "frontier"
    bool hasUnknowNeighbor = false;
    for( Pos fPosN : stateGrid.adj(fPos,{Occupied})){
        hasUnknowNeighbor = stateGrid[fPosN] == Unknown;
        if(hasUnknowNeighbor) break;
    }
    if(hasUnknowNeighbor){ 
      stateGrid[fPos] = Frontier;
      filteredFrontiers.insert(fPos);
    }
  }
  frontiers = filteredFrontiers;

  cout << "debug :: get significant frontiers" << endl;
  // Get the significant frontiers taking into account the vision range of the robots
  switch (frontierSimplificationMethod) {
    case 1:
      // Calculate the connectedComponents of frontiers
      if (!frontierConComps) {
        frontierConComps = new ConnectedComponents(stateGrid, notFrontier);
      }
      frontierConComps->update();

      for ( auto it : frontierConComps->connectedComponents){
        vector<Pos> frontiers = toVec(it.second.members);

        // Calculate k y and execute kMeans
        int k = ceil(frontiers.size()/(sensorRange*2.0));

        vector<Pos> nonEmbeddedCentroids = kMeans(frontiers, k, kMeansMaxIter, kMeansTolerance);
        vector<Pos> centroids = embed(nonEmbeddedCentroids, frontiers);

        // unmark all the forntiers
        for(Pos f : frontiers) stateGrid[f] = Free;
       
        // mark only the significant frontiers
        for(Pos f : centroids) stateGrid[f] = Frontier;
      }
      break;
    case 2:
      // Calculate the connectedComponents of frontiers
      if (!frontierConComps) {
        frontierConComps = new ConnectedComponents(stateGrid, {Occupied, Unknown, Free, Critical, CriticalLine});
      }
      frontierConComps->update();

      for ( auto it : frontierConComps->connectedComponents){
        vector<Pos> frontiers = toVec(it.second.members);

        int estimatedK = ceil(frontiers.size()/(sensorRange*2.0));
        int k = estimatedK;

        vector<Pos> nonEmbeddedCentroids = kMeans(frontiers, k, kMeansMaxIter, kMeansTolerance);
        vector<Pos> centroids = embed(nonEmbeddedCentroids, frontiers);

        if(contains(centroids, sensorRange, frontiers)){
          // decrement k until the kMeans result does not cover all the frontiers (and keep the result of the previous k)
          vector<Pos> oldCentroids;
          do {
            oldCentroids = centroids;
            if( k == 1 ) break; // k = 1 is the minimum value possible
            k--;
            // execute kMeans
            vector<Pos> nonEmbeddedCentroids = kMeans(frontiers, k, kMeansMaxIter, kMeansTolerance);
            centroids = embed(nonEmbeddedCentroids, frontiers);
          }while( contains(centroids, sensorRange, frontiers) );
          centroids = oldCentroids;
        }else{
          // increment k until the kMeans result covers all the frontiers
          do {
            k++;
            vector<Pos> nonEmbeddedCentroids = kMeans(frontiers, k, kMeansMaxIter, kMeansTolerance);
            centroids = embed(nonEmbeddedCentroids, frontiers);
            // execute kMeans
          }while( !contains(centroids, sensorRange, frontiers) );
        }

        // unmark all the forntiers
        for(Pos f : frontiers) stateGrid[f] = Free;
       
        // mark only the significant frontiers
        for(Pos f : centroids) stateGrid[f] = Frontier;
      }
      break;
    case 3:
      // Calculate the connectedComponents of frontiers
      if (!frontierConComps) {
        frontierConComps = new ConnectedComponents(stateGrid, {Occupied, Unknown, Free, Critical, CriticalLine});
      }
      frontierConComps->update();

      for ( auto it : frontierConComps->connectedComponents){
        vector<Pos> frontiers = toVec(it.second.members);

        vector<Pos> centroids = affinityPropagation(frontiers, sensorRange);

        // unmark all the forntiers
        for(Pos f : frontiers) stateGrid[f] = Free;
       
        // mark only the significant frontiers
        for(Pos f : centroids) stateGrid[f] = Frontier;
      }
      break;
    case 4: 
      // Calculate the connectedComponents of frontiers
      if (!frontierConComps) {
        frontierConComps = new ConnectedComponents(stateGrid, {Occupied, Unknown, Free, Critical, CriticalLine});
      }
      frontierConComps->update();

      for ( auto it : frontierConComps->connectedComponents){
        PosSet frontiers = it.second.members;

        PosSet significativeFrontiers = getRandomSignificativeFroniers(frontiers, sensorRange, stateGrid, {Occupied});

        // unmark all the forntiers
        for(Pos f : frontiers) stateGrid[f] = Free;
       
        // mark only the significant frontiers
        for(Pos f : significativeFrontiers) stateGrid[f] = Frontier;
      }
      break;
    case 5: {
      // From int set to Pos set
      PosSet significativeFrontiers = getRandomSignificativeFroniers(frontiers, sensorRange, stateGrid, {Occupied});

      // unmark all the forntiers
      for(Pos f : frontiers) stateGrid[f] = Free;
     
      // mark only the significant frontiers
      for(Pos f : significativeFrontiers) stateGrid[f] = Frontier;
      break;}
  }

  // Construct the acution rosmsg
  Auction auctionInfo;

  // Get the info for the auction (topological map), and the GVD as a subproduct
  cout << "debug :: get topoMap" << endl;
  if(!topoMap){
    topoMap = new TopoMap(stateGrid);
  }
  topoMap->update();


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
    PosSet frontiers = it.second.typeMembers[Frontier];

    for (Pos frontier : frontiers) {
      auctionInfo.frontiers.push_back(toPoint2D(frontier));
    }
  }
  auctionInfo.id = auctionId;

  // Increment the auction ID as a new auction will begin
  auctionId++;

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
    Float value = bid.pathLength[i] + segmentValueComponent;

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

/////////
// Aux //
/////////

// custom

bool unobstructedLine(Pos p1, Pos p2, StateGrid &sg, vector<CellState> obstructedTypes){
  for ( Pos p : discretizeLine(p1,p2)){
    if(is_elem(sg[p], obstructedTypes)) return false;
  }
  return true;
}

void accumCircleCells(PosSet &coveredCells, Pos c, Float radius, PosSet &toCover, StateGrid &sg, vector<CellState> nonTraversables){
  PosSet visited;

  list<Pos> toVisit; 
  toVisit.push_back(c);

  while (!toVisit.empty()) {
    Pos p = toVisit.front();
    toVisit.pop_front();

    if(is_elem(p,visited)) continue;

    if(is_elem(p,toCover) && unobstructedLine(c, p, sg, nonTraversables)) coveredCells.insert(p);

    visited.insert(p);

    Float pDist = p.distanceTo(c);

    for (Pos pN : sg.adj(p,nonTraversables)) {
      Float pNDist = pN.distanceTo(c);

      if (pDist < pNDist && pNDist <= radius) {
        toVisit.push_back(pN);
      }
    }
  }
}

  
PosSet getRandomSignificativeFroniers(PosSet &frontiersSet, Float radius, StateGrid& sg, vector<CellState> nonTraversables){

  vector<Pos> frontiersVec = toVec(frontiersSet);

  /* sort(frontiersVec.begin(), frontiersVec.end()); */
  shuffle(std::begin(frontiersVec), std::end(frontiersVec), default_random_engine(2021));
  int i = 0;

  PosSet significativeFrontiers;

  PosSet coveredFrontiers;

  while(coveredFrontiers.size() < frontiersVec.size()){
    if(i>=frontiersVec.size()) FAIL("Out of index: "<<coveredFrontiers.size()<<"/"<<i<<"/"<<frontiersVec.size());

    Pos significativeFrontier = frontiersVec[i];
    i++;

    if(is_elem(significativeFrontier,coveredFrontiers)) continue;

    significativeFrontiers.insert(significativeFrontier);

    accumCircleCells(coveredFrontiers, significativeFrontier, radius, frontiersSet, sg, nonTraversables);
  }

  return significativeFrontiers;
}

// kMeans

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

/////////////
// k-means //
/////////////

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
