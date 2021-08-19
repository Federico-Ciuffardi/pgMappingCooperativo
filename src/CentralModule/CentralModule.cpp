#include "CentralModule.h"
#include <cstddef>
#include <vector>

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
centralMouleState CentralModule::getEstado() {
  return state;
};

void CentralModule::setState(centralMouleState newState) {
  state = newState;
};

int CentralModule::getNumRobots() {
  return robotNumber;
};

void CentralModule::setNumRobots(int newNumRobots) {
  robotNumber = newNumRobots;
};

///////////
// Other //
///////////

void CentralModule::updateMap(const pgmappingcooperativo::mapMergedInfoConstPtr& newMap) {
  // store occGrid
  occupancyGrid = newMap->mapa;

  // store frontiers
  boost::unordered_set<int> newFrontiers(newMap->frontera.begin(), newMap->frontera.end());
  frontiers = newFrontiers;
}

// Get the info to start an Auction
pgmappingcooperativo::Auction CentralModule::getAuctionInfo() {
  // Convert a occupancyGrid and a frontier list to a stateGrid
  cout << "debug :: convert a occupancyGrid and a frontier list to a stateGrid" << endl;
  stateGrid = toStateGrid(occupancyGrid, frontiers, &cellCount);

  cout << "debug :: get significant frontiers" << endl;
  // Get the significant frontiers taking into account the vision range of the robots
  switch (frontierSimplificationMethod) {
    case 1:
      // Calculate the connectedComponents of frontiers
      if (!frontierConComps) {
        frontierConComps = new ConnectedComponents(stateGrid, {Occupied, Unknown, Free, Critical, CriticalLine});
      }
      frontierConComps->update();

      for ( auto it : frontierConComps->connectedComponents){
        vector<Pos> frontiers = toVec(it.second.members);

        // Calculate k y and execute kMeans
        int k = ceil(frontiers.size()/(sensorRange*2.0));

        vector<Pos> centroids = kMeans(frontiers,k,10000);

        // unmark all the forntiers
        for(Pos f : frontiers) stateGrid[f] = Free;
       
        // mark only the significant frontiers
        for(Pos f : embed(centroids, frontiers)) stateGrid[f] = Frontier;

      }
      break;
  }

  // Construct the acution rosmsg
  pgmappingcooperativo::Auction auctionInfo;

  // Get the info for the auction (segments and frontiers), and the GVD as a subproduct
  cout << "debug :: gvd and cis" << endl;
  if(!topoMap){
    topoMap = new TopoMap(stateGrid);
  }
  topoMap->update();

  GvdGraph& gvd = *(topoMap->gvd->graphGvd);

  for(auto it : topoMap->segmenter->connectedComponents){
    SegmentId segmentId = it.first;
    PosSet frontiers = it.second.typeMembers[Frontier];
    if(!frontiers.empty()){
      segmentsWithFrontiers[segmentId] = frontiers;
    }
  }

  // Turn the boost GVD to a ros message
  cout << "debug :: gvd to rosmsg" << endl;
  for (GvdGraph::Vertex v : gvd) {
    auctionInfo.gvd.vertices.push_back(toPoint2D(gvd.g[v].p));
    auctionInfo.vertex_segment.push_back(toPoint2D(gvd.g[v].segment));
    for (GvdGraph::Vertex nv : gvd.adj(v)) {
      pgmappingcooperativo::Edge e;
      e.from = toPoint2D(gvd.g[v].p);
      e.to = toPoint2D(gvd.g[nv].p);
      auctionInfo.gvd.edges.push_back(e);
    }
  }

  // Turn the segment and frontiers info into a ros message
  cout << "debug :: cis to rosmsg" << endl;
   for (auto it : segmentsWithFrontiers) {
     SegmentId segmentId = it.first;
     PosSet frontiers = it.second;
 
     for (Pos frontier : frontiers) {
       auctionInfo.frontiers.push_back(toPoint2D(frontier));
       auctionInfo.frontiers_segment.push_back(segmentId);
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
bool CentralModule::saveBid(pgmappingcooperativo::Bid bid, RobotId robotId) {
  // skip if old
  if (assignmentId != bid.id)  return false; 

  // Store the bid of the robot 'name'
  for (int i = 0; i < bid.frontiers.size(); i++) {
    Float value = bid.values[i];
    Pos frontier = toPos(bid.frontiers[i]);

    auctioneer.addBid(robotId, topoMap->segmenter->idGrid[frontier] , frontier, value);
    bids[robotId][frontier] = value;
  }

  return true;
}

// Resolve the auction given the current bids
boost::unordered_map<string, pgmappingcooperativo::Assignment> CentralModule::assign() {
  // Resolve the auction
  boost::unordered_map<RobotId, Pos> resolution = auctioneer.resolveAuction(segmentsWithFrontiers);

  // Process the auction resolution and bundle it as a ros message
  boost::unordered_map<string, pgmappingcooperativo::Assignment> resolutionRosMessages;

  for (auto it : resolution) {
    RobotId robotId = it.first;
    Pos frontier = it.second;

    pgmappingcooperativo::Assignment assignment;
    assignment.id = assignmentId;
    assignment.frontier = toPoint2D(frontier);

    resolutionRosMessages[robotId] = assignment;
  }

  // Increment the segment assignment ID to set to the next assignment
  assignmentId++;

  // return the info of the assignment bundled as a ros message
  return resolutionRosMessages;
}

/////////
// Aux //
/////////

// Embed the pos from `from` to pos on `to`
vector<Pos> embed(vector<Pos> from, vector<Pos> to){
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
