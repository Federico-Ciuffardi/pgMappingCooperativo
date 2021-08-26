#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <pgmappingcooperativo/MapMergedInfo.h>
#include <pgmappingcooperativo/Auction.h>
#include <pgmappingcooperativo/Bid.h>
#include <pgmappingcooperativo/Assignment.h>

#include <iostream>
#include <list>
#include <sstream>
#include <string>
#include <vector>

#include "../lib/GVD/src/TopoMap.h"
#include "../lib/utils.h"
#include "../lib/conversion.h"
#include "../lib/RvizHelper.h"
#include "../lib/Auctioneer.h"
#include "../lib/graph/gnuplot.h"

vector<Pos> kMeans(const vector<Pos>& data, size_t k, size_t maxIterations, Float tolerance = 0);
vector<Pos> embed(vector<Pos> from, vector<Pos> to);
bool contains(vector<Pos> &centers, Float radius, vector<Pos> points);

enum centralMouleState { WaitingAuction = 1, WaitingBids = 2, WaitingFirstBid=3, Resolving = 4 };

typedef string RobotId;
typedef Int SegmentId;

class CentralModule {
 public:
  ////////////////
  // Parameters //
  ////////////////

  // frontierSimplificationMethod:
  // * 0: No frontier simplification
  // * 1: Frontier are clustered into k significant frontiers using kMeans; k is estimated through the robot sensor range
  // * 2: Frontier are clustered into k significant frontiers using kMeans; k is the minimun so that the circles defined
  //      with the significant frontiers as centers and the robot sensor range as the radius contains all the frontiers 
  int frontierSimplificationMethod = 2;

  // kMeans parameters
  Float kMeansMaxIter = 10000;
  Float kMeansTolerance = 0;

  // Global parameters (explained in ../../launch/multirobot.launch)
  int robotNumber;
  float sensorRange;
  float robotSpeed;
  string mapName;
  int mapSize;

  int fileLogLevel;
  string fileLogDir;

 private:
  //////////
  // vars //
  //////////

  // state
  centralMouleState state;
  int auctionId;
  int assignmentId;

  // Map related
  StateGrid stateGrid;

  // frontier related
  boost::unordered_set<int> frontiers;
  ConnectedComponents* frontierConComps = NULL;

  // Auction related
  Auctioneer<RobotId, SegmentId, Pos> auctioneer;

  ///////////////
  // Functions //
  ///////////////

 public:

  //////////
  // vars //
  //////////

  // map related
  nav_msgs::OccupancyGrid occupancyGrid;

  // Auction related
  TopoMap* topoMap = NULL;
  boost::unordered_map<RobotId,boost::unordered_map<Pos,Float>> bids;

  //log related
  int cellCount = 0;

  ///////////////
  // Functions //
  ///////////////

  // Constructors
  CentralModule();

  // getters and setters
  centralMouleState getState();
  void setState(centralMouleState newState);

  // API 
  void updateMap(const MapMergedInfoConstPtr&);
  Auction getAuctionInfo();
  boost::unordered_map<string,Assignment> assign();
  bool saveBid(Bid, RobotId);

  // Destructor
  ~CentralModule();
};
