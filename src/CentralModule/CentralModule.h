#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
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
#include "../lib/IncrementalMap.h"
#include "map_msgs/OccupancyGridUpdate.h"

vector<Pos> kMeans(const vector<Pos>& data, size_t k, size_t maxIterations, Float tolerance = 0);
vector<Pos> embed(vector<Pos> &from, vector<Pos> &to);
bool contains(vector<Pos> &centers, Float radius, vector<Pos> &points);
vector<Pos> getRandomSignificativeFroniers(PosSet &frontiersSet, Float radius, Map& map, vector<CellState> nonTraversables);
vector<Pos> getSignificativeFroniers(PosSet &frontiersSet, Float radius, Map& map, vector<CellState> nonTraversables);

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
  // * 1: Apply kMeans to each connected component of the frontier set, only the frontiers closest to a k-means centroid are used.
  //      k is estimated through the robot sensor range
  // * 2: Apply kMeans to each connected component of the frontier set,, only the frontiers closest to a k-means centroid are used.
  //      k is the minimun so that the circles defined with the significant frontiers as centers and the robot sensor range
  //      as the radius contains all the frontiers 
  // * 3: Apply affinity propagation to each connected component of the frontier set, only the frontiers closest to a affinity
  //      propagation centroid are used.
  // * 4: Use a custom method to get the frontiers to use from the frontier set. (to each connected component of the frontier set)
  int frontierSimplificationMethod = 5;

  // bidSegmentValueComponentCoefficient: 
  // Used it the bidSegmentValueComponent to calculate bid values for forntiers. Check below for what this parameter means for
  // each bidSegmentValueComponent value.
  Float bidSegmentValueComponentCoefficient = 10;

  // bidSegmentValueComponentMode: 
  // The bidSegmentValueComponentMode especifies how the bidSegmentValueComponent is calculated. Then the frontierValue is calculated
  // as `frontierValue = baseValue + bidSegmentValueComponent`, for each frontier. This the bid segment value component, is intended
  // to modify the frontier values so the auction resolution is influenced to keep the robots in their current segment.
  // * 0: No penalization:
  //      bidSegmentValueComponent = 0
  // * 1: Constant penalization:
  //      bidSegmentValueComponent = 0                                   ; if robot inside of the frontier segment
  //                               | bidSegmentValueComponentCoefficient ; otherwise
  // * 2: Constant discount: (the same as 1, more efficient, but could lead to negative values)
  //      bidSegmentValueComponent = -bidSegmentValueComponentCoefficient ; if robot inside of the frontier segment
  //                               |  0                                   ; otherwise
  int bidSegmentValueComponentMode = 2;

  
  // incremental auction info
  // 0: OFF
  // 1: ON
  int auctionInfoIncremental = 1;

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

  // frontier related
  boost::unordered_set<Pos> frontiers;
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


  // Auction related
  TopoMap* topoMap = NULL;
  boost::unordered_map<RobotId,boost::unordered_map<Pos,Float>> bids;

  // Map related
  IncrementalMap map;

  ///////////////
  // Functions //
  ///////////////

  // Constructors
  CentralModule();

  // getters and setters
  centralMouleState getState();
  void setState(centralMouleState newState);

  // API 
  void updateMap(const OccupancyGridConstPtr&);
  void updateMap(const OccupancyGridUpdateConstPtr& update);
  Auction getAuctionInfo();
  boost::unordered_map<string,Assignment> assign();
  bool saveBid(Bid, RobotId);

  // Destructor
  ~CentralModule();
};
