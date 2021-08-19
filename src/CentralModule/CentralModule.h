#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <pgmappingcooperativo/asignacion.h>
#include <pgmappingcooperativo/asignacionCelda.h>
#include <pgmappingcooperativo/frontierReport.h>
#include <pgmappingcooperativo/infoCentro.h>
#include <pgmappingcooperativo/mapMergedInfo.h>
#include <pgmappingcooperativo/takeobjetive.h>
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

enum centralMouleState { WaitingAuction = 1, WaitingBids = 2, WaitingFirstBid=3, Resolving = 4 };

typedef string RobotId;
typedef int    SegmentId;

class CentralModule {
 public:
  ////////////////
  // Parameters //
  ////////////////

  // frontierSimplificationMethod:
  // * 0: No frontier simplification
  // * 1: Frontier is clustered into significant frontiers considering the robot sensor range
  int frontierSimplificationMethod = 1;

  // Global parameters (explained in ../../launch/multirobot.launch)
  int robotNumber;
  float sensorRange;
  float robotSpeed;
  string mapName;
  int mapSize;
  float cellSize;

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
  vector<int> frontierCenters;
  ConnectedComponents* frontierConComps = NULL;

  // Auction related
  Auctioneer<RobotId, SegmentId, Pos> auctioneer;
  boost::unordered_map<SegmentId,boost::unordered_set<Pos>> segmentsWithFrontiers;
  CriticalInfos cis;

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
  vector<int> getFrontierCenters();
  void setFrontierCenters(vector<int> newFrontierCenters);

  centralMouleState getEstado();
  void setState(centralMouleState newState);

  int getNumRobots();
  void setNumRobots(int newNumRobots);

  nav_msgs::OccupancyGrid getMap();

  // Other
  void updateMap(const pgmappingcooperativo::mapMergedInfoConstPtr&);
  pgmappingcooperativo::Auction getAuctionInfo();
  boost::unordered_map<string,pgmappingcooperativo::Assignment> assign();
  bool saveBid(pgmappingcooperativo::Bid, RobotId);

  // Destructor
  ~CentralModule();
};
