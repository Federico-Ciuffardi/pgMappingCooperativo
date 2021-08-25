#include "../lib/GVD/src/Gvd.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <pgmappingcooperativo/GoalList.h>
#include <pgmappingcooperativo/Auction.h>
#include <pgmappingcooperativo/Bid.h>
#include <pgmappingcooperativo/Assignment.h>
#include <pgmappingcooperativo/MapMergedInfo.h>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <list>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <sstream>
#include <string>
#include <vector>
#include "../lib/utils.h"
#include "../lib/conversion.h"
#include "../lib/RvizHelper.h"

class Robot {
 public:
  ////////////////
  // Parameters //
  ////////////////

  // Global parameters (explained in ../../launch/multirobot.launch)
  float sensorRange;
  float robotSpeed;

 private:
  //////////
  // vars //
  //////////

  // auction related
  boost::unordered_map<Pos,list<GvdVecGraph::Vertex>> paths;
  boost::unordered_map<Pos,float> pathCosts;

  GvdVecGraph gvd;

  ///////////////
  // Functions //
  ///////////////

 public:
  //////////
  // vars //
  //////////

  string name;
  int lastAssignmentId = -1;
  int lastAuctionId = -1;
  nav_msgs::OccupancyGrid occupancyGrid;
  StateGrid grid;
  Point position;

  ///////////////
  // Functions //
  ///////////////

  // Constructor
  Robot();

  // getters and setters

  // API
  Bid getBid(Auction msg);
  GoalList getPathTo(Pos p);

  // Aux
  bool addToGraph(Pos, GvdVecGraph& graph, StateGrid&);
  void addToGraph(PosSet&, GvdVecGraph& graph, StateGrid&);

};
