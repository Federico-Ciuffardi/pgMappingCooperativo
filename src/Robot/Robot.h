#include "../lib/GVD/src/Gvd.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <pgmappingcooperativo/asignacion.h>
#include <pgmappingcooperativo/asignacionCelda.h>
#include <pgmappingcooperativo/frontierReport.h>
#include <pgmappingcooperativo/goalList.h>
#include <pgmappingcooperativo/infoCentro.h>
#include <pgmappingcooperativo/resumenInstancia.h>
#include <pgmappingcooperativo/takeobjetive.h>
#include <pgmappingcooperativo/Auction.h>
#include <pgmappingcooperativo/Bid.h>
#include <pgmappingcooperativo/Assignment.h>
#include <pgmappingcooperativo/FrontierBid.h>
#include <pgmappingcooperativo/mapMergedInfo.h>
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

  // Segmentation relatied
  boost::unordered_map<Pos,list<GvdVecGraph::Vertex>> paths;
  boost::unordered_map<Pos,float> pathCosts;

  GvdVecGraph gvd;
  Pos my_pos;
  Pos my_segment;

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
  pgmappingcooperativo::mapMergedInfo map_merged;
  StateGrid grid;
  geometry_msgs::Point position;

  ///////////////
  // Functions //
  ///////////////

  // Constructor
  Robot();

  // getters and setters
  int getRobotId();

  // API
  pgmappingcooperativo::Bid getBid(pgmappingcooperativo::Auction msg);
  pgmappingcooperativo::goalList getPathTo(Pos p);

  // Aux
  bool addToGraph(Pos, GvdVecGraph& graph, StateGrid&);
  void addToGraph(PosSet&, GvdVecGraph& graph, StateGrid&);

};
