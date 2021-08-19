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
 private:
  ////////////////
  // Parameters //
  ////////////////

  float sensor_range;
  float lado;

  //////////
  // vars //
  //////////

  // Segmentation relatied
  geometry_msgs::Point position;
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

  std::string nombreRobot;
  int lastAssignmentId = -1;
  int lastAuctionId = -1;
  pgmappingcooperativo::mapMergedInfo map_merged;
  StateGrid grid;

  ///////////////
  // Functions //
  ///////////////

  // Constructor
  Robot();

  // getters and setters
  void setPosition(int x, int y);
  geometry_msgs::Point getPosition();
  Pos getGVDPos();
  void setNombre(std::string nom);
  string getNombre();
  int getRobotId();
  Pos getOffset();
  void getGrid();

  pgmappingcooperativo::Bid getBid(pgmappingcooperativo::Auction msg);


  void savePose(const geometry_msgs::Pose msg);

  geometry_msgs::Point pos_to_real_p3d(Pos p);

  bool addToGraph(Pos, GvdVecGraph& graph, StateGrid&);
  PosSet addToGraph(PosSet&, GvdVecGraph& graph, StateGrid&);

  pgmappingcooperativo::goalList getPathTo(Pos p);
};
