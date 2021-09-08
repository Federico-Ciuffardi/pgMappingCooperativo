#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pgmappingcooperativo/MapMergedInfo.h>
#include <unistd.h>
#include <iostream>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include "../lib/utils.h"
#include "../lib/conversion.h"
#include "nav_msgs/Odometry.h"

class MapMerger {
 private:
  //////////
  // vars //
  //////////

  bool init;

  boost::unordered_map<string, PoseStamped> positions;
  boost::unordered_map<string, OccupancyGrid> robotMaps;

  ///////////////
  // Functions //
  ///////////////

  void initMapMerger(const OccupancyGridConstPtr& msg);
  bool isAnyRobotCloser(float dist, int ind, string name);

 public:
  //////////
  // vars //
  //////////

  float sensorRange;
  OccupancyGrid mapMerged;

  ///////////////
  // Functions //
  ///////////////

  // Constructors
  MapMerger();

  // API 
  void updateMap(const OccupancyGridConstPtr& newMap, string name);
  void saveRobotMap(const OccupancyGridConstPtr& msg, string name);
  void updatePose(PoseStamped newPose, string name);
};
