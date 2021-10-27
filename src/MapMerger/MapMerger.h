#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include "../lib/utils.h"
#include "../lib/conversion.h"
#include "nav_msgs/Odometry.h"

class MapMerger {
 public:
  ////////////////
  // Parameters //
  ////////////////

 private:
  //////////
  // vars //
  //////////

  boost::unordered_map<string, PoseStamped> positions;

  ///////////////
  // Functions //
  ///////////////
  void setOccupancy(int,double);
  void updateOccupancy(int, int8_t newOccupancy);

 public:
  //////////
  // vars //
  //////////

  boost::unordered_map<string,int> mapsArrived;
  float sensorRange;
  OccupancyGrid mapMerged;
  vector<double> mapMergedDoubleData;

  ///////////////
  // Functions //
  ///////////////

  // Constructors
  MapMerger();

  // API 
  void mergeMap(const OccupancyGridConstPtr& msg, string name);
  OccupancyGridUpdate mergeMapUpdate(const OccupancyGridUpdateConstPtr& update, string name);
  void updatePose(PoseStamped newPose, string name);

  // others
  bool isInitialized();
};
