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

  // decay
  /// when a new occupancy grid cell is merged the final value is:
  ///   * merged_cell_value = decay*new_cell_value + (1-decay)*merged_cell_value
  /// where decay belongs to the interval (0,1]
  float decay = 0.1;

 private:
  //////////
  // vars //
  //////////

  boost::unordered_map<string, PoseStamped> positions;

  ///////////////
  // Functions //
  ///////////////
  bool unobstructedLine(Pos p1, Pos p2, const vector<int8_t> &data, int width);

 public:
  //////////
  // vars //
  //////////

  boost::unordered_map<string,int> mapsArrived;
  float sensorRange;
  OccupancyGrid mapMerged;

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
