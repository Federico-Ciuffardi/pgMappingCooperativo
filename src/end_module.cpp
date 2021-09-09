#include <bits/stdc++.h>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <stdio.h>
#include <cstdint>
#include <iostream>
#include <list>
#include <sstream>
#include <string>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/MapMetaData.h>
#include <std_msgs/String.h>
#include <pgmappingcooperativo/GoalList.h>
#include <cstdlib>
#include <ctime>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "lib/utils.h"
#include "std_msgs/String.h"

using namespace std;

///////////////
// Variables //
///////////////

// Topics
/// Subscribers
ros::Subscriber end_robot_sub;
ros::Subscriber mapSub;
ros::Subscriber mapUpdateSub;

/// Publishers
ros::Publisher endPub;

// others
OccupancyGrid occupancyGrid;
int maxCellCoverage = 5832;

///////////////////
// Aux Functions //
///////////////////

void checkTermination() {
  int cont = 0;
  for (uint32_t value : occupancyGrid.data) {
    cont += value != -1;
  }

  if (cont >= maxCellCoverage) {
    std_msgs::String end_msg;
    std::stringstream ss;
    ss << "END";
    end_msg.data = ss.str();
    ROS_INFO(" Stopping at %f ", cont / (float)maxCellCoverage);
    endPub.publish(end_msg);
  }
}

///////////////
// CallBacks //
///////////////

void mapCallBack(const OccupancyGridConstPtr& msg) {
  occupancyGrid = *msg;
  checkTermination();
}

void mapUpdateCallBack(const OccupancyGridUpdateConstPtr& msg) {
  updateOccupancyGrid(occupancyGrid, *msg);
  checkTermination();
}

//////////
// main //
//////////

int main(int argc, char* argv[]) {
  // Init node
  ros::init(argc, argv, "end_module");
  ros::NodeHandle n;

  // Initilize Subscribers
  mapSub             = n.subscribe<OccupancyGrid>("/map", 1, mapCallBack);
  mapUpdateSub       = n.subscribe<OccupancyGridUpdate>("/map_update", 1, mapUpdateCallBack);

  // Initilize Publishers
  endPub = n.advertise<std_msgs::String>("/end", 1);

  // spin
  ros::spin();

  return 0;
}
