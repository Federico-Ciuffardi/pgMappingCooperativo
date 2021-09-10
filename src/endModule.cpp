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
int maxCellCoverage = 5832;
boost::unordered_set<int> coveredIndices;

bool first = true;
int width;

///////////////////
// Aux Functions //
///////////////////

void checkTermination() {
  if (coveredIndices.size() >= maxCellCoverage) {
    std_msgs::String end_msg;
    std::stringstream ss;
    ss << "END";
    end_msg.data = ss.str();
    ROS_INFO("Stopping");
    endPub.publish(end_msg);
  }
}

///////////////
// CallBacks //
///////////////

void mapCallBack(const OccupancyGridConstPtr& msg) {
  if(first){
    width = msg->info.width;
    maxCellCoverage /= msg->info.resolution*msg->info.resolution;
    first = false;
  }
  for (int i =0; i < msg->data.size(); i++) {
    if(msg->data[i] != -1){
      coveredIndices.insert(i);
    }
  }
  checkTermination();
}

void mapUpdateCallBack(const OccupancyGridUpdateConstPtr& update) {
  Pos updateToGlobal = Pos(update->x,update->y);
  for (int updateInd = 0; updateInd < update->data.size(); updateInd++) {
    int globalInd = toInt( updateToGlobal + toPos(updateInd,update->width), width);
    if(update->data[updateInd] != -1){
      coveredIndices.insert(globalInd);
    }
  }

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
