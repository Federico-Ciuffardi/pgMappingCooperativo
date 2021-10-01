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
int maxCellCoverage;
string fileLogDir;

boost::unordered_set<int> coveredIndices;

bool firstMap = true;
ros::Time firstMapTime;
int width;

bool endFlag = false;

///////////////////
// Aux Functions //
///////////////////

void checkTermination() {
  if (!endFlag || coveredIndices.size() >= maxCellCoverage) return; // if already ended or not ready to end then skip

  endFlag = true;

  // log termination
  string explorationTime = to_string((ros::Time::now() - firstMapTime).toSec());
  string exploredCells = to_string(coveredIndices.size());
  terminateExploration(fileLogDir, endPub, exploredCells, explorationTime);
}

///////////////
// CallBacks //
///////////////

void mapCallBack(const OccupancyGridConstPtr& msg) {
  if(firstMap){
    firstMapTime = ros::Time::now();
    width = msg->info.width;
    maxCellCoverage /= msg->info.resolution*msg->info.resolution;
    firstMap = false;
  }
  for (int i =0; i < msg->data.size(); i++) {
    if(!isUnknown(msg->data[i])){
      coveredIndices.insert(i);
    }
  }
  checkTermination();
}

void mapUpdateCallBack(const OccupancyGridUpdateConstPtr& update) {
  Pos updateToGlobal = Pos(update->x,update->y);
  for (int updateInd = 0; updateInd < update->data.size(); updateInd++) {
    int globalInd = toInt( updateToGlobal + toPos(updateInd,update->width), width);
    if(!isUnknown(update->data[updateInd])){
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

  // Load params
  FAIL_IFN(n.param<int>   ("/map_size", maxCellCoverage, 0));
  FAIL_IFN(n.param<string>("/file_log_dir", fileLogDir, ""));

  // Initilize Subscribers
  mapSub             = n.subscribe<OccupancyGrid>("/map", 1, mapCallBack);
  mapUpdateSub       = n.subscribe<OccupancyGridUpdate>("/map_update", 1, mapUpdateCallBack);

  // Initilize Publishers
  endPub = n.advertise<std_msgs::String>("/end", 1);

  // spin
  ros::spin();

  return 0;
}
