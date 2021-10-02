#pragma once

#include <math.h>
#include <ros/ros.h>
#include <stdio.h>
#include <boost/filesystem/operations.hpp>
#include <iostream>
#include <list>
#include <sstream>
#include <string>
#include <vector>
//#include <nav_msgs/Odometry.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <fstream>
#include <ctime>
#include "map_msgs/OccupancyGridUpdate.h"
#include "conversion.h"
#include "std_msgs/String.h"

using namespace std;

inline Float minAngleRep(Float angle){
  return remainder(angle, 2.0 * M_PI);
}

inline bool unobstructedLine(Pos p1, Pos p2, const vector<int8_t> &data, int width,int clearence = 0){
  for (Pos p : discretizeLine(p1,p2)){
    if(p == p2 || p == p1) continue;

    for(int x = -clearence; x <= clearence; x++){
      for(int y = -clearence; y <= clearence; y++){
        Pos pN = p + Pos(x,y);
        int pNInd = toInt(pN,width);
        if(pNInd < data.size() && isOccupied(data[pNInd])){
          return false;
        }
      }
    }
  }
  return true;
}

inline bool unobstructedLine(Pos p1, Pos p2, OccupancyGrid &occupancyGrid, int clearence = 0){
  return unobstructedLine(p1, p2, occupancyGrid.data, occupancyGrid.info.width, clearence); 
}

inline void updateOccupancyGrid(OccupancyGrid& occupancyGrid, const OccupancyGridUpdate& update){
  Pos updateToGlobal = Pos(update.x,update.y);
  for (int updateInd = 0; updateInd < update.data.size(); updateInd++) {
    int globalInd = toInt( updateToGlobal + toPos(updateInd,update.width), occupancyGrid.info.width);
    occupancyGrid.data[globalInd] = update.data[updateInd];
  }
}

/////////////
// logging //
/////////////

inline void logIfNotExists(string name_file, string s) {
  if(!boost::filesystem::exists(name_file)){
    ofstream outfile;
    outfile.open(name_file, ios::out);
    outfile << s << endl;
    outfile.close();
  }
}

inline void logReplace(string name_file, string s) {
  ofstream outfile;
  outfile.open(name_file, ios::out);
  outfile << s << endl;
  outfile.close();
}


inline void logAppend(string name_file, string s) {
  ofstream outfile;
  outfile.open(name_file, ios::out | std::ofstream::app);
  outfile << s << endl;
  outfile.close();
}

///////////////////////////////
// multinode ros interaction //
///////////////////////////////

void inline endExploration(string fileLogDir, ros::Publisher terminationPublisher, string exploredCells, string explorationTime){
  ROS_INFO("Termination started...");
  logIfNotExists(fileLogDir+"/termination.yaml", "explored_cells: "+exploredCells + "\n" + 
                                                 "exploration_time: "+explorationTime);
  std_msgs::String end_msg;
  end_msg.data = "END";
  terminationPublisher.publish(end_msg);
}
