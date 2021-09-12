#pragma once

#include <math.h>
#include <ros/ros.h>
#include <stdio.h>
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
#include "conversion.h"
#include "map_msgs/OccupancyGridUpdate.h"

using namespace std;

inline bool unobstructedLine(Pos p1, Pos p2, const vector<int8_t> &data, int width,int clearence = 0){
  int threshold = 50;
  for (Pos p : discretizeLine(p1,p2)){
    if(p == p2 || p == p1) continue;

    for(int x = -clearence; x <= clearence; x++){
      for(int y = -clearence; y <= clearence; y++){
        Pos pN = p + Pos(x,y);
        int pNInd = toInt(pN,width);
        if(pNInd < data.size() && data[pNInd] >= threshold){
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

inline string get_current_date_and_time(){
  time_t rawtime;
  struct tm * timeinfo;
  char buffer[80];

  time (&rawtime);
  timeinfo = localtime(&rawtime);

  strftime(buffer,sizeof(buffer),"%d-%m-%Y %H:%M:%S",timeinfo);
  std::string str(buffer);
  return str;
}

inline string get_file_log_name(){
  return "log"+get_current_date_and_time();
}

inline string log_data(string s, string name_file) {
  //struct passwd *pw = getpwuid(getuid());
  // char *homedir = pw->pw_dir;
  //string path = getenv("HOME");
  ofstream outfile;
  //if(name_file.size() == 0) name_file = get_file_log_name();
  outfile.open(name_file +".dat",
               ios::out | std::ofstream::app);
  outfile << s << endl;
  outfile.close();
  return name_file;
}
