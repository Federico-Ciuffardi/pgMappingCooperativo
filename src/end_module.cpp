#include <bits/stdc++.h>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <stdio.h>
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

// Topicos para la comunicacion con ROS

ros::Subscriber map_sub;
ros::Subscriber end_robot_sub;
ros::Publisher end_pub;
ros::Publisher coverage;
bool init = false;
double secondsPassed = 0;
float PORCENTAJE = 1;  // 0.10;
float next_coverage = 0.00;
float coverage_granularity = 0.02;
int TOTALCOVER;  // 81 * 81;
int robots_waiting = 0;
int NUM_ROBOTS;

void handleMap(const nav_msgs::OccupancyGridConstPtr& msg) {
  int y_origin = msg->info.origin.position.x;
  int x_origin = msg->info.origin.position.y;
  uint width = msg->info.width;
  uint height = msg->info.height;
  int indice_origen = (abs(y_origin) * width) + abs(x_origin);
  int cont = 0;
  for (int i = 0; i < width * height; i++) {
    if (msg->data[i] != -1) {
      cont++;
    }
  }

  /*std_msgs::String cover;
  std::stringstream ssc;
  ssc << "COVERAGE: " << cont;
  cover.data = ssc.str();
  if (cont >= TOTALCOVER * next_coverage) {
    next_coverage = next_coverage + coverage_granularity;
    coverage.publish(cover);
  }*/
  // ROS_INFO(" van %d celdas exploradas ", cont);
  // ROS_INFO("cubrimiento %d ", cont);
  if (cont >= (TOTALCOVER * PORCENTAJE)) {
    std_msgs::String end_msg;
    std::stringstream ss;
    ss << "END";
    end_msg.data = ss.str();
    ROS_INFO(" Stopping at %f ", cont / (float(TOTALCOVER)));
    end_pub.publish(end_msg);
  }
}

void handleRobotEnd(const std_msgs::StringConstPtr& msg) {
  robots_waiting += 1;
  ROS_INFO("MURIO %s", msg->data.c_str());
  bool FIN = (robots_waiting == NUM_ROBOTS);
  if (FIN) {
    std_msgs::String msg_request2;
    std::stringstream ss2;
    ss2 << "END";
    msg_request2.data = ss2.str();
    end_pub.publish(msg_request2);
    ros::Rate loop_rate(1);
    loop_rate.sleep();
    ros::shutdown();
  }
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "fp_explorer");

  ros::NodeHandle n;

  n.param<int>("/starting_robot_number", NUM_ROBOTS, 0);
  string map_name;
  n.param<string>("/map_name", map_name, "");
  if (map_name == "office") {
    TOTALCOVER = 5832;
  } else if (map_name == "labyrinth") {
    TOTALCOVER = 81 * 81;
  }
  ROS_INFO("Map size = %d ", TOTALCOVER);
  // Recibir map
  map_sub = n.subscribe("/map", 1, handleMap);
  end_robot_sub = n.subscribe("/end_robots", 1, handleRobotEnd);
  // Retroalimentacion de el navegador

  end_pub = n.advertise<std_msgs::String>("/end", 1);
  coverage = n.advertise<std_msgs::String>("/coverage", 1);
  //

  ros::spin();

  return 0;
}
