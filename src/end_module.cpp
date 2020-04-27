#include <sstream>
#include <string>
#include <list>
#include <vector>
#include <stdio.h>
#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
//#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/Pose.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <tscf_exploration/goalList.h>
#include <tscf_exploration/infoCentro.h>
#include <tscf_exploration/mapMergedInfo.h>
#include <tscf_exploration/frontierReport.h>
#include "std_msgs/String.h"
#include <tscf_exploration/asignacion.h>
#include <tscf_exploration/asignacionCelda.h>
#include "utils.cpp"
#include <ctime>
#include <cstdlib>

using namespace std;

//Topicos para la comunicacion con ROS

ros::Subscriber map_sub;
ros::Subscriber end_robot_sub;
ros::Publisher end_pub;
ros::Publisher coverage;
bool init = false;
double secondsPassed = 0;
float PORCENTAJE = 1.0;
float next_coverage = 0.00;
float coverage_granularity = 0.02;
int TOTALCOVER = 81*81;
int robots_waiting = 0;
int NUM_ROBOTS = 3;

void handleMap(const tscf_exploration::mapMergedInfoConstPtr& msg){
  int y_origin = msg->mapa.info.origin.position.x;
  int x_origin = msg->mapa.info.origin.position.y;
  uint width = msg->mapa.info.width;
  uint height = msg->mapa.info.height;
  int indice_origen = (abs(y_origin) * width) + abs(x_origin);
  int cont = 0;
  for (int i = 0; i < width*height; i++){
    if (msg->mapa.data[i] != -1){
      cont++;
    }
  }

  std_msgs::String cover;
  std::stringstream ssc;
  ssc << "COVERAGE: " <<  cont;
  cover.data = ssc.str();
  if (cont >= TOTALCOVER*next_coverage){
    next_coverage = next_coverage + coverage_granularity;
    coverage.publish(cover);
  }

  //ROS_INFO("cubrimiento %d ", cont);
  if (cont >= (TOTALCOVER*PORCENTAJE)){
    std_msgs::String end_msg;
    std::stringstream ss;
    ss << "END";
    end_msg.data = ss.str();
    end_pub.publish(end_msg);
    ROS_INFO("FINALIZO MODULE !! %d ", cont);
    ros::Rate loop_rate(1);
    loop_rate.sleep();
    ros::shutdown();

  }
}

void handleRobotEnd(const std_msgs::StringConstPtr& msg){
	robots_waiting += 1;
  ROS_INFO("MURIO %s", msg->data);
	bool FIN = (robots_waiting == NUM_ROBOTS);
	if (FIN){
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



int main(int argc, char* argv[]){
	ros::init(argc, argv, "fp_explorer");

	ros::NodeHandle n;
	// // Recibir mapa
	map_sub = n.subscribe("/map_merged", 1, handleMap);
  end_robot_sub = n.subscribe("/end_robots", 1, handleRobotEnd);
	// // Retroalimentacion de el navegador

	end_pub = n.advertise<std_msgs::String>("/end", 1);
  coverage = n.advertise<std_msgs::String>("/coverage", 1);
	//

	ros::spin();

	return 0;
}
