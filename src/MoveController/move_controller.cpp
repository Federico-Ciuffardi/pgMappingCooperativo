#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tscf_exploration/mapMergedInfo.h>
#include <ctime>
#include <sstream>
#include <string>
#include "TCPClient.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "tf/transform_datatypes.h"
#include "tscf_exploration/goalList.h"

// Constantes

const double TOLERANCE_GOAL = 1.5;//0.30;
const double TOLERANCE_WAYPOINTS = 1.5;// 0.50;
const double SPEED = 0.8; //0.5

// const std::string ODOM_FRAME = "p3dx0_tf/odom";

using namespace std;

// VARIABLES
ros::Publisher path_result_pub;
ros::Publisher path_info_pub;
ros::Subscriber goalPath_sub;
ros::Subscriber pose_sub;
ros::Subscriber map_merged_sub;
TCPClient client;
tscf_exploration::goalList path;
tscf_exploration::goalList path_saved;
geometry_msgs::PoseStamped position;
geometry_msgs::PoseStamped position_old;
nav_msgs::OccupancyGrid odometry_map;
ros::Subscriber end_sub;

bool wait_last_point = false;
int path_step = 0;
int msg_id = 0;
int estado = 0;
string name_space = "";
bool start = false;
int cont = 0;
bool FIN = false;
int pathflag = 0;
std::string end_msg("END");
float metros = 0.0;
clock_t startTime;

void handleEnd(const std_msgs::StringConstPtr& msg) {
  std::string str1(msg->data.c_str());
  FIN = (str1.compare(end_msg) == 0);
  if (FIN) {
    std_msgs::String msg_info;
    std::stringstream ss9;
    int sed = (clock() - startTime);
    ss9 << "SUCCES: " << metros << " " << sed << " " << path.indice;
    msg_info.data = ss9.str();
    path_info_pub.publish(msg_info);
    // ROS_INFO("%s :: RECORRIO %f METROS", name_space.c_str(), metros);
  }
}

void poseCallback(const geometry_msgs::PoseStamped& msg) {
  position_old = position;
  position = msg;
}

void setPath(const tscf_exploration::goalList& msg) {
  if (msg.listaGoals.size() != 0) {
    path_saved = msg;
    // ROS_INFO("MOVE CONTROLLER :: Saving new path. Long -> %lu",
    // msg.listaGoals.size());
    pathflag = 1;
  }
}

void send_point(geometry_msgs::Point next_point) {
  // ROS_INFO("Creating path step to (%d,%d)",next_point.x,next_point.y);
  stringstream ss;
  ss << msg_id << " " << name_space << ".waypoint "
     << "goto "
     << "[" << next_point.x << ", " << next_point.y << ", " << next_point.z << ", "
     << TOLERANCE_GOAL << ", " << SPEED << "]"
     << "\n";
  string msg = ss.str();
  // ROS_INFO("Sending path step ---> X = %f , Y = %f , Z = %f", next_point.x,
  // next_point.y, next_point.z );
  client.send_msg(msg);
  msg_id++;
}

void saveMap(const tscf_exploration::mapMergedInfoConstPtr& msg) {
  ROS_DEBUG(" MOVE CONTROLLER :: Guardo Mapa");
  odometry_map.info = msg->mapa.info;
  odometry_map.header = msg->mapa.header;
  odometry_map.data = msg->mapa.data;
}

float getDistance(int target) {
  float dx, dy;
  dx = position.pose.position.x - path.listaGoals[target].x;
  dy = position.pose.position.y - path.listaGoals[target].y;
  return sqrt(pow(dx, 2) + pow(dy, 2));
}

float getNextDistance(int target) {
  float dx, dy;
  dx = path.listaGoals[target - 1].x - path.listaGoals[target].x;
  dy = path.listaGoals[target - 1].y - path.listaGoals[target].y;
  return sqrt(pow(dx, 2) + pow(dy, 2));
}

int main(int argc, char** argv) {
  // ROS_INFO("Initializing node");
  startTime = clock();
  // Initializing ros
  ros::init(argc, argv, "simple_navigation_goals");

  bool primera = true;
  char buffer_ns[20];
  ros::NodeHandle n;
  name_space = n.getNamespace().substr(1, 5);

  ROS_DEBUG("Initializing node %s", name_space.c_str());

  goalPath_sub = n.subscribe("goalPath", /*1*/ 10, setPath);
  pose_sub = n.subscribe("pose", 1, poseCallback);
  map_merged_sub = n.subscribe("/map_merged", 1, saveMap);
  end_sub = n.subscribe("/end", 1, handleEnd);
  path_result_pub = n.advertise<std_msgs::String>("path_result", 10);
  path_info_pub = n.advertise<std_msgs::String>("path_info", 10);

  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Initializing morse comunication
  ROS_INFO("Starting TCP conection");
  client = TCPClient();
  client.setup("localhost", 4000);
  ROS_INFO("End TCP conection");
  float last_distancie = 0;
  ros::Rate loop_rate(10);
  int secondsPassed;
  // while(path_result_pub.getNumSubscribers() == 0){};
  while (ros::ok()) {
    switch (estado) {
      case 0: {
        // ROS_INFO("Nothing to do");
        if (path_step != path.listaGoals.size()) {
          // //ROS_INFO("MOVE CONTROLLER :: Esperando %s", name_space.c_str());
          estado = 1;
        }
        loop_rate.sleep();
        break;
      }
      case 1: {  // AVANZANDO A UN PUNTO
        // ROS_INFO("Sending new objetive %f, %f, %f,",
        // path.listaGoals[path_step].x, path.listaGoals[path_step].y,
        // path.listaGoals[path_step].z);
        send_point(path.listaGoals[path_step]);
        path_step++;
        if (path_step != path.listaGoals.size()) {
          metros = metros + getNextDistance(path_step);
        }
        estado = 2;
        break;
      }
      case 2: {
        float dist_to_target = getDistance(path_step - 1);
        // if (dist_to_target == last_distancie){
        //	send_point(path.listaGoals[path_step - 1 ]);
        //	ROS_INFO("resendig target");
        //}
        if (dist_to_target <= TOLERANCE_WAYPOINTS) {
          // ROS_INFO("Arriving");
          if (path_step == path.listaGoals.size()) {  // if wait_last_point
            estado = 4;
          } else {
            estado = 1;
          }
        }
        ////ROS_INFO("Distance ---> %f, X ---> %f, Y ---> %f", dist_to_target,
        /// path.listaGoals[path_step].x, path.listaGoals[path_step].y);
        break;
      }
      case 4: {  // AVANZANDO AL ULTIMO PUNTO
        float dist_to_target = getDistance(path_step - 1);
        ROS_INFO("Waiting for last point");
        //string recv = "";
        //stringstream ss;
        //ss << (msg_id - 1) << " SUCCES";
        //string msg = ss.str();
        //recv = client.receive_msg();
        //if (recv.find(msg)) {
        if (dist_to_target <= TOLERANCE_WAYPOINTS) {
          ROS_INFO("Path end %s", name_space.c_str());
          estado = 0;
          //std_msgs::String msg_succes;
          //std::stringstream ss2;
          //secondsPassed = (clock() - startTime);
          //ss2 << " " << metros << " " << secondsPassed << " " << path.indice;
          //msg_succes.data = ss2.str();
          std_msgs::String str;
          str.data = "signal";
          path_result_pub.publish(str);//msg_succes);
          path_info_pub.publish(str);//msg_succes);
        }
        break;
      }
    }
    if (pathflag == 1) {
      std_msgs::String msg_info;
      std::stringstream ss9;
      int sed = (clock() - startTime);
      ss9 << "SUCCES: " << metros << " " << sed << " " << path.indice;
      msg_info.data = ss9.str();
      path_info_pub.publish(msg_info);
      start = true;
      cont = 0;
      path_step = 0;
      pathflag = 0;
      estado = 0;
      path = path_saved;
    }
  }

  return 0;
}
