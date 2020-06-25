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
#include "../lib/conversion.h"
#include "../lib/GVD/GVD.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include "../GlobalParameters.h"

// Constantes

const double TOLERANCE_GOAL = 0.75;       // 0.30;
const double TOLERANCE_WAYPOINTS = 2.25;  // 0.50;
const double SPEED = ROBOT_SPEED;                // 0.5

// const std::string ODOM_FRAME = "p3dx0_tf/odom";

using namespace std;

// VARIABLES
ros::Publisher path_result_pub;
ros::Publisher path_info_pub;
ros::Subscriber goalPath_sub;
ros::Subscriber pose_sub;
//ros::Subscriber map_sub;
ros::Subscriber scan_sub;
TCPClient client;
tscf_exploration::goalList path;
tscf_exploration::goalList path_saved;
geometry_msgs::PoseStamped position;
geometry_msgs::PoseStamped position_old;
//nav_msgs::OccupancyGrid odometry_map;
ros::Subscriber end_sub;

ros::Timer idleTimer;

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
sensor_msgs::LaserScan laserScan;
float angleToTarget;

//aux funcs
float getDistance(int target) {
  float dx, dy;
  dx = position.pose.position.x - path.listaGoals[target].x;
  dy = position.pose.position.y - path.listaGoals[target].y;
  return sqrt(pow(dx, 2) + pow(dy, 2));
}

float getDistance(geometry_msgs::Point p3d) {
  float dx, dy;
  dx = position.pose.position.x - p3d.x;
  dy = position.pose.position.y - p3d.y;
  return sqrt(pow(dx, 2) + pow(dy, 2));
}

float getNextDistance(int target) {
  float dx, dy;
  dx = path.listaGoals[target - 1].x - path.listaGoals[target].x;
  dy = path.listaGoals[target - 1].y - path.listaGoals[target].y;
  return sqrt(pow(dx, 2) + pow(dy, 2));
}

void notification(char* data){
  std_msgs::String str;
  str.data = data;
  path_result_pub.publish(str);  // msg_succes);
  path_info_pub.publish(str);    // msg_succes);
  //idleTimer.start();
}

void idleTimerRoutine(const ros::TimerEvent&){
  notification("idle");
} 

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


tscf_exploration::goalList trim_path(tscf_exploration::goalList msg){
  int path_start = 0;
  for(int i = 0; i<msg.listaGoals.size();i++){
    float dist_to_target = getDistance(msg.listaGoals[i]);
    if (dist_to_target <= TOLERANCE_WAYPOINTS) {
      path_start = i;
    }
  }
  msg.listaGoals = std::vector<geometry_msgs::Point, std::allocator<geometry_msgs::Point>>(
    msg.listaGoals.begin()+path_start,
    msg.listaGoals.end()
  );
  return msg;
}

void setPath(const tscf_exploration::goalList& msg) {
  if (msg.listaGoals.size() != 0) {
    path_saved = trim_path(msg);
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

/*void saveMap(const nav_msgs::OccupancyGridConstPtr& msg) {
  ROS_DEBUG(" MOVE CONTROLLER :: Guardo Mapa");
  odometry_map.info = msg->info;
  odometry_map.header = msg->header;
  odometry_map.data = msg->data;
}*/

void saveScan(const sensor_msgs::LaserScanConstPtr& msg) {
  ROS_DEBUG(" MOVE CONTROLLER :: Guardo Mapa");
  laserScan = *msg;
}

//int last_safe_index = -1;
int last_unsafe_index = -1;
/*// -1 if is safe else the first obstacle index is returned
int safePath(int from_index, int lookahead){
  int safe = -1;
  geometry_msgs::Point origin_p3d  = odometry_map.info.origin.position;
  origin_p3d.x = -origin_p3d.x-1;
  origin_p3d.y = -origin_p3d.y;
  int origin_p1d = p3d_to_p1d(origin_p3d,0,odometry_map.info.height);
  //ROS_INFO("origin = %d",origin_p1d);
  for(int i = from_index; i< from_index + lookahead && i<path.listaGoals.size(); i++){
    int final_p1d = p3d_to_p1d(path.listaGoals[i],origin_p1d,odometry_map.info.height); 
    ROS_INFO("final_p1d = %d, is -> %d ",final_p1d,odometry_map.data[final_p1d]);
    if(odometry_map.data[final_p1d] == 100){
      return i;
    }
  }
  return safe;
}*/

double getAngle(geometry_msgs::Quaternion q){
  tf::Quaternion quat_tf;

  tf::quaternionMsgToTF(q , quat_tf);
  tf::Matrix3x3 m(quat_tf);

  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

double normalize(double angle){
    angle = fmod(angle + M_PI,M_PI*2);
    if (angle < 0)
        angle += M_PI*2;
    return angle - M_PI;
}

bool isSafe(float target_angle, float safe_distance = 2){

  geometry_msgs::Point current_pos = position.pose.position;

  double robot_angle = getAngle(position.pose.orientation);
  //ROS_INFO("angulo del robotitio %f, dir objetivo (%f, %f), angulo del objetivo %f",robot_angle,target_pos.x,target_pos.y ,target_angle);
  double target_angle_adjusted= normalize(target_angle-robot_angle);

  double increment = laserScan.angle_increment;
  int laser_index = (target_angle_adjusted - laserScan.angle_min)/increment;
  //ROS_INFO("angulo inicial = %f",laserScan.angle_min);

  //ROS_INFO("%s :: pos objetivo (%f,%f), angulo del objetivo %f, angulo del robot %f",name_space.c_str(), target_pos.x,target_pos.y, target_angle,robot_angle);


  //ROS_INFO("%s :: angulo del objetivo ajustado %f, indice %d, representa %f, valor laser %f, umbral %f",name_space.c_str(), target_angle_adjusted, laser_index, laser_index*increment+laserScan.angle_min, laserScan.ranges[laser_index],safe_distance);
  
  return laserScan.ranges[laser_index] > safe_distance;
}

float getTargetAngle(int index){
  geometry_msgs::Point current_pos = position.pose.position;
  geometry_msgs::Point target_pos = path.listaGoals[index];
  target_pos.x -= current_pos.x;
  target_pos.y -= current_pos.y;
  return atan2(target_pos.y,target_pos.x);
}

/*bool isSafe(int index){
  geometry_msgs::Point current_pos = position.pose.position;
  geometry_msgs::Point target_pos = path.listaGoals[index];
  target_pos.x -= current_pos.x;
  target_pos.y -= current_pos.y;

  if(target_pos.x<0.5 && target_pos.y < 0.5 ){
    return true;
  }

  double robot_angle = getAngle(position.pose.orientation);
  double target_angle = atan2(target_pos.y,target_pos.x);
  //ROS_INFO("angulo del robotitio %f, dir objetivo (%f, %f), angulo del objetivo %f",robot_angle,target_pos.x,target_pos.y ,target_angle);
  double target_angle_adjusted= normalize(target_angle-robot_angle);

  double increment = laserScan.angle_increment;
  int laser_index = (target_angle_adjusted - laserScan.angle_min)/increment;
  //ROS_INFO("angulo inicial = %f",laserScan.angle_min);

  //ROS_INFO("%s :: pos objetivo (%f,%f), angulo del objetivo %f, angulo del robot %f",name_space.c_str(), target_pos.x,target_pos.y, target_angle,robot_angle);

  double safe_distance =  1 + getDistance(index);
  //ROS_INFO("%s :: angulo del objetivo ajustado %f, indice %d, representa %f, valor laser %f, umbral %f",name_space.c_str(), target_angle_adjusted, laser_index, laser_index*increment+laserScan.angle_min, laserScan.ranges[laser_index],safe_distance);
  
  return laserScan.ranges[laser_index] > safe_distance;
}*/

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

  //idleTimer = n.createTimer(ros::Duration(30.0), idleTimerRoutine, true,false);

  goalPath_sub = n.subscribe("goalPath", /*1*/ 10, setPath);
  pose_sub = n.subscribe("pose", 1, poseCallback);
  //map_sub = n.subscribe<nav_msgs::OccupancyGrid>("map", 1, saveMap);
  scan_sub = n.subscribe<sensor_msgs::LaserScan>("scan", 1, saveScan);
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
      //last_safe_index = -1;
      last_unsafe_index = -1;
      path = path_saved;
    }

    loop_rate.sleep();

    switch (estado) {
      case 0: {
        if (path_step != path.listaGoals.size()) {
          estado = 1;
        }
        break;
      }
      case 1: {  // AVANZANDO A UN PUNTO
        send_point(path.listaGoals[path_step]);
        angleToTarget = getTargetAngle(path_step);
        path_step++;
        if (path_step != path.listaGoals.size()) {
          metros = metros + getNextDistance(path_step);
        }
        estado = 2;
        break;
      }
    }

    if(estado <2){ continue; }

    if(!isSafe(angleToTarget)){
      if(last_unsafe_index != path_step){
        last_unsafe_index = path_step;
        ROS_INFO("obstacle");
        notification("obstacle");
        send_point(position.pose.position);
      }
    }else{
      if(last_unsafe_index == path_step){
        send_point(path.listaGoals[path_step]);
      }
    }

    switch (estado) {
      case 2: {
        float dist_to_target = getDistance(path_step - 1);
        if (dist_to_target <= TOLERANCE_WAYPOINTS) {
          // ROS_INFO("Arriving");
          if (path_step == path.listaGoals.size()) {  // if wait_last_point
            estado = 4;
          } else {
            estado = 1;
          }
        }
        break;
      }
      case 4: {  // AVANZANDO AL ULTIMO PUNTO
        float dist_to_target = getDistance(path_step - 1);
        if (dist_to_target <= TOLERANCE_GOAL) {
          estado = 0;
          notification("done");
        }
        break;
      }
    }

  }


  return 0;
}
