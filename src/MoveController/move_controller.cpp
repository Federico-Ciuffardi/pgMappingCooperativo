#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <ctime>
#include <sstream>
#include <string>
#include "../lib/GVD/src/Gvd.h"
#include "../lib/conversion.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "tf/transform_datatypes.h"
#include "pgmappingcooperativo/GoalList.h"

// Constantes

const double TOLERANCE_GOAL = 1.25;       // 0.30;
const double TOLERANCE_WAYPOINTS = 2.25;  // 0.50;

// const std::string ODOM_FRAME = "p3dx_0_tf/odom";

using namespace std;

// VARIABLES
ros::Publisher path_result_pub;
ros::Publisher path_info_pub;
ros::Publisher waypoint_pub;
ros::Subscriber goalPath_sub;
ros::Subscriber pose_sub;
// ros::Subscriber map_sub;
ros::Subscriber scan_sub;
GoalList path;
GoalList path_saved;
PoseStamped position;
PoseStamped position_old;
// nav_msgs::OccupancyGrid odometry_map;
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

// aux funcs
float getDistance(int target) {
  float dx, dy;
  dx = position.pose.position.x - path.goals[target].x;
  dy = position.pose.position.y - path.goals[target].y;
  return sqrt(pow(dx, 2) + pow(dy, 2));
}

float getDistance(Point p3d) {
  float dx, dy;
  dx = position.pose.position.x - p3d.x;
  dy = position.pose.position.y - p3d.y;
  return sqrt(pow(dx, 2) + pow(dy, 2));
}

float getNextDistance(int target) {
  float dx, dy;
  dx = path.goals[target - 1].x - path.goals[target].x;
  dy = path.goals[target - 1].y - path.goals[target].y;
  return sqrt(pow(dx, 2) + pow(dy, 2));
}

void notification(char* data) {
  std_msgs::String str;
  str.data = data;
  path_result_pub.publish(str);  // msg_succes);
  path_info_pub.publish(str);    // msg_succes);
  // idleTimer.start();
}

void handleEnd(const std_msgs::StringConstPtr& msg) {
  /* std::string str1(msg->data.c_str()); */
  /* FIN = (str1.compare(end_msg) == 0); */
  /* if (FIN) { */
    /* std_msgs::String msg_info; */
    /* std::stringstream ss9; */
    /* int sed = (clock() - startTime); */
    /* ss9 << "SUCCES: " << metros << " " << sed << " " << path.id; */
    /* msg_info.data = ss9.str(); */
    /* path_info_pub.publish(msg_info); */
    // ROS_INFO("%s :: RECORRIO %f METROS", name_space.c_str(), metros);
  /* } */
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &odom) {
  position_old = position;

  position = toPoseStamped(*odom);
}

GoalList trim_path(GoalList msg) {
  int path_start = 0;
  for (int i = 0; i < msg.goals.size(); i++) {
    float dist_to_target = getDistance(msg.goals[i]);
    if (dist_to_target <= TOLERANCE_WAYPOINTS) {
      path_start = i;
    }
  }
  msg.goals = std::vector<Point, std::allocator<Point>>(
      msg.goals.begin() + path_start, msg.goals.end());
  return msg;
}

void setPath(const GoalList& msg) {
  if (msg.goals.size() != 0) {
    path_saved = trim_path(msg);
    // ROS_INFO("MOVE CONTROLLER :: Saving new path. Long -> %lu",
    // msg.goals.size());
    pathflag = 1;
  }
}

void send_point(Point next_point) {
  // ROS_INFO("Creating path step to (%d,%d)",next_point.x,next_point.y);
  // ROS_INFO("Sending path step ---> X = %f , Y = %f , Z = %f", next_point.x,
  // next_point.y, next_point.z );
  waypoint_pub.publish(next_point);
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

// int last_safe_index = -1;
int last_unsafe_index = -1;
/*// -1 if is safe else the first obstacle index is returned
int safePath(int from_index, int lookahead){
  int safe = -1;
  Point origin_p3d  = odometry_map.info.origin.position;
  origin_p3d.x = -origin_p3d.x-1;
  origin_p3d.y = -origin_p3d.y;
  int origin_p1d = toInt(origin_p3d,0,odometry_map.info.height);
  //ROS_INFO("origin = %d",origin_p1d);
  for(int i = from_index; i< from_index + lookahead && i<path.goals.size(); i++){
    int final_p1d = toInt(path.goals[i],origin_p1d,odometry_map.info.height);
    ROS_INFO("final_p1d = %d, is -> %d ",final_p1d,odometry_map.data[final_p1d]);
    if(odometry_map.data[final_p1d] == 100){
      return i;
    }
  }
  return safe;
}*/

double getAngle(Quaternion q) {
  tf::Quaternion quat_tf;

  tf::quaternionMsgToTF(q, quat_tf);
  tf::Matrix3x3 m(quat_tf);

  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

double normalize(double angle) {
  angle = fmod(angle + M_PI, M_PI * 2);
  if (angle < 0)
    angle += M_PI * 2;
  return angle - M_PI;
}

bool isSafe(float target_angle, float safe_distance = 2.5) {
  Point current_pos = position.pose.position;

  double robot_angle = getAngle(position.pose.orientation);
  // ROS_INFO("angulo del robotitio %f, dir objetivo (%f, %f), angulo del objetivo
  // %f",robot_angle,target_pos.x,target_pos.y ,target_angle);
  double target_angle_adjusted = normalize(target_angle - robot_angle);

  double increment = laserScan.angle_increment;
  int laser_index = (target_angle_adjusted - laserScan.angle_min) / increment;
  // ROS_INFO("angulo inicial = %f",laserScan.angle_min);

  // ROS_INFO("%s :: pos objetivo (%f,%f), angulo del objetivo %f, angulo del robot
  // %f",name_space.c_str(), target_pos.x,target_pos.y, target_angle,robot_angle);

  // ROS_INFO("%s :: angulo del objetivo ajustado %f, id %d, representa %f, valor laser %f,
  // umbral %f",name_space.c_str(), target_angle_adjusted, laser_index,
  // laser_index*increment+laserScan.angle_min, laserScan.ranges[laser_index],safe_distance);

  return laserScan.ranges[laser_index] > safe_distance;
}

float getTargetAngle(int index) {
  Point current_pos = position.pose.position;
  Point target_pos = path.goals[index];
  target_pos.x -= current_pos.x;
  target_pos.y -= current_pos.y;
  return atan2(target_pos.y, target_pos.x);
}

/*bool isSafe(int index){
  Point current_pos = position.pose.position;
  Point target_pos = path.goals[index];
  target_pos.x -= current_pos.x;
  target_pos.y -= current_pos.y;

  if(target_pos.x<0.5 && target_pos.y < 0.5 ){
    return true;
  }

  double robot_angle = getAngle(position.pose.orientation);
  double target_angle = atan2(target_pos.y,target_pos.x);
  //ROS_INFO("angulo del robotitio %f, dir objetivo (%f, %f), angulo del objetivo
%f",robot_angle,target_pos.x,target_pos.y ,target_angle); double target_angle_adjusted=
normalize(target_angle-robot_angle);

  double increment = laserScan.angle_increment;
  int laser_index = (target_angle_adjusted - laserScan.angle_min)/increment;
  //ROS_INFO("angulo inicial = %f",laserScan.angle_min);

  //ROS_INFO("%s :: pos objetivo (%f,%f), angulo del objetivo %f, angulo del robot
%f",name_space.c_str(), target_pos.x,target_pos.y, target_angle,robot_angle);

  double safe_distance =  1 + getDistance(index);
  //ROS_INFO("%s :: angulo del objetivo ajustado %f, id %d, representa %f, valor laser %f,
umbral %f",name_space.c_str(), target_angle_adjusted, laser_index,
laser_index*increment+laserScan.angle_min, laserScan.ranges[laser_index],safe_distance);

  return laserScan.ranges[laser_index] > safe_distance;
}*/

int main(int argc, char** argv) {
  // ROS_INFO("Initializing node");
  startTime = clock();
  // Initializing ros
  ros::init(argc, argv, "move_controller");

  bool primera = true;
  char buffer_ns[20];
  ros::NodeHandle n;
  name_space = n.getNamespace().substr(1, 5);

  ROS_DEBUG("Initializing node %s", name_space.c_str());
  ROS_DEBUG("Initializing node %s", name_space.c_str());

  goalPath_sub = n.subscribe("goalPath", /*1*/ 10, setPath);
  pose_sub = n.subscribe("odom", 1, odomCallback);
  // map_sub = n.subscribe<nav_msgs::OccupancyGrid>("map", 1, saveMap);
  scan_sub = n.subscribe<sensor_msgs::LaserScan>("scan", 1, saveScan);
  end_sub = n.subscribe("/end", 1, handleEnd);
  path_result_pub = n.advertise<std_msgs::String>("path_result", 10);
  path_info_pub = n.advertise<std_msgs::String>("path_info", 10);
  waypoint_pub = n.advertise<Point>("waypoint", 1);

  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Initializing morse comunication
  /* ROS_INFO("Starting TCP conection"); */
  /* ROS_INFO("End TCP conection"); */
  float last_distancie = 0;
  ros::Rate loop_rate(10);
  int secondsPassed;
  // while(path_result_pub.getNumSubscribers() == 0){};
  while (ros::ok()) {
    if (pathflag == 1) {
      /* std_msgs::String msg_info; */
      /* std::stringstream ss9; */
      /* int sed = (clock() - startTime); */
      /* ss9 << "SUCCES: " << metros << " " << sed << " " << path.id; */
      /* msg_info.data = ss9.str(); */
      /* path_info_pub.publish(msg_info); */
      start = true;
      cont = 0;
      path_step = 0;
      pathflag = 0;
      estado = 0;
      // last_safe_index = -1;
      last_unsafe_index = -1;
      path = path_saved;
    }

    loop_rate.sleep();

    cout<<"state: "<<estado<<endl;
    switch (estado) {
      case 0: {
        if (path_step != path.goals.size()) {
          estado = 1;
        }
        break;
      }
      case 1: {  // AVANZANDO A UN PUNTO
        send_point(path.goals[path_step]);
        angleToTarget = getTargetAngle(path_step);
        cout<<"got angle"<<endl;
        path_step++;
        if (path_step != path.goals.size()) {
          metros = metros + getNextDistance(path_step);
        }
        cout<<"distance updated"<<endl;
        estado = 2;
        break;
      }
    }

    if (estado < 2) {
      continue;
    }

    /* if (!isSafe(angleToTarget)) { */
    /*   if (last_unsafe_index != path_step) { */
    /*     last_unsafe_index = path_step; */
    /*     ROS_INFO("OBSTACLE"); */
    /*     notification((char*)"OBSTACLE"); */
    /*     send_point(position.pose.position); */
    /*   } */
    /* } else { */
    /*   if (last_unsafe_index == path_step) { */
    /*     send_point(path.goals[path_step]); */
    /*   } */
    /* } */
    /* cout<<"safety check done"<<endl; */


    switch (estado) {
      case 2: {
        float dist_to_target = getDistance(path_step - 1);
        if (dist_to_target <= TOLERANCE_WAYPOINTS) {
          cout<<"Arriving"<<endl;
          if (path_step == path.goals.size()) {  // if wait_last_point
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
          notification((char*)"SUCCESS");
        }
        break;
      }
    }
  }

  return 0;
}
