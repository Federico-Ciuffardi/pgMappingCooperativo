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
#include "../lib/GVD/src/data/Pos.h"

using namespace sensor_msgs;

////////////////
// Parameters //
////////////////

/* Float TOLERANCE_GOAL = 2.25;// 1.25;       // 0.30; */
Float waypointCompletionToleranceSquared = 2.25*2.25;  // 0.50;
Float movementDetectionThresholdSquared = 0.01*0.01;

///////////////
// Variables //
///////////////

// Topics
/// Publishers
ros::Publisher pathResultPub;
ros::Publisher pathInfoPub;
ros::Publisher waypointPub;
ros::Publisher moveBaseSimpleGoalPub;

/// Subscribers
ros::Subscriber endSub;
ros::Subscriber goalPathSub;
ros::Subscriber poseSub;

// Others
GoalList path;
PoseStamped position;

Float metersTraveled = 0;

int pathflag = 0;
int path_step = 0;
int estado = 0;
bool FIN = false;
string endMsg("END");

Vector2<Float> robotPos;
bool firstRobotPos = true; 
int currentWaypointIndex = 0;
Vector2<Float> currentWaypoint;

///////////////////
// Aux Functions //
///////////////////

int moveBaseSeq = 0;
void send_point(Point next_point) {
  PoseStamped poseStamped;
  poseStamped.header.frame_id = "map";
  poseStamped.header.stamp = ros::Time::now();
  poseStamped.header.seq = moveBaseSeq;

  moveBaseSeq++;

  poseStamped.pose.position = next_point;
  poseStamped.pose.orientation.w = 1;
  
  moveBaseSimpleGoalPub.publish(poseStamped);
}

void notifyStatus(char* data) {
  std_msgs::String str;
  str.data = data;
  pathResultPub.publish(str);
  pathInfoPub.publish(str);
}

bool isPathOver(){
  return path.goals.size() == currentWaypointIndex;
}

void nextWaypoint(){
  currentWaypointIndex++;
  if(!isPathOver()){
    currentWaypoint = toVector2<Float>(path.goals[currentWaypointIndex]);
    send_point(path.goals[currentWaypointIndex]);
  }else{
    // if there was a path assigned
    if(path.goals.size() > 0){
      // clear path
      currentWaypointIndex = 0;
      path.goals.clear();
      // notify path completion
      notifyStatus((char*)"SUCCESS");
    }
  }
}

GoalList trimPath(GoalList msg) {
  int path_start = 0;
  for (int i = 0; i < msg.goals.size(); i++) {
    Vector2<Float> waypoint = toVector2<Float>(msg.goals[i]);
    
    if (robotPos.distanceToSquared(waypoint) <= waypointCompletionToleranceSquared) {
      path_start = i;
    }
  }
  msg.goals = vector<Point, allocator<Point>>( msg.goals.begin() + path_start, msg.goals.end());
  return msg;
}

///////////////
// CallBacks //
///////////////

void goalPathCallback(const GoalList& msg) {
  if (msg.goals.size() > 0) {
    path = trimPath(msg);
    currentWaypointIndex = -1;
    nextWaypoint();
  }
}

void endCallback(const std_msgs::StringConstPtr& msg) {
  bool endFlag = msg->data.compare(endMsg) == 0;

  if (!endFlag) return;

  ROS_INFO_STREAM("Meters traveled: "<< metersTraveled);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &odom) {

  Vector2<Float> currentRobotPos = toVector2<Float>(odom->pose.pose.position);

  Float distanceOffsetSquared = currentRobotPos.distanceToSquared(robotPos);

  if(firstRobotPos || distanceOffsetSquared > movementDetectionThresholdSquared){
    robotPos = toVector2<Float>(odom->pose.pose.position);

    metersTraveled += sqrt(distanceOffsetSquared);

    while(robotPos.distanceToSquared(currentWaypoint) < waypointCompletionToleranceSquared && !isPathOver()){
      nextWaypoint();
    }
  }
}

int main(int argc, char** argv) {
  // Init node
  ros::init(argc, argv, "move_controller");
  ros::NodeHandle n;

  // Load params
  /* float cell_size; */
  /* FAIL_IFN(n.param<float>   ("/cell_size", cell_size, cell_size)); */
  /* TOLERANCE_GOAL /= cell_size; */
  /* waypointCompletionToleranceSquared /= cell_size; */

  // Initilize Publishers
  pathResultPub         = n.advertise<std_msgs::String>("path_result", 10);
  pathInfoPub           = n.advertise<std_msgs::String>("path_info", 10);
  waypointPub           = n.advertise<Point>("waypoint", 1);
  moveBaseSimpleGoalPub = n.advertise<PoseStamped>("move_base_simple/goal", 1);

  // Initilize Subscribers
  goalPathSub = n.subscribe("goalPath", /*1*/ 10, goalPathCallback);
  poseSub     = n.subscribe("odom", 1, odomCallback);
  endSub     = n.subscribe("/end", 1, endCallback);

  // spin
  ROS_INFO_STREAM("Initilized");
  ros::spin();

  return 0;
}
