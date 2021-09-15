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
#include "geometry_msgs/Quaternion.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "nav_msgs/OccupancyGrid.h"
#include "actionlib_msgs/GoalStatus.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "tf/transform_datatypes.h"
#include "pgmappingcooperativo/GoalList.h"
#include "../lib/GVD/src/data/Pos.h"
#include "../lib/utils.h"

using namespace sensor_msgs;

#define squared(x) x*x

////////////////
// Parameters //
////////////////

Float forcedCompletionToleranceSquared   = squared(1.5);
Float pathCompletionToleranceSquared     = squared(2);
Float waypointCompletionToleranceSquared = squared(50);
Float movementDetectionThresholdSquared  = squared(0.01);

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
ros::Subscriber mapSub;
ros::Subscriber mapUpdateSub;
ros::Subscriber moveBaseResultSub;
// Others
GoalList path;
PoseStamped position;

Float cellDiagonal;
int meterToCells;
Float metersTraveled = 0;

int pathflag = 0;
int path_step = 0;
int estado = 0;
bool FIN = false;
string endMsg("END");

bool firstRobotPos = true; 
Vector2<Float> robotPos;

int currentWaypointIndex = 0;
Vector2<Float> currentWaypoint;

bool firstCompletedGoal = true; 
Vector2<Float> lastCompleatedGoal;

OccupancyGrid occupancyGrid;

///////////////////
// Aux Functions //
///////////////////

bool isPathOver(){
  return path.goals.size() == currentWaypointIndex;
}

bool isWaypointCompleted(Vector2<Float> fromPos, Vector2<Float> waypointPos){

  Float completionToleranceSquared;
  if(currentWaypointIndex < path.goals.size() - 1){
    completionToleranceSquared = waypointCompletionToleranceSquared;
  }else{
    completionToleranceSquared = pathCompletionToleranceSquared; 
  }

  return fromPos.distanceToSquared(waypointPos) <= forcedCompletionToleranceSquared ||
         fromPos.distanceToSquared(waypointPos) <= completionToleranceSquared &&
           unobstructedLine(toPos(fromPos, occupancyGrid.info), toPos(waypointPos, occupancyGrid.info),occupancyGrid, meterToCells);
}

///////////////
// Functions //
///////////////

int moveBaseSeq = 0;
void sendWaypoint(Pose pose) {
  PoseStamped poseStamped;
  poseStamped.header.frame_id = "map";
  poseStamped.header.stamp = ros::Time::now();
  poseStamped.header.seq = moveBaseSeq;
  poseStamped.pose = pose;

  moveBaseSimpleGoalPub.publish(poseStamped);

  moveBaseSeq++;
}

void sendWaypoint(Point point) {
  Pose waypointPose;
  waypointPose.position = path.goals[currentWaypointIndex] ;
  waypointPose.orientation.w = 1;

  sendWaypoint(waypointPose);
}

void notifyStatus(char* data) {
  std_msgs::String str;
  str.data = data;
  pathResultPub.publish(str);
  pathInfoPub.publish(str);
}

void nextWaypoint(){
  currentWaypointIndex++;
  if(!isPathOver()){
    currentWaypoint = toVector2<Float>(path.goals[currentWaypointIndex]);

    Pose waypointPose;
    waypointPose.position = path.goals[currentWaypointIndex] ;
   
    if(currentWaypointIndex + 1 < path.goals.size()){
      // if currentWaypoint is not the last one set the direction to the nextWaypoint as the waypoint orientation
      Vector2<Float> nextWaypoint = toVector2<Float>(path.goals[currentWaypointIndex+1]);
      Vector2<Float> direction = nextWaypoint - currentWaypoint;
      waypointPose.orientation = toQuaternion(direction);
    }else if (path.goals.size() > 1){
      // if currentWaypoint is the last one set the direction from the robot to the currentWaypoint as the 
      // orientation of the waypoint
      Vector2<Float> direction = currentWaypoint - robotPos;
      waypointPose.orientation = toQuaternion(direction);
    }

    sendWaypoint(waypointPose);
  }else{
    // if there was a path assigned
    if(path.goals.size() > 0 && (firstCompletedGoal || lastCompleatedGoal != currentWaypoint) ){
      firstCompletedGoal = false;
      lastCompleatedGoal = currentWaypoint;
      // clear path
      currentWaypointIndex = 0;
      path.goals.clear();
      // notify path completion
      notifyStatus((char*)"SUCCESS");
    }
  }
}

GoalList trimPath(GoalList msg) {
  int pathStart = 0;
  for (; pathStart < msg.goals.size() && !isWaypointCompleted(robotPos,toVector2<Float>(msg.goals[pathStart])); pathStart++);
  msg.goals = vector<Point, allocator<Point>>( msg.goals.begin() + pathStart, msg.goals.end());
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

void moveBaseResultCallback(const move_base_msgs::MoveBaseActionResult &msg) {
  // fix weird behavior of carrot planner
  if(msg.status.status == actionlib_msgs::GoalStatus::SUCCEEDED && !isPathOver()){
    Pose waypointPose;
    waypointPose.position = path.goals[currentWaypointIndex] ;
    Vector2<Float> direction = currentWaypoint - robotPos;
    waypointPose.orientation = toQuaternion(direction);
    sendWaypoint(waypointPose);
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

  // check if an update is needed
  if(firstRobotPos || distanceOffsetSquared > movementDetectionThresholdSquared){
    robotPos = toVector2<Float>(odom->pose.pose.position);

    // update distance traveled
    metersTraveled += sqrt(distanceOffsetSquared);

    // update path completion
    while(isWaypointCompleted(robotPos,currentWaypoint) && !isPathOver()){
      nextWaypoint();
    }
  }
}

void mapCallBack(const OccupancyGridConstPtr& msg) {
  occupancyGrid = *msg;
}

void mapUpdateCallBack(const OccupancyGridUpdateConstPtr& msg) {
  updateOccupancyGrid(occupancyGrid, *msg);
}

int main(int argc, char** argv) {
  // Init node
  ros::init(argc, argv, "move_controller");
  ros::NodeHandle n;

  // Load params
  float cellSize;
  FAIL_IFN(n.param<float>   ("/cell_size", cellSize, cellSize));
  cellDiagonal = cellSize*sqrt(2);
  meterToCells = round(1/cellSize);

  // Initilize Publishers
  pathResultPub         = n.advertise<std_msgs::String>("path_result", 10);
  pathInfoPub           = n.advertise<std_msgs::String>("path_info", 10);
  waypointPub           = n.advertise<Point>("waypoint", 1);
  moveBaseSimpleGoalPub = n.advertise<PoseStamped>("move_base_simple/goal", 1);

  // Initilize Subscribers
  moveBaseResultSub = n.subscribe("move_base/result", 1, moveBaseResultCallback);
  goalPathSub       = n.subscribe("goalPath", 1, goalPathCallback);
  poseSub           = n.subscribe("odom", 1, odomCallback);
  endSub            = n.subscribe("/end", 1, endCallback);
  mapSub            = n.subscribe<OccupancyGrid>("/map", 1, mapCallBack);
  mapUpdateSub      = n.subscribe<OccupancyGridUpdate>("/map_update", 1, mapUpdateCallBack);

  // spin
  ROS_INFO_STREAM("Initilized");
  ros::spin();

  return 0;
}
