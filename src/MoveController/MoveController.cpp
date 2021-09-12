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

////////////////
// Parameters //
////////////////

/* Float TOLERANCE_GOAL = 2.25;// 1.25;       // 0.30; */
Float waypointCompletionToleranceSquared = 50*50;  // 0.50;
Float pathCompletionToleranceSquared = 2*2;  // 0.50;
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

bool unobstructedLine(Vector2<Float> v1, Vector2<Float> v2, OccupancyGrid &occupancyGrid){
  Pos p1 = toPos(v1, occupancyGrid.info);
  Pos p2 = toPos(v2, occupancyGrid.info);

  int threshold = 50;
  for (Pos p : discretizeLine(p1,p2)){
    if(p == p2 || p == p1) continue;

    int clearence = meterToCells;
    for(int x = -clearence; x < clearence; x++){
      for(int y = -clearence; y < clearence; y++){
        Pos pN = p + Pos(x,y);
        int pNInd = toInt(pN,occupancyGrid.info.width);
        if(pNInd < occupancyGrid.data.size() && occupancyGrid.data[pNInd] >= threshold){
          return false;
        }
      }
    }
  }
  return true;
}

bool isPathOver(){
  return path.goals.size() == currentWaypointIndex;
}

bool isWaypointCompleted(Vector2<Float> fromPos, Vector2<Float> waypointPos){

  Float completionTolerance;
  if(currentWaypointIndex < path.goals.size() - 1){
    completionTolerance = waypointCompletionToleranceSquared;
  }else{
    completionTolerance = pathCompletionToleranceSquared; 
  }

  return fromPos.distanceToSquared(waypointPos) <= 1 ||
         fromPos.distanceToSquared(waypointPos) <= completionTolerance && unobstructedLine(fromPos,waypointPos,occupancyGrid);
}

///////////////
// Functions //
///////////////

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

void nextWaypoint(){
  currentWaypointIndex++;
  if(!isPathOver()){
    currentWaypoint = toVector2<Float>(path.goals[currentWaypointIndex]);
    send_point(path.goals[currentWaypointIndex]);
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
    send_point(path.goals[currentWaypointIndex]);
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
