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
#include "pgmappingcooperativo/RobotReport.h"
#include "../lib/GVD/src/data/Pos.h"
#include "../lib/utils.h"

using namespace sensor_msgs;
using namespace pgmappingcooperativo;

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
ros::Publisher moveBaseSimpleGoalPub;
ros::Publisher robotReportPub;

/// Subscribers
ros::Subscriber endSub;
ros::Subscriber goalPathSub;
ros::Subscriber poseSub;
ros::Subscriber mapSub;
ros::Subscriber mapUpdateSub;
ros::Subscriber moveBaseResultSub;
// Others
vector<Point> path;

int meterToCells;
Float metersTraveled = 0;

string endMsg = "END";

bool firstRobotPos = true; 
Pose robotPose;
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
  return path.size() == currentWaypointIndex;
}

bool isWaypointCompleted(Vector2<Float> fromPos, Vector2<Float> waypointPos){
  Float completionToleranceSquared;
  if(currentWaypointIndex < path.size() - 1){
    completionToleranceSquared = waypointCompletionToleranceSquared;
  }else{
    completionToleranceSquared = pathCompletionToleranceSquared; 
  }

  return fromPos.distanceToSquared(waypointPos) <= forcedCompletionToleranceSquared ||
         (fromPos.distanceToSquared(waypointPos) <= completionToleranceSquared &&
          unobstructedLine(toPos(fromPos, occupancyGrid.info), toPos(waypointPos, occupancyGrid.info),occupancyGrid,meterToCells));
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

void notifyStatus(char* data) {
  std_msgs::String str;
  str.data = data;
  pathResultPub.publish(str);
}

void nextWaypoint(){
  currentWaypointIndex++;
  if(!isPathOver()){
    currentWaypoint = toVector2<Float>(path[currentWaypointIndex]);

    Pose waypointPose;
    waypointPose.position = path[currentWaypointIndex] ;
   
    if(currentWaypointIndex + 1 < path.size()){
      // if currentWaypoint is not the last one set the direction to the nextWaypoint as the waypoint orientation
      Vector2<Float> nextWaypoint = toVector2<Float>(path[currentWaypointIndex+1]);
      Vector2<Float> direction = nextWaypoint - currentWaypoint;
      waypointPose.orientation = toQuaternion(direction);
    }else if (path.size() > 1){
      // if currentWaypoint is the last one set the direction from the robot to the currentWaypoint as the 
      // orientation of the waypoint
      Vector2<Float> direction = currentWaypoint - robotPos;
      waypointPose.orientation = toQuaternion(direction);
    }

    sendWaypoint(waypointPose);
  }else{
    // if there was a path assigned
    if(path.size() > 0){
      if (firstCompletedGoal || lastCompleatedGoal != currentWaypoint){
        firstCompletedGoal = false;
        lastCompleatedGoal = currentWaypoint;
        // clear path
        currentWaypointIndex = 0;
        path.clear();
        // notify path completion
        notifyStatus((char*)"SUCCEED");
      }else{
        Pose waypointPose;
        Vector2<Float> direction = currentWaypoint - robotPos;
        waypointPose.orientation = toQuaternion(direction);
        waypointPose.position = toPoint(robotPos - 3*(lastCompleatedGoal-robotPos).normalize());
        sendWaypoint(waypointPose);
        sleep(10);
        notifyStatus((char*)"RECOVERY");
      }
    }
  }
}

void trimPath(vector<Point> &path) {
  int pathStart = 0;
  for (; pathStart < path.size()-2 && isWaypointCompleted(robotPos,toVector2<Float>(path[pathStart])); pathStart++);
  path = vector<Point, allocator<Point>>(path.begin() + pathStart, path.end());
}

///////////////
// CallBacks //
///////////////

void goalPathCallback(const GoalList& msg) {
  path = msg.goals;
  if (msg.goals.empty()){
    // cancel
    currentWaypointIndex = 0;
    sendWaypoint(robotPose);
  }else{
    // set path
    trimPath(path);
    currentWaypointIndex = -1;
    nextWaypoint();
  }
}

void moveBaseResultCallback(const move_base_msgs::MoveBaseActionResult &msg) {
  switch (msg.status.status){
    case actionlib_msgs::GoalStatus::SUCCEEDED:
      // fix weird behavior of carrot planner
      if(!isPathOver()){
        Pose waypointPose;
        waypointPose.position = path[currentWaypointIndex] ;
        Vector2<Float> direction = currentWaypoint - robotPos;
        waypointPose.orientation = toQuaternion(direction);
        sendWaypoint(waypointPose);
      }
      break;
    case actionlib_msgs::GoalStatus::PREEMPTED:
    case actionlib_msgs::GoalStatus::RECALLED:
      // Do nothing
      break;
    case actionlib_msgs::GoalStatus::ABORTED:
      notifyStatus((char*)"FAILURE");
      break;
    default:
      ROS_INFO_STREAM(msg.status.text << " | STATUS ID: " << (int)msg.status.status );
      break;
  }
}

void endCallback(const std_msgs::StringConstPtr& msg) {
  bool endFlag = msg->data.compare(endMsg) == 0;

  if (!endFlag) return;

  RobotReport report;
  report.metersTraveled = metersTraveled;
  robotReportPub.publish(report);

  ROS_INFO_STREAM("Meters traveled: "<< metersTraveled);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &odom) {
  robotPose = odom->pose.pose;

  Vector2<Float> currentRobotPos = toVector2<Float>(robotPose.position);

  Float distanceOffsetSquared = currentRobotPos.distanceToSquared(robotPos);

  // check if an update is needed
  if(firstRobotPos || distanceOffsetSquared > movementDetectionThresholdSquared){
    robotPos = currentRobotPos;

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
  meterToCells = ceil(1/cellSize);

  // Initilize Publishers
  pathResultPub         = n.advertise<std_msgs::String>("path_result", 10);
  moveBaseSimpleGoalPub = n.advertise<PoseStamped>("move_base_simple/goal", 1);
  robotReportPub        = n.advertise<RobotReport>("report", 10);

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
