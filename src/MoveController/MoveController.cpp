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
#include "ros/time.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "tf/transform_datatypes.h"
#include "pgmappingcooperativo/GoalList.h"
#include "pgmappingcooperativo/RobotReport.h"
#include "../lib/GVD/src/data/Pos.h"
#include "../lib/utils.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

using namespace sensor_msgs;
using namespace pgmappingcooperativo;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

#define squared(x) x*x

////////////////
// Parameters //
////////////////

// meters
Float forcedCompletionToleranceSquared  = squared(1.5);
Float pathCompletionToleranceSquared    = squared(2.5);//squared(2);
Float goalCompletionToleranceSquared    = squared(50);
Float movementDetectionThresholdSquared = squared(0.000001); // 1 micrometer
Float stopTresholdSquared               = squared(1); // squared(1.25); // squared(1);
Float progressToleranceSquared          = squared(0.5);

Float baseRecoveryDistance = 1.5;

//secs
Float recoveryBehaviorTimeTolerance = 7.5;

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
ros::Subscriber laserScansub;

/// Action client
MoveBaseClient* ac;

// Others

vector<Point> path;

float robotSpeed;
int meterToCells;
Float metersTraveled = 0;
Float lastMetersTraveled = 0;

string endMsg = "END";

bool firstRobotPos = true; 
Pose robotPose;
Vector2<Float> robotPos;

int currentGoalPosIndex = 0;
Vector2<Float> currentGoalPos;
Point currentGoalPose;

bool firstCompletedGoal = true; 
Vector2<Float> lastCompletedGoalPos;

int timesReCompleted = 0; 

bool firstGoal = true; 
ros::Time lastProgressTime;
Float lastDistanceSquaredToGoal;
Vector2<Float> lastGoalPos;

int noProgressCounter = 0;

OccupancyGrid occupancyGrid;

///////////////////
// Aux Functions //
///////////////////

bool isPathOver(){
  return path.size() <= currentGoalPosIndex;
}

bool isGoalCompleted(Vector2<Float> fromPos, Vector2<Float> goalPos){
  Float completionToleranceSquared;
  if(currentGoalPosIndex < path.size() - 1){
    completionToleranceSquared = goalCompletionToleranceSquared;
  }else{
    completionToleranceSquared = pathCompletionToleranceSquared; 
  }

  return fromPos.distanceToSquared(goalPos) <= forcedCompletionToleranceSquared ||
         (fromPos.distanceToSquared(goalPos) <= completionToleranceSquared &&
          unobstructedLine(toPos(fromPos, occupancyGrid.info), toPos(goalPos, occupancyGrid.info),occupancyGrid,meterToCells));
}

///////////////
// Functions //
///////////////

int moveBaseSeq = 0;
void sendGoal(Pose pose) {
  PoseStamped poseStamped;
  poseStamped.header.frame_id = "map";
  poseStamped.header.stamp = ros::Time::now();
  poseStamped.header.seq = moveBaseSeq;
  poseStamped.pose = pose;

  moveBaseSimpleGoalPub.publish(poseStamped);

  moveBaseSeq++;
}

// to currentGoalPos
void sendGoal(){
    Pose goalPose;
    goalPose.position = currentGoalPose;
   
    if(currentGoalPosIndex + 1 < path.size()){
      // if currentGoalPos is not the last one set the direction to the nextGoal as the goal orientation
      Vector2<Float> nextGoal = toVector2<Float>(path[currentGoalPosIndex+1]);
      Vector2<Float> direction = nextGoal - currentGoalPos;
      goalPose.orientation = toQuaternion(direction);
    }else if (path.size() > 1){
      // if currentGoalPos is the last one set the direction from the robot to the currentGoalPos as the 
      // orientation of the goal
      Vector2<Float> direction = currentGoalPos - robotPos;
      goalPose.orientation = toQuaternion(direction);
    }

    sendGoal(goalPose);
}

void notifyStatus(char* data) {
  std_msgs::String str;
  str.data = data;
  pathResultPub.publish(str);
}

void recoveryBehavior(Vector2<Float> failedPos, float minDistToFailed){
  Vector2<Float> robot2failedOffset = failedPos-robotPos;
  Float robot2recoveryOffsetLenght = max(1.0f , minDistToFailed - robot2failedOffset.length());
  Vector2<Float> robot2recoveryOffset = -robot2recoveryOffsetLenght*robot2failedOffset.normalize();

  Pose goalPose;
  goalPose.orientation = toQuaternion(-robot2recoveryOffset);
  goalPose.position = toPoint(robotPos + robot2recoveryOffset);

  sendGoal(goalPose);

  sleep(minDistToFailed/(robotSpeed*0.75));
  notifyStatus((char*)"RECOVERY");
  sendGoal();
}

void nextGoal(){
  currentGoalPosIndex++;
  if(!isPathOver()){
    currentGoalPos  = toVector2<Float>(path[currentGoalPosIndex]);
    currentGoalPose = path[currentGoalPosIndex];
    sendGoal();

    if(firstGoal || currentGoalPos != lastGoalPos){
      firstGoal = false;
      lastGoalPos = currentGoalPos;

      noProgressCounter = 0;
      lastProgressTime = ros::Time::now();
      lastDistanceSquaredToGoal = robotPos.distanceToSquared(currentGoalPos);
      /* ROS_INFO_STREAM("Progress to goal by getting a new one "<< recoveryBehaviorTimeTolerance - (ros::Time::now() - lastProgressTime).toSec()<<"s"); */
    }
  }else{
    // if there was a path assigned the path was completed
    if(path.size() > 0){ 
      // process path completion
      if (firstCompletedGoal || lastCompletedGoalPos != currentGoalPos){
        firstCompletedGoal = false;
        lastCompletedGoalPos = currentGoalPos;
        timesReCompleted = 0;
        // notify path completion
        notifyStatus((char*)"SUCCEED");
      }else {
        timesReCompleted++;
        if(timesReCompleted >= 2){
          ROS_INFO_STREAM("Too many reassignations to complated goal, starting RECOVERY");
          recoveryBehavior(lastCompletedGoalPos, baseRecoveryDistance + (timesReCompleted-2));
        }else{
          notifyStatus((char*)"SUCCEED_AGAIN");
        }
      }

      // clear path
      currentGoalPosIndex = 0;
      path.clear();
    }
  }
}

// Returns the index of the last completed goal
// if all the goals in the path are completed returns the penultimate goal
// returns -1 if no goal is completed or path.size == 1
// Precondition: path.size() > 0
int lastCompletedGoalIndex(vector<Point> &path) { 
  int goalIndex = path.size()-2;
  for (; goalIndex >= 0 && !isGoalCompleted(robotPos,toVector2<Float>(path[goalIndex])); goalIndex--);
  /* path = vector<Point, allocator<Point>>(path.begin() + pathStart, path.end()); */
  return goalIndex;
}

///////////////
// CallBacks //
///////////////

Float minDist = INF;
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &scan) {
  minDist = INF;
  for(Float dist : scan->ranges){
    minDist = min(minDist, dist);
  }
}

void goalPathCallback(const GoalList& msg) {
  if (msg.goals.empty()){
    // stop
    currentGoalPosIndex = 0;
    path.clear();
    ac->cancelAllGoals();
  }else{
    // set path
    path = msg.goals;
    currentGoalPosIndex = lastCompletedGoalIndex(path); // efective completion on nextGoal()
    do{
      nextGoal();
    }while(!isPathOver() && isGoalCompleted(robotPos,currentGoalPos) ); // the path could already be completed so another nextGoal() could be needed
  }
}

void moveBaseResultCallback(const move_base_msgs::MoveBaseActionResult &msg) {
  switch (msg.status.status){
    case actionlib_msgs::GoalStatus::SUCCEEDED:
      // fix weird behavior of carrot planner
      if(!isPathOver()){
        Pose goalPose;
        goalPose.position = currentGoalPose;
        Vector2<Float> direction = currentGoalPos - robotPos;
        goalPose.orientation = toQuaternion(direction);
        sendGoal(goalPose);
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

  if(firstRobotPos){
    firstRobotPos = false;
    robotPos = currentRobotPos;
    return;
  }

  Float robotPosDistanceOffsetSquared = currentRobotPos.distanceToSquared(robotPos);
  // check if an update is needed
  if(robotPosDistanceOffsetSquared >= movementDetectionThresholdSquared){
    robotPos = currentRobotPos;

    // update distance traveled
    metersTraveled += sqrt(robotPosDistanceOffsetSquared);

    // update path completion
    while(!isPathOver() && isGoalCompleted(robotPos,currentGoalPos)){
      nextGoal();
    }

    if(isPathOver() && minDist*minDist <= stopTresholdSquared){
      // stop
      ac->cancelAllGoals();
    }
  }

  if ( !isPathOver() ){
    Float currentDistanceSquaredToGoal = robotPos.distanceToSquared(currentGoalPos);
    if(lastDistanceSquaredToGoal > currentDistanceSquaredToGoal + progressToleranceSquared){
      /* ROS_INFO_STREAM("Progress to goal by getting closer "<< recoveryBehaviorTimeTolerance - (ros::Time::now() - lastProgressTime).toSec()<<"s"); */
      noProgressCounter = 0;
      lastProgressTime = ros::Time::now();
      lastDistanceSquaredToGoal = currentDistanceSquaredToGoal;
    }else if ((ros::Time::now() - lastProgressTime).toSec() > recoveryBehaviorTimeTolerance){
      noProgressCounter++;
      ROS_INFO_STREAM("No progress to goal, starting RECOVERY");
      recoveryBehavior(currentGoalPos, baseRecoveryDistance + (noProgressCounter-1));
      lastProgressTime = ros::Time::now(); // try again in recoveryBehaviorTimeTolerance seconds
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
  FAIL_IFN(n.param<float> ("/robot_speed", robotSpeed, 0));

  float cellSize;
  FAIL_IFN(n.param<float>   ("/cell_size", cellSize, cellSize));
  meterToCells = ceil(1/cellSize);

  // Initilize Publishers
  pathResultPub         = n.advertise<std_msgs::String>("path_result", 1);
  moveBaseSimpleGoalPub = n.advertise<PoseStamped>("move_base_simple/goal", 1);
  robotReportPub        = n.advertise<RobotReport>("report", 1);

  // Initilize Subscribers
  moveBaseResultSub = n.subscribe("move_base/result", 1, moveBaseResultCallback);
  goalPathSub       = n.subscribe("goalPath", 1, goalPathCallback);
  poseSub           = n.subscribe("odom", 1, odomCallback);
  endSub            = n.subscribe("/end", 1, endCallback);
  mapSub            = n.subscribe<OccupancyGrid>("/map", 1, mapCallBack);
  mapUpdateSub      = n.subscribe<OccupancyGridUpdate>("/map_update", 100000, mapUpdateCallBack);
  laserScansub = n.subscribe("laser/scan", 1, lidarCallback);


  // Initilize action client
  ac = new MoveBaseClient("move_base");
  //wait for the action server to come up
  while(!ac->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // spin
  ROS_INFO_STREAM("Initilized");
  ros::spin();

  delete ac;

  return 0;
}
