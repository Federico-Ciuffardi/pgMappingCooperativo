#include <ros/ros.h>
#include <cstdlib>
#include <ctime>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "Robot.h"
#include "ros/init.h"
#include "ros/this_node.h"
#include "ros/time.h"
#include "std_msgs/Float64.h"

////////////////
// Parameters //
////////////////

/////////////////
// Rviz params //
/////////////////

/// Markers used to represent cells
unsigned int cellMarkerType = RvizHelper::CUBE_LIST;

/// Markers dimensions
float cubeHeight = 0.02;

/// Layers (planes orthogonal to the z axis)

//// Separation between each layer
float layerSeparation = cubeHeight;

//// Assign markers to layers
float pathLineZ = 12*layerSeparation;
float rtPosZ    = 11*layerSeparation;
float posZ      = 10*layerSeparation;

///////////////
// Variables //
///////////////

// Topics
/// Subscribers
ros::Subscriber odomSub;

ros::Subscriber auctionSub;
ros::Subscriber assignmentSub;

ros::Subscriber pathResultSub;
ros::Subscriber endSub;

ros::Subscriber mapSub;
ros::Subscriber mapUpdateSub;

/// Publishers
ros::Publisher endPub;
ros::Publisher bidPub;

ros::Publisher requestObjetivePub;
ros::Publisher goalPathPub;
ros::Publisher pathMarkerPub;
ros::Publisher posMarkerPub;

// others
Robot robot;
RvizHelper rvizHelper;

string endMsg("END");

////////////////
// Rviz marks //
////////////////

void setPathRvizMarks(GoalList &path, mapInfoType mapInfo) {
  if(path.goals.empty()){
    rvizHelper.deleteMark(robot.name + "_path", 0);
  }else{
    float cellSize = mapInfo.resolution;

    // draw path to objective
    rvizHelper.topic = &pathMarkerPub;

    rvizHelper.color    = MAGENTA;
    rvizHelper.type     = RvizHelper::LINE_STRIP;
    rvizHelper.scale    = makeVector3(cellSize*0.15, cellSize, 1);
    rvizHelper.position = makeVector3(0,0,pathLineZ);
    rvizHelper.mark(path.goals, robot.name + "_path");
  }
}

void setPositionRvizMarks(Point &position, mapInfoType mapInfo) {
  float cellSize = mapInfo.resolution;

  // Mark Current position on rviz
  rvizHelper.topic = &pathMarkerPub;

  RvizHelper::MarkerPoints posMarkerPoint;
  posMarkerPoint.push_back(toMarkerPoint(toPos(position, mapInfo), mapInfo));

  rvizHelper.color    = makeColorRGBA(0.75);
  rvizHelper.type     = cellMarkerType;
  rvizHelper.scale    = makeVector3(cellSize*0.5); rvizHelper.scale.z  = cubeHeight;
  rvizHelper.position = makeVector3(0,0,posZ);
  rvizHelper.mark(posMarkerPoint, robot.name + "_pos");
}

Pos lastRtPos = NULL_POS;
void setRealTimePositionRvizMarks(Pose pose) {
  Pos currRtPos;
  currRtPos.x = pose.position.x ;
  currRtPos.y = pose.position.y ;
  if(currRtPos.distanceToSquared(lastRtPos) < 1) return; // pos not different enough
  lastRtPos = currRtPos;

  // Mark Current position on rviz
  rvizHelper.topic = &posMarkerPub;

  RvizHelper::MarkerPoints posMarkerPoint;
  Point marker;
  posMarkerPoint.push_back(marker);
  marker.x=1;
  posMarkerPoint.push_back(marker);

  rvizHelper.color    = RED;
  rvizHelper.type     = RvizHelper::ARROW;

  rvizHelper.scale    = makeVector3(0.25,0.5,0.4);

  rvizHelper.position = toVector3(pose.position); 
  rvizHelper.position.z = rtPosZ;

  rvizHelper.orientation = pose.orientation;

  rvizHelper.mark(posMarkerPoint, robot.name + "_rt_pos");

  // reset rvizHelper state
  rvizHelper.orientation = Quaternion(); rvizHelper.orientation.w = 1;
  rvizHelper.position = Vector3();
}

///////////////
// CallBacks //
///////////////

void odomCallback(const nav_msgs::Odometry::ConstPtr &odom) {
  // Store current position
  robot.position    = odom->pose.pose.position;
  robot.orientation = odom->pose.pose.orientation;

  // Mark Current position on rviz
  rvizHelper.topic = &pathMarkerPub;

  setRealTimePositionRvizMarks(odom->pose.pose);
  setPositionRvizMarks(robot.position, robot.occupancyGrid.info);
}

void pathResultCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO_STREAM("Result "<<msg->data<<", requesting objective");
  std_msgs::Float64 msgRequest;
  msgRequest.data = ros::Time::now().toSec();

  requestObjetivePub.publish(msgRequest);

  if(msg->data != "RECOVERY"){
    // Delete the path mark
    rvizHelper.topic = &pathMarkerPub;
    rvizHelper.deleteMark(robot.name + "_path", 0);
    rvizHelper.deleteMark(robot.name + "_pos", 0);
  }
}

void auctionCallBack(const AuctionConstPtr& msg) {
  // Validate
  if (robot.lastAuctionId >= msg->id) {
    ROS_INFO_STREAM("An old auction arrived: last_id >= id | "<<robot.lastAuctionId<<" >= "<<msg->id);
    return;
  }

  // Process
  ROS_INFO("Auction message arrived");
  robot.lastAuctionId = msg->id;

  /* ros::Time bidStart = ros::Time::now(); */
  ROS_INFO("Calculate bids");
  Bid bid;
  if(!msg->frontiers.empty()){
    bid = robot.getBid(*msg);
  }

  ROS_INFO("Publish bids");
  bid.id = msg->id;
  bidPub.publish(bid);
}

void assignmentCallback(const AssignmentConstPtr& msg) {
  // Validate
  if (robot.lastAssignmentId >= msg->id) {
    ROS_INFO_STREAM("An old assingment arrived: last_id >= id | "<<robot.lastAssignmentId<<" >= "<<msg->id);
    return;
  }

  // Process
  ROS_INFO("A assingment arived");
  robot.lastAssignmentId = msg->id;

  // Set path info
  GoalList goalList;

  if(msg->assigned){
    goalList = robot.getPathTo(toPos(msg->frontier));
  }else{
    goalList.id = robot.lastAssignmentId;
  }

  // Publish the path
  ROS_INFO("Publish the path that leads to the assingment");
  goalPathPub.publish(goalList);
    
  // Mark path info
  setPathRvizMarks(goalList,  robot.occupancyGrid.info);
}

void mapCallBack(const OccupancyGridConstPtr& msg) {
  robot.occupancyGrid = *msg;
}

void mapUpdateCallBack(const OccupancyGridUpdateConstPtr& msg) {
  updateOccupancyGrid(robot.occupancyGrid, *msg);
}

void handleEnd(const std_msgs::StringConstPtr& msg) {
  bool endFlag = msg->data.compare(endMsg) == 0;

  if (!endFlag) return;

  // do nothing for the time being
}

//////////
// main //
//////////

int main(int argc, char* argv[]) {
  // Init node
  ros::init(argc, argv, "robot");
  ros::NodeHandle n;

  // Load params

  /// Global params
  FAIL_IFN(n.param<float> ("/robot_speed", robot.robotSpeed, 0));
  FAIL_IFN(n.param<float> ("/robot_sensor_range", robot.sensorRange, 0));

  float cellSize;
  FAIL_IFN(n.param<float>   ("/cell_size", cellSize, cellSize));
  robot.meterToCells = round(1/cellSize);

  // Get namespace
  string nameSpace = ros::this_node::getNamespace();
  robot.name = nameSpace.erase(0, 1);

  // Initilize Publishers
  bidPub             = n.advertise<Bid>("bid", 1);
  goalPathPub        = n.advertise<GoalList>("goalPath", 1, true);
  requestObjetivePub = n.advertise<std_msgs::Float64>("/request_objetive", 1);
  pathMarkerPub      = n.advertise<visualization_msgs::Marker>("/robot_path_marker", 1);
  posMarkerPub       = n.advertise<visualization_msgs::Marker>("/robot_pos_marker", 1);

  // Initilize Subscribers
  odomSub       = n.subscribe("odom", 1, odomCallback);
  auctionSub    = n.subscribe("/auction", 1, auctionCallBack);
  assignmentSub = n.subscribe("/" + robot.name + "/assigment", 1, assignmentCallback);
  pathResultSub = n.subscribe("path_result", 1, pathResultCallback);
  mapSub        = n.subscribe<OccupancyGrid>("/map", 1, mapCallBack);
  mapUpdateSub  = n.subscribe<OccupancyGridUpdate>("/map_update", 100000, mapUpdateCallBack);
  endSub        = n.subscribe("/end", 1, handleEnd);

  // spin
  ROS_INFO("Initialized");
  ros::spin();

  return 0;
}
