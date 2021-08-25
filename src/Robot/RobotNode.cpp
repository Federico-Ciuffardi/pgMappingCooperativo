#include <ros/ros.h>
#include <cstdlib>
#include <ctime>
#include "nav_msgs/Odometry.h"
#include "Robot.h"
#include "ros/init.h"
#include "ros/this_node.h"

////////////////
// Parameters //
////////////////

// Rviz params

/// Markers used to represent cells
unsigned int cellMarkerType = RvizHelper::CUBE_LIST;

/// Markers dimensions
float cubeHeight = 0.02;

/// Layers (planes orthogonal to the z axis)

//// Separation between each layer
float layerSeparation = cubeHeight;

//// Assign markers to layers
float pathLineZ = 11*layerSeparation;
float posZ      = 10*layerSeparation;


///////////////
// Variables //
///////////////

// Topics
/// Subscribers
ros::Subscriber odom_sub;

ros::Subscriber auction_sub;
ros::Subscriber assignment_sub;

ros::Subscriber path_result_sub;
ros::Subscriber end_sub;

ros::Subscriber map_merged_sub;

/// Publishers
ros::Publisher end_pub;
ros::Publisher bid_pub;

ros::Publisher request_objetive_pub;
ros::Publisher goalPath_pub;
ros::Publisher marker_pub;

// others
Robot robot;
RvizHelper rvizHelper;

string endMsg("END");

////////////////
// Rviz marks //
////////////////

void setRvizMarks(Robot &r, pgmappingcooperativo::goalList &path, mapInfoType mapInfo) {

  float cellSize = mapInfo.resolution;

  // draw path to objective
  rvizHelper.topic = &marker_pub;

  rvizHelper.color    = MAGENTA;
  rvizHelper.type     = RvizHelper::LINE_STRIP;
  rvizHelper.scale    = makeVector3(cellSize*0.15, cellSize, 1);
  rvizHelper.position = makeVector3(0,0,pathLineZ);
  rvizHelper.mark(path.listaGoals, robot.name + "_path");

  // Mark Current position on rviz
  rvizHelper.topic = &marker_pub;

  RvizHelper::MarkerPoints posMarkerPoint;
  posMarkerPoint.push_back(toMarkerPoint(toPos(robot.position, robot.map_merged.mapa.info), robot.map_merged.mapa.info));

  rvizHelper.color    = makeColorRGBA(0.75);
  rvizHelper.type     = cellMarkerType;
  rvizHelper.scale    = makeVector3(cellSize*0.5); rvizHelper.scale.z  = cubeHeight;
  rvizHelper.position = makeVector3(0,0,posZ);
  rvizHelper.mark(posMarkerPoint, robot.name + "_pos");
}

///////////////////
// Aux Functions //
///////////////////

void publishPath(Pos frontier) {
  // Get path
  pgmappingcooperativo::goalList path = robot.getPathTo(frontier);

  // Publish the path
  goalPath_pub.publish(path);
  
  // Mark path info on gvd
  setRvizMarks(robot, path,  robot.map_merged.mapa.info);
}

///////////////
// CallBacks //
///////////////

void odomCallback(const nav_msgs::Odometry::ConstPtr &odom) {
  // Save current pose
  robot.savePose(odom->pose.pose);
}

void pathSucceedCallback(const std_msgs::String::ConstPtr& msg) {

  ROS_INFO("Arrived to the objective, requesting objective");
  std_msgs::String msg_request;
  msg_request.data = "signal";
  request_objetive_pub.publish(msg_request);

  // Delete the path mark
  rvizHelper.deleteMark(robot.name + "_path", 0);
}

void auctionCallBack(const pgmappingcooperativo::AuctionConstPtr& msg) {
  // Validate
  if (robot.lastAuctionId >= msg->id) {
    ROS_INFO_STREAM("An old auction arrived: last_id >= id | "<<robot.lastAuctionId<<" >= "<<msg->id);
    return;
  }

  // Process
  ROS_INFO("Auction message arrived");
  robot.lastAuctionId = msg->id;

  ROS_INFO("Calculate bids");
  pgmappingcooperativo::Bid segment_bid = robot.getBid(*msg);

  ROS_INFO("Publish bids");
  segment_bid.id = msg->id;
  bid_pub.publish(segment_bid);
}

void assignmentCallback(const pgmappingcooperativo::AssignmentConstPtr& msg) {
  // Validate
  if (robot.lastAssignmentId >= msg->id) {
    ROS_INFO_STREAM("An old assingment arrived: last_id >= id | "<<robot.lastAssignmentId<<" >= "<<msg->id);
    return;
  }

  // Process
  ROS_INFO("A assingment arived");
  robot.lastAssignmentId = msg->id;

  ROS_INFO("Publish the path that leads to the assingment");
  publishPath(toPos(msg->frontier));
}

void mapMergedCallBack(const pgmappingcooperativo::mapMergedInfoConstPtr& msg) {
  robot.map_merged = (*msg);
}

void handleEnd(const std_msgs::StringConstPtr& msg) {

  bool endFlag = msg->data.compare(endMsg) == 0;

  if (!endFlag) return;

  // do nothing for the time being

}

int main(int argc, char* argv[]) {
  // Init node
  ros::init(argc, argv, "robot");
  ros::NodeHandle n;

  // Load params

  /// Global params
  assert(n.param<float> ("/robot_speed", robot.robotSpeed, 0));
  assert(n.param<float> ("/robot_sensor_range", robot.sensorRange, 0));

  // Get namespace
  string nameSpace = ros::this_node::getNamespace();
  robot.name = nameSpace.erase(0, 1);

  // Initilize Publishers
  bid_pub      = n.advertise<pgmappingcooperativo::Bid>("bid", 1);
  goalPath_pub         = n.advertise<pgmappingcooperativo::goalList>("goalPath", 1, true);
  request_objetive_pub = n.advertise<std_msgs::String>("/request_objetive", 1);
  marker_pub           = n.advertise<visualization_msgs::Marker>("/visualization_marker", 10);

  // Initilize Subscribers
  odom_sub        = n.subscribe("odom", 1, odomCallback);
  auction_sub     = n.subscribe("/auction", 1, auctionCallBack);
  assignment_sub  = n.subscribe("/" + robot.name + "/assigment", 1, assignmentCallback);
  path_result_sub = n.subscribe("path_result", 1, pathSucceedCallback);
  map_merged_sub  = n.subscribe<pgmappingcooperativo::mapMergedInfo>("/map_merged", 1, mapMergedCallBack);
  end_sub         = n.subscribe("/end", 1, handleEnd);

  // spin
  ROS_INFO("Initialized");
  ros::spin();

  return 0;
}
