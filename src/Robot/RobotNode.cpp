#include <ros/ros.h>
#include <cstdlib>
#include <ctime>
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "Robot.h"

///////////////
// Variables //
///////////////

ros::NodeHandle* nh = NULL;

// Topics
/// Subscribers
ros::Subscriber pose_sub;
ros::Subscriber take_obj_sub;

ros::Subscriber segment_auction_sub;
ros::Subscriber segment_assignment_sub;

ros::Subscriber path_result_sub;
ros::Subscriber objetive_sub;
ros::Subscriber _map_sub;
ros::Subscriber end_sub;

ros::Subscriber frontier_bid_sub;

ros::Subscriber map_merged_sub;

/// Publishers
ros::Publisher debug_pub;
ros::Publisher end_pub;
ros::Publisher bid_pub;
ros::Publisher pose_pub;
ros::Publisher segment_bid_pub;
ros::Publisher frontier_bid_pub;

ros::Publisher request_objetive_pub;
ros::Publisher goalPath_pub;
ros::Publisher end_robots_pub;
ros::Publisher marker_pub;

// others
Robot robot;
RvizHelper rvizHelper;
pgmappingcooperativo::goalList path;
Pos f;

// intends to prevent handling multiple auction at one time
// could fail if a new segment auction starts before the segment assignment message arrives
// could be fixed by making it block all the way and making that all the robots receives an advice
// of the end of the bid
bool handlingAuction = false;
bool FIN = false;
std::string end_msg("END");

bool first_frontier = true;
Pos last_frontier;
Pos current_frontier;

///////////////////
// Aux Functions //
///////////////////
float layerSeparation = 0.175;

void publishPath(Pos frontier) {
  f = frontier;
  path = robot.getPathTo(frontier);
  cout<<"Current pos: "<<robot.getGVDPos()<<endl;
  cout<<"Path to "<<frontier<<": "<<path<<endl;

  // draw objective
  visualization_msgs::Marker::_points_type points;
  geometry_msgs::Point p3d = toPoint(frontier + robot.getOffset(), 0.2);
  p3d.x += 0.5;
  p3d.y += 0.5;

  points.push_back(p3d);
  std_msgs::ColorRGBA magenta;
  magenta.r = 1.0f;
  magenta.b = 1.0f;
  magenta.a = 1.0f;
  /* marker_pub.publish(rvizHelper.mark_points(robot.getNombre() + "objective", points, magenta)); */

  // draw path to objective
  visualization_msgs::Marker::_points_type lines;

  for (auto it = path.listaGoals.begin(); it != path.listaGoals.end(); it++) {
    lines.push_back(*it);
  }
  marker_pub.publish(rvizHelper.mark_lines(robot.getNombre() + "path", lines, magenta, layerSeparation*5,
                                visualization_msgs::Marker::LINE_STRIP));

  // send objective to move controller
  if (path.listaGoals.size() > 0) {
    goalPath_pub.publish(path);
  }
}

///////////////
// CallBacks //
///////////////

void odomCallback(const nav_msgs::Odometry::ConstPtr &odom) {
  robot.savePose(odom->pose.pose);
  geometry_msgs::PoseStamped ps;
  ps.header = odom->header;
  ps.pose = odom->pose.pose;

  pose_pub.publish(ps);
}

void pathSucceedCallback(const std_msgs::String::ConstPtr& msg) {
  // if(first_frontier || current_frontier != last_frontier || msg->data == "done" ){
  std_msgs::String msg_request;
  msg_request.data = "signal";
  request_objetive_pub.publish(msg_request);
  ROS_INFO("%s", msg->data.c_str());
  // clear las objective on rviz
  if (msg->data == "OBSTACLE") {
    ROS_INFO("%s :: Found an obstacle on my path, requesting objective", robot.getNombre().c_str());

    std_msgs::ColorRGBA grey;
    grey.r = 0.6f;
    grey.g = 0.6f;
    grey.b = 0.6f;
    grey.a = 1.0f;

    visualization_msgs::Marker::_points_type points;
    geometry_msgs::Point p3d = toPoint(f + robot.getOffset(), 0.2);
    p3d.x += 0.5;
    p3d.y += 0.5;

    points.push_back(p3d);
    /* marker_pub.publish(rvizHelper.mark_points(robot.getNombre() + "objective", points, grey)); */

    // draw path to objective
    visualization_msgs::Marker::_points_type lines;

    for (auto it = path.listaGoals.begin(); it != path.listaGoals.end(); it++) {
      lines.push_back(*it);
    }
    marker_pub.publish(rvizHelper.mark_lines(robot.getNombre() + "path", lines, grey, 0.1, visualization_msgs::Marker::LINE_STRIP));
  } else {
    ROS_INFO("%s :: Arrived to the objective, requesting objective", robot.getNombre().c_str());
    marker_pub.publish(rvizHelper.delete_marks(robot.getNombre() + "objective"));
    marker_pub.publish(rvizHelper.delete_marks(robot.getNombre() + "path"));
  }
}

void auctionCallBack(const pgmappingcooperativo::AuctionConstPtr& msg) {
  cout<<"Segment auction arrived| ID: "<<msg->id<<endl;

  if (robot.lastAuctionId >= msg->id) {
    ROS_INFO("%s :: An old segment auction arrived: last_id >= id, %d >= %d",
             robot.getNombre().c_str(), robot.lastAuctionId, msg->id);
    return;
  }

  robot.lastAuctionId = msg->id;

  ROS_INFO("%s :: A segment message arrived", robot.getNombre().c_str());
  pgmappingcooperativo::Bid segment_bid = robot.getBid(*msg);
  ROS_INFO("%s :: Sending my bids", robot.getNombre().c_str());
  segment_bid.id = msg->id;
  segment_bid_pub.publish(segment_bid);
}

boost::unordered_map<int, pgmappingcooperativo::FrontierBid> frontierBids;
int robots_num = -1;

void assignmentCallback(const pgmappingcooperativo::AssignmentConstPtr& msg) {

  if (robot.lastAssignmentId >= msg->id) {
    ROS_INFO("%s :: An old frontier assingment arrived: last_id >= id, %d >= %d",
             robot.getNombre().c_str(), robot.lastAssignmentId, msg->id);
    return;
  }

  ROS_INFO("%s :: A segment assingment arived", robot.getNombre().c_str());
  robot.lastAssignmentId = msg->id;

  publishPath(toPos(msg->frontier));
}

void handleNewMap(const pgmappingcooperativo::mapMergedInfoConstPtr& msg) {
  robot.map_merged = (*msg);
}

void handleEnd(const std_msgs::StringConstPtr& msg) {
  std::string str1(msg->data.c_str());
  FIN = (str1.compare(end_msg) == 0);
  if (FIN) {
    std_msgs::String msg_request2;
    std::stringstream ss2;
    ss2 << "END";
    msg_request2.data = ss2.str();
    // end_pub.publish(msg_request2);
  }
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "fp_explorer");
  nh = new ros::NodeHandle();
  ros::NodeHandle n = *nh;

  ros::NodeHandle private_node_handle("~");

  std::string nom = ros::this_node::getNamespace();
  robot.setNombre(nom.erase(0, 1));

  // Subscribed to
  ros::Subscriber odom_sub = n.subscribe("odom", 1, odomCallback);

  // Segment Auction sub
  segment_auction_sub = n.subscribe("/auction", 1, auctionCallBack);
  segment_assignment_sub = n.subscribe("/" + nom + "/assigment", 1, assignmentCallback);

  path_result_sub = n.subscribe("path_result", 1, pathSucceedCallback);

  map_merged_sub = n.subscribe<pgmappingcooperativo::mapMergedInfo>("/map_merged", 1, handleNewMap);

  end_sub = n.subscribe("/end", 1, handleEnd);

  // Publishers
  pose_pub = n.advertise<geometry_msgs::PoseStamped>("pose", 1);
  segment_bid_pub = n.advertise<pgmappingcooperativo::Bid>("bid", 1);
  goalPath_pub = n.advertise<pgmappingcooperativo::goalList>("goalPath", 1, true);
  end_pub = n.advertise<std_msgs::String>("end", 1);

  request_objetive_pub = n.advertise<std_msgs::String>("/request_objetive", 1);
  end_robots_pub = n.advertise<std_msgs::String>("/end_robots", 1);
  marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 10);

  ROS_INFO("%s :: initialized", robot.getNombre().c_str());
  ros::spin();

  return 0;
}
