#include <ros/ros.h>
#include <cstdlib>
#include <ctime>
#include "Robot.h"

/*
 *  Variables
 */

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
ros::Subscriber coverage_sub;
ros::Subscriber end_sub;

ros::Subscriber frontier_bid_sub;

/// Publishers
ros::Publisher debug_pub;
ros::Publisher end_pub;
ros::Publisher bid_pub;

ros::Publisher segment_bid_pub;
ros::Publisher frontier_bid_pub;

ros::Publisher request_objetive_pub;
ros::Publisher goalPath_pub;
ros::Publisher robot_debug_pub;
ros::Publisher coverage_report_pub;
ros::Publisher end_robots_pub;

// others
Robot robot;

bool FIN = false;

std::string end_msg("END");
/*
 *  Aux Functions
 */

string get_frontier_auction_topic(tscf_exploration::Point2D segment, int auction_id) {
  return "frontier_aution_" + to_string(auction_id) + "_" + to_string(segment.x) + "|" +
         to_string(segment.y);
}

/*
 *  Main Functions
 */

// Handlers
void handlePose(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  robot.savePose(msg);
}

void handlePathSucced(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("%s :: Entro handlePathSucced", robot.getNombre().c_str());
  std_msgs::String msg_request;
  msg_request.data = "signal";
  request_objetive_pub.publish(msg_request);
  /*if (!FIN) {

    std::stringstream ss;
    //std::string ret = robot.getRealInfoGain();
    //ss << robot.getNombre() << msg->data.c_str() << " " << ret;
    if (robot.isFinByError()) {
      std_msgs::String msg_request2;
      std::stringstream ss7;
      ss7 << robot.getNombre();
      msg_request2.data = ss7.str();
      end_robots_pub.publish(msg_request2);
      FIN = true;
      ROS_INFO("%s MUERO", robot.getNombre().c_str());
    }
    msg_request.data = ss.str();

    // ROS_INFO("Salgo handlePathSucced");
  }*/
}

// The robot receives the gvd and criticals_info and pubilshes criticals with the Cis.
void handleSegmentAuction(const tscf_exploration::SegmentAuctionConstPtr& msg) {
  if (!FIN) {
    ROS_INFO("%s :: Me llego el mensaje con los segmentos", robot.getNombre().c_str());
    tscf_exploration::SegmentBid segment_bid = robot.getSegmentBid(*msg);
    ROS_INFO("%s :: Voy a enviar los segment_bid", robot.getNombre().c_str());

    segment_bid_pub.publish(segment_bid);
  }
}

boost::unordered_map<int, tscf_exploration::FrontierBid> frontierBids;
int robot_num = -1;

void handleFrontierBid(const tscf_exploration::FrontierBidConstPtr& msg) {
  // frontierBids[msg->robotId];
}

void handleSegmentAssignment(const tscf_exploration::SegmentAssignmentConstPtr& msg) {
  robot.assigned_segment = p2d_to_pos(msg->segment);

  robot_num = msg->robots_num;

  if (true /*robot_num == 1*/) {
    tscf_exploration::goalList path = robot.getPathToSegment(msg->frontiers[0]);
    if (path.listaGoals.size() > 0)
      goalPath_pub.publish(path);
  } else {
    frontierBids.clear();

    // start the frontier auction
    string topic = get_frontier_auction_topic(msg->segment, msg->id);
    // frontier_bid_sub = nh->subscribe(topic,robot_num , handleCoverage); TODO
    frontier_bid_pub = nh->advertise<tscf_exploration::FrontierBid>(topic, 1);
    // value the segments
    // frontier_bid_pub.publish(my_bids);
  }
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "fp_explorer");

  ros::NodeHandle n;

  nh = &n;

  ros::NodeHandle private_node_handle("~");

  std::string nom = ros::this_node::getNamespace();
  robot.setNombre(nom.erase(0, 1));
  int x_ahora = -1;
  int y_ahora = -1;

  private_node_handle.getParam("init_pose_x", x_ahora);
  private_node_handle.getParam("init_pose_y", y_ahora);

  robot.setPosition(x_ahora, y_ahora);
  // robot.setErrorAverage(0.0);
  // robot.resetCountError();

  // Subscribed to
  pose_sub = n.subscribe("pose", 1, handlePose);
  // take_obj_sub = n.subscribe("/take_obj", 1, handleObjetiveSolicitation);
  // Segment Auction sub
  segment_auction_sub = n.subscribe("/segment_auction", 1, handleSegmentAuction);
  segment_assignment_sub =
      n.subscribe("/" + nom + "/segment_assigment", 1, handleSegmentAssignment);

  path_result_sub = n.subscribe("path_result", 1, handlePathSucced);
  // objetive_sub = n.subscribe("/objetive", 1, handleObjetive);
  //_map_sub = n.subscribe("/" + nom + "/map", 1, handleControlMap);
  // coverage_sub = n.subscribe("/coverage", 1, handleCoverage);
  // end_sub = n.subscribe("/end", 1, handleEnd);

  // Publishers
  // debug_pub = n.advertise<nav_msgs::OccupancyGrid>("/debug", 1);
  bid_pub = n.advertise<tscf_exploration::frontierReport>("bid", 1);

  segment_bid_pub = n.advertise<tscf_exploration::SegmentBid>("segment_bid", 1);

  request_objetive_pub = n.advertise<std_msgs::String>("/request_objetive", 1);
  goalPath_pub = n.advertise<tscf_exploration::goalList>("goalPath", 1, true);
  end_pub = n.advertise<std_msgs::String>("end", 1);
  end_robots_pub = n.advertise<std_msgs::String>("/end_robots", 1);
  robot_debug_pub = n.advertise<nav_msgs::OccupancyGrid>("debugggg", 1);
  coverage_report_pub = n.advertise<std_msgs::String>("/coverage_report", 1);

  ROS_INFO("%s :: inicializado :D", robot.getNombre().c_str());
  ros::spin();

  return 0;
}

/*void handleEnd(const std_msgs::StringConstPtr& msg) {
  std::string str1(msg->data.c_str());
  FIN = (str1.compare(end_msg) == 0);
  if (FIN) {
    std_msgs::String msg_request2;
    std::stringstream ss2;
    ss2 << "END";
    msg_request2.data = ss2.str();
    end_pub.publish(msg_request2);
  }
}*/

/*void handleCoverage(const std_msgs::StringConstPtr& msg) {
  std::string str1(msg->data.c_str());
  std_msgs::String msg_request2;
  std::stringstream ss2;
  ss2 << robot.getNombre() << " " << str1 << " " << robot.getErrorAverage();
  msg_request2.data = ss2.str();
  coverage_report_pub.publish(msg_request2);
}*/

/*void handleObjetive(const tscf_exploration::asignacionConstPtr& msg) {
  if (!FIN) {

      int centro = robot.getobjetive(msg);
      tscf_exploration::goalList path;
      if (centro != -1) {
        nav_msgs::OccupancyGrid p;
        path = robot.getPathToObjetive(centro, msg->obstaculos, p);
        robot_debug_pub.publish(p);
        ROS_INFO("%s :: Publico Camino", robot.getNombre().c_str());
      }else{
        ROS_INFO("%s :: NO TENGO OBJETIVO :C", robot.getNombre().c_str());
      }
      path.indice = msg->indice;  // numero de subasta
      goalPath_pub.publish(path);
      ROS_INFO("%s :: Objective handled", robot.getNombre().c_str());

  }
}*/

/*
void handleControlMap(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  // ROS_INFO("guardo mapa");
  //robot.saveControlMap(msg);
}

// cuando: recibo puntos de interes (objetivos)
// que: Valorarlos y enviarlos a la central
void handleObjetiveSolicitation(const tscf_exploration::takeobjetiveConstPtr& msg) {
  if (!FIN) {
    int indice = msg->indice;
    robot.saveGlobalMap(msg->mapa);
    robot.setCentrosF(msg->centrosf);
    tscf_exploration::frontierReport report = robot.processMap();
    report.indice = indice;
    bid_pub.publish(report);
    ROS_INFO("%s :: make bid", robot.getNombre().c_str());
  }
}*/