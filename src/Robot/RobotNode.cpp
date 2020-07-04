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

ros::Subscriber map_merged_sub;

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
ros::Publisher marker_pub;

// others
Robot robot;
tscf_exploration::goalList path;
pos f;

// intends to prevent handling multiple auction at one time
// could fail if a new segment auction starts before the segment assignment message arrives
// could be fixed by making it block all the way and making that all the robots receives an advice
// of the end of the bid
bool handlingAuction = false;

bool FIN = false;
std::string end_msg("END");

bool first_frontier = true;
pos last_frontier;
pos current_frontier;
/*
 *  Aux Functions
 */

string get_frontier_auction_topic(tscf_exploration::Point2D segment, int auction_id) {
  return "/frontier_aution_" + to_string(auction_id) + "_" + to_string(segment.x) + "_" +
         to_string(segment.y);
}

void publishPath(pos frontier) {
  f = frontier;
  path = robot.getPathToSegment(frontier);

  // draw objective
  visualization_msgs::Marker::_points_type points;
  geometry_msgs::Point p3d = pos_to_p3d(frontier + robot.offset, 0.2);
  p3d.x += 0.5;
  p3d.y += 0.5;

  points.push_back(p3d);
  std_msgs::ColorRGBA magenta;
  magenta.r = 1.0f;
  magenta.b = 1.0f;
  magenta.a = 1.0f;
  marker_pub.publish(mark_points(robot.getNombre() + "objective", points, magenta));

  // draw path to objective
  visualization_msgs::Marker::_points_type lines;

  for (auto it = path.listaGoals.begin(); it != path.listaGoals.end(); it++) {
    lines.push_back(*it);
  }
  marker_pub.publish(mark_lines(robot.getNombre() + "path", lines, magenta, 0.1,
                                visualization_msgs::Marker::LINE_STRIP));

  // send objective to move controller
  if (path.listaGoals.size() > 0) {
    goalPath_pub.publish(path);
  }
}

/*
 *  Main Functions
 */

// Handlers
void handlePose(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  robot.savePose(msg);
}

void handlePathSucced(const std_msgs::String::ConstPtr& msg) {
  // if(first_frontier || current_frontier != last_frontier || msg->data == "done" ){
  std_msgs::String msg_request;
  msg_request.data = "signal";
  request_objetive_pub.publish(msg_request);
  // set last frontier
  // first_frontier = false;
  // last_frontier = current_frontier;
  //}else{
  // ROS_INFO("%s :: Arrived to the objective AGAIN, NOT requesting objective",
  // robot.getNombre().c_str());
  //}
  ROS_INFO("%s", msg->data);
  // clear las objective on rviz
  if (msg->data == "OBSTACLE") {
    ROS_INFO("%s :: Found an obstacle on my path, requesting objective", robot.getNombre().c_str());

    std_msgs::ColorRGBA grey;
    grey.r = 0.6f;
    grey.g = 0.6f;
    grey.b = 0.6f;
    grey.a = 1.0f;

    visualization_msgs::Marker::_points_type points;
    geometry_msgs::Point p3d = pos_to_p3d(f + robot.offset, 0.2);
    p3d.x += 0.5;
    p3d.y += 0.5;

    points.push_back(p3d);
    marker_pub.publish(mark_points(robot.getNombre() + "objective", points, grey));

    // draw path to objective
    visualization_msgs::Marker::_points_type lines;

    for (auto it = path.listaGoals.begin(); it != path.listaGoals.end(); it++) {
      lines.push_back(*it);
    }
    marker_pub.publish(mark_lines(robot.getNombre() + "path", lines, grey, 0.1,
                                  visualization_msgs::Marker::LINE_STRIP));
  } else {
    ROS_INFO("%s :: Arrived to the objective, requesting objective", robot.getNombre().c_str());
    marker_pub.publish(delete_marks(robot.getNombre() + "objective"));
    marker_pub.publish(delete_marks(robot.getNombre() + "path"));
  }
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

// int last_segment_auction_id = -1;
// The robot receives the gvd and criticals_info and pubilshes criticals with the Cis.
void handleSegmentAuction(const tscf_exploration::SegmentAuctionConstPtr& msg) {
  if (robot.last_segment_auction_id >= msg->id) {
    ROS_INFO("%s :: An old segment auction arrived: last_id >= id, %d >= %d",
             robot.getNombre().c_str(), robot.last_segment_auction_id, msg->id);
    return;
  }
  if (handlingAuction) {
    ROS_INFO("%s :: Segment auction arriving but already processing one, discarding it",
             robot.getNombre().c_str());
    return;
  }

  // handlingAuction = true; //prender el bloqueo

  robot.last_segment_auction_id = msg->id;
  if (!FIN) {  //&& msg->id>last_id) {
    ROS_INFO("%s :: A segment message arrived", robot.getNombre().c_str());
    tscf_exploration::SegmentBid segment_bid = robot.getSegmentBid(*msg);
    ROS_INFO("%s :: Sending my bids", robot.getNombre().c_str());
    segment_bid.id = msg->id;
    segment_bid_pub.publish(segment_bid);
  }
}

boost::unordered_map<int, tscf_exploration::FrontierBid> frontierBids;
int robots_num = -1;

void handleFrontierBid(const tscf_exploration::FrontierBidConstPtr& msg) {
  // ROS_INFO("%s :: a frontier bid of %d arrived", robot.getNombre().c_str(),msg->robotId);
  // if(robot.last_frontier_auction_id > msg->id) return;

  // robot.last_frontier_auction_id = msg->id;
  if (robot.auction_robots <= robots_num) {
    robot.saveFrontierBid(*msg);
    if (robot.auction_robots == robots_num) {
      pos frontier = robot.assignFrontier();
      publishPath(frontier);
      current_frontier = frontier;
      ROS_INFO("%s :: Frontier auction ended, moving to (%d,%d)", robot.getNombre().c_str(),
               frontier.first, frontier.second);

      handlingAuction = false;
    }
  }
}

void handleSegmentAssignment(const tscf_exploration::SegmentAssignmentConstPtr& msg) {
  if (robot.last_segment_assignment_id >= msg->id) {
    ROS_INFO("%s :: An old frontier assingment arrived: last_id >= id, %d >= %d",
             robot.getNombre().c_str(), robot.last_segment_assignment_id, msg->id);
    return;
  }
  if (msg->assigned == 0) {
    ROS_INFO("%s :: Rejected for the auction %d", robot.getNombre().c_str(), msg->id);
    handlingAuction = false;
    tscf_exploration::goalList path;
    path.listaGoals.push_back(robot.getPosition());
    goalPath_pub.publish(path);
    return;
  }
  ROS_INFO("%s :: A segment assingment arived", robot.getNombre().c_str());
  robot.last_segment_assignment_id = msg->id;
  robot.assigned_segment = p2d_to_pos(msg->segment);

  robots_num = msg->robots_num;

  // frontierBids.clear();
  clear_bids(robot.bids_pq);
  robot.auction_robots = 0;
  tscf_exploration::FrontierBid frontiers_bid = robot.getFrontierBid(msg->frontiers);
  // ROS_INFO("romi ya sali de la funcion para armar las frontierbids");
  // start the frontier auction
  string topic = get_frontier_auction_topic(msg->segment, msg->id);
  // ROS_INFO("topico dinamico para el intercambio de fronteras: %s",topic.c_str());
  // ROS_INFO("romi me suscribi al topico de las frontierbids");
  frontier_bid_sub = nh->subscribe(topic, robots_num, handleFrontierBid);
  frontier_bid_pub = nh->advertise<tscf_exploration::FrontierBid>(topic, robots_num, true);
  ROS_INFO("%s :: Starting a frontier auction", robot.getNombre().c_str());
  frontier_bid_pub.publish(frontiers_bid);
}

void handleNewMap(const tscf_exploration::mapMergedInfoConstPtr& msg) {
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

  map_merged_sub = n.subscribe<tscf_exploration::mapMergedInfo>("/map_merged", 1, handleNewMap);

  // objetive_sub = n.subscribe("/objetive", 1, handleObjetive);
  //_map_sub = n.subscribe("/" + nom + "/map", 1, handleControlMap);
  // coverage_sub = n.subscribe("/coverage", 1, handleCoverage);
  end_sub = n.subscribe("/end", 1, handleEnd);

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
  marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 10);

  ROS_INFO("%s :: initialized", robot.getNombre().c_str());
  ros::spin();

  return 0;
}

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