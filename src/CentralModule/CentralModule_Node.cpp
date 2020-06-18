#include "CentralModule.h"

using namespace std;

/*
 *  Variables
 */

// Topics
/// Subscribers
map<string, ros::Subscriber> bids;

map<string, ros::Subscriber> segment_bids;

ros::Subscriber end_sub;
ros::Subscriber map_merged_sub;
ros::Subscriber request_objetive_sub;

/// Publishers
ros::Publisher take_obj_pub;
ros::Publisher objetive_pub;
ros::Publisher obj_pub2;
ros::Publisher marker_pub;
ros::Publisher segment_auction_pub;
map<string, ros::Publisher> segment_assignment_pubs;
// others
ros::Timer auctionTimer;
CentralModule centralModule;
bool FIN = false;

int mapsHandled = 0;

string end_msg("END");

/*
 *  Rviz mark functions
 */

/* Publishes marks corresponding to the gvd to be visualized on rviz */
static void draw_gvd(tscf_exploration::SegmentAuction sac, map_info_type map_info) {
  // Colors
  std_msgs::ColorRGBA blue;
  blue.b = 1.0f;
  blue.a = 1.0;
  std_msgs::ColorRGBA yellow;
  yellow.r = 1.0f;
  yellow.g = 1.0f;
  yellow.a = 1.0;

  // set up critical points
  visualization_msgs::Marker::_points_type critical_points;
  for (int i = 0; i < sac.criticals.size(); i++) {
    critical_points.push_back(p2d_to_p3d(sac.criticals[i], map_info));
  }

  // set up vertices points
  visualization_msgs::Marker::_points_type points;
  for (int i = 0; i < sac.gvd.vertices.size(); i++) {
    points.push_back(p2d_to_p3d(sac.gvd.vertices[i], -0.1, map_info));
  }

  // edges
  visualization_msgs::Marker::_points_type edges;
  for (int i = 0; i < sac.gvd.edges.size(); i++) {
    edges.push_back(p2d_to_p3d(sac.gvd.edges[i].from, map_info));
    edges.push_back(p2d_to_p3d(sac.gvd.edges[i].to, map_info));
  }

  // publish
  marker_pub.publish(mark_points("gvd_vertices", points, blue));
  marker_pub.publish(mark_points("gvd_critical_vertices", critical_points, yellow));
  marker_pub.publish(mark_lines("gvd_edges", edges, blue));
}

/*
 *  Handle Functions
 */

// Aux functions
void startAuction() {
  ROS_INFO("Segment Auction Start");

  // initialization
  ROS_INFO("1");
  auctionTimer.stop();
  auctionTimer.setPeriod(ros::Duration(4.0));
  centralModule.setEstado(WaitingFirstBid);
  ROS_INFO("2");
  map_info_type map_info = centralModule.getMap().info;
  ROS_INFO("3");
  // get aution info
  tscf_exploration::SegmentAuction segment_auction;
  GVD gvd;
  ROS_INFO("Getting segment Auction");
  boost::tie(segment_auction, gvd) = centralModule.getSegmentAuctionInfo();
  ROS_INFO("Got segment Auction");

  // set markers for rviz gvd visualization
  draw_gvd(segment_auction, map_info);

  // Send auction info: Graph and criticals info
  segment_auction_pub.publish(segment_auction);

  ROS_INFO("Segment Auction End");


}

// Handler Functions
//bool pending = false;
/* cuando: auctionTimer timeout */
/* que: ejecucion subasta (asignacion de tareas) y publicacion de resultados */
void timerRoutine(const ros::TimerEvent&) {
  if (centralModule.getEstado() == WaitingBids) {
    ROS_INFO("Auction Resolution Start");

    // set up 
    visualization_msgs::Marker::_points_type points;
    map_info_type map_info = centralModule.getMap().info;

    boost::unordered_map<string, tscf_exploration::SegmentAssignment> assignment = centralModule.assignSegment();
    for (auto it = assignment.begin(); it != assignment.end(); it++) {
      tscf_exploration::SegmentAssignment sa = it->second;
      string s = it->first;

      segment_assignment_pubs[it->first].publish(it->second);

      for (auto it = sa.frontiers.begin(); it != sa.frontiers.end(); it++){
        points.push_back(p2d_to_p3d(*it, 0.1, map_info));
      } 
    }

    std_msgs::ColorRGBA green;
    green.g = 1.0f;
    green.a = 1.0f;
    marker_pub.publish(mark_points("Frontiers", points, green));
    ROS_INFO("Auction end");
  } else {
    startAuction();
    ROS_WARN("WARNING: auction timeout with no bids, starting a new one...");
  }
  centralModule.setEstado(WaitingAuction);
}

/* cuando: un robot te pide un objetivo */
/* que: inicia una subasta */
void handleRequest(const std_msgs::StringConstPtr& msg) {
  if (!FIN && (centralModule.getEstado() == WaitingAuction) ) {
    ROS_INFO("Auction request successful");
    startAuction();
  }else{
    ROS_INFO("Auction request ignored, already one on course");
  }
}

bool first =true;
/* cuando: llega un nuevo mapa */
/* que: actualizar mapa si no se estan esperando ofertas ni se termino
                y si es el decimo mapa recibido inicia una subasta*/
void handleNewMap(const tscf_exploration::mapMergedInfoConstPtr& msg) {
  centralModule.updateMap(msg);
  if(first){
    first = false;
    startAuction();
  }
}

void handleEnd(const std_msgs::StringConstPtr& msg) {
  string str1(msg->data.c_str());
  FIN = (str1.compare(end_msg) == 0);
  if (FIN) {
    ROS_INFO("CENTRAL MODULE :: FIN");
    ros::Rate loop_rate(1);
    loop_rate.sleep();
    ros::shutdown();
  }
}

void handleSegmentBid(const tscf_exploration::SegmentBidConstPtr& msg, string name) {
  if (!FIN) {
    // centralModule.saveSegmentBid(*msg, name);
    bool successful = centralModule.saveSegmentBid(*msg, name);
    if(successful){     
      if(centralModule.getEstado() == WaitingFirstBid){
        ROS_INFO("Got segment_bid from %s, starting timer", name.c_str());
        centralModule.setEstado(WaitingBids);
        auctionTimer.start();
      }else{
        ROS_INFO("Got segment_bid from %s, timer already started", name.c_str());
      }
    }else{
      ROS_INFO("Got OLD segment_bid from %s", name.c_str());
    }
  }
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "map_merger");
  ros::NodeHandle n;

  auctionTimer = n.createTimer(ros::Duration(3.0), timerRoutine, true);

  // Publishers
  take_obj_pub = n.advertise<tscf_exploration::takeobjetive>("/take_obj", 1);
  objetive_pub = n.advertise<tscf_exploration::asignacion>("/objetive", 1);
  obj_pub2 = n.advertise<nav_msgs::OccupancyGrid>("/debbi", 1);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  segment_auction_pub = n.advertise<tscf_exploration::SegmentAuction>("/segment_auction", 1);

  // Subscribed to
  map_merged_sub = n.subscribe<tscf_exploration::mapMergedInfo>("/map_merged", 1, handleNewMap);
  end_sub = n.subscribe("/end", 1, handleEnd);
  request_objetive_sub = n.subscribe<std_msgs::String>("/request_objetive", 1, &handleRequest);

  ros::Rate loop_rate(1);

  centralModule = CentralModule();

  /// Wait for the robots to be ready
  int cont_atrv = 0;
  ros::master::V_TopicInfo topic_infos;

  while (cont_atrv < centralModule.getNumRobots()) {
    cont_atrv = 0;
    ros::master::getTopics(topic_infos);
    for (ros::master::V_TopicInfo::const_iterator it_topic = topic_infos.begin();
         it_topic != topic_infos.end(); ++it_topic) {
      const ros::master::TopicInfo& published_topic = *it_topic;
      if (published_topic.name.find("/pose") != string::npos) {
        cont_atrv++;
      }
      /* ROS_INFO("CENTRAL MODULE :: %d", cont_atrv); */
      loop_rate.sleep();
    }
    ROS_INFO("Waiting for %d robots ",
             centralModule.getNumRobots() - cont_atrv);
  }

  /// subscribe to robot specific topics
  ros::master::getTopics(topic_infos);

  for (ros::master::V_TopicInfo::const_iterator it_topic = topic_infos.begin();
       it_topic != topic_infos.end(); ++it_topic) {
    const ros::master::TopicInfo& published_topic = *it_topic;
    if (published_topic.name.find("/pose") != string::npos) {
      string nombre = published_topic.name;
      nombre.erase(0, 1);
      int pos = nombre.find('/');
      nombre = nombre.substr(0, pos);
      // string rep_topic = "/" + nombre + "/bid";
      // bids[nombre] = n.subscribe<tscf_exploration::frontierReport>(
      // rep_topic, 1, boost::bind(&handleReport, _1, nombre));

      string topic = "/" + nombre + "/segment_bid";
      segment_bids[nombre] = n.subscribe<tscf_exploration::SegmentBid>(
          topic, 1, boost::bind(&handleSegmentBid, _1, nombre));

      segment_assignment_pubs[nombre] =
          n.advertise<tscf_exploration::SegmentAssignment>("/" + nombre + "/segment_assigment", 1);
    }
  }

  ROS_INFO("Initialized", centralModule.getNumRobots()); 

  ros::spin();

  return 0;
}

/* cuando: un robot tiene una oferta */
/* que: esta se gurda */
/*void handleReport(const tscf_exploration::frontierReportConstPtr& msg, string name) {
  if (!FIN) {
    //centralModule.saveBid(msg, name); OLD

    ROS_INFO("CENTRAL MODULE :: got bid from %s", name.c_str());
  }
}*/
