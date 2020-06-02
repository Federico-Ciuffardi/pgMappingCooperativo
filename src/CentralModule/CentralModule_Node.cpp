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
vector<ros::Publisher> segment_assignment_pubs;
// others
ros::Timer segmentAuctionTimer;
ros::Timer auctionTimer;

CentralModule centralModule;

bool FIN = false;

int mapsHandled = 0;

string end_msg("END");

/*
 *  Rviz mark functions
 */

// typedefs

typedef nav_msgs::OccupancyGrid::_info_type map_info_type;

// Auxiliar functions

/* Converts from `pos` (pair<int,int>) in a grid map to a point (geometry_msgs::Point)
   adjusting the latter with the `map_info` so it lands on it's correspoing world position */
geometry_msgs::Point p2d_to_p3d(tscf_exploration::Point2D p2d, map_info_type map_info) {
  geometry_msgs::Point p3d;
  p3d.x = p2d.x * map_info.resolution + map_info.origin.position.x + 0.5;
  p3d.y = p2d.y * map_info.resolution + map_info.origin.position.y + 0.5;
  p3d.z = 0;
  return p3d;
}

geometry_msgs::Point p2d_to_p3d(tscf_exploration::Point2D p2d,float z, map_info_type map_info){
  geometry_msgs::Point p3d = p2d_to_p3d(p2d, map_info); 
  p3d.z = z;
  return p3d;
}

geometry_msgs::Point pos_to_p3d(pos p, map_info_type map_info) {
  geometry_msgs::Point p3d;
  p3d.x = p.first * map_info.resolution + map_info.origin.position.x + 0.5;
  p3d.y = p.second * map_info.resolution + map_info.origin.position.y + 0.5;
  p3d.z = 0;
  return p3d;
}


/* Publishes mark points `ps` on the namespace `ns` with the color `color` to be visualized on rviz
 */
void mark_points(string ns,
                 visualization_msgs::Marker::_points_type ps,
                 std_msgs::ColorRGBA color) {
  visualization_msgs::Marker points;
  // ns & id
  points.ns = ns;
  points.id = 0;
  // header
  points.header.frame_id = "/world";
  points.header.stamp = ros::Time::now();
  // pose
  points.pose.orientation.w = 1.0;
  // others
  points.type = visualization_msgs::Marker::POINTS;
  points.action = visualization_msgs::Marker::ADD;
  points.scale.x = 1;
  points.scale.y = 1;
  points.color = color;
  points.points = ps;

  marker_pub.publish(points);
}

/* Publishes mark lines `ls` on the namespace `ns` with the color `color` to be visualized on rviz
 */
void mark_lines(string ns, visualization_msgs::Marker::_points_type ls, std_msgs::ColorRGBA color) {
  visualization_msgs::Marker lines;
  // ns & id
  lines.ns = ns;
  lines.id = 0;
  // header
  lines.header.frame_id = "/world";
  lines.header.stamp = ros::Time::now();
  // pose
  lines.pose.orientation.w = 1.0;
  lines.pose.position.z = -0.1;
  // others
  lines.type = visualization_msgs::Marker::LINE_LIST;
  lines.action = visualization_msgs::Marker::ADD;
  lines.scale.x = 0.2;
  lines.color = color;
  lines.points = ls;

  marker_pub.publish(lines);
}

/* Publishes marks corresponding to the gvd to be visualized on rviz */
void draw_gvd(tscf_exploration::SegmentAuction sac, map_info_type map_info) {
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
    points.push_back(p2d_to_p3d(sac.gvd.vertices[i],-0.1, map_info));
  }

  // edges
  visualization_msgs::Marker::_points_type edges;
   for (int i = 0; i < sac.gvd.edges.size(); i++) {
    edges.push_back(p2d_to_p3d(sac.gvd.edges[i].from, map_info));
    edges.push_back(p2d_to_p3d(sac.gvd.edges[i].to, map_info));
  }

  // publish
  mark_points("gvd_vertices", points, blue);
  mark_points("gvd_critical_vertices", critical_points, yellow);
  mark_lines("gvd_edges", edges, blue);
}

/*
 *  Handle Functions
 */

// Aux functions
void startAuction() {
  //initialization
  auctionTimer.stop();
  auctionTimer.setPeriod(ros::Duration(1.0));
  centralModule.resetArrivals();
  centralModule.setEstado(WaitingBids);

  tscf_exploration::takeobjetive ret;

  tscf_exploration::SegmentAuction segment_auction;

  GVD gvd, gvd1;


  // get aution info
  boost::tie(segment_auction, gvd1) = centralModule.getSegmentAuctionInfo();

  boost::tie(ret, gvd) = centralModule.getObjetiveMap();

  // set markers for rviz
  visualization_msgs::Marker::_points_type ps;
  for (auto it = ret.centrosf.begin(); it != ret.centrosf.end(); it++) {
    ps.push_back(
        pos_to_p3d(pos(*it % ret.mapa.info.width, *it / ret.mapa.info.width), ret.mapa.info));
  }
  std_msgs::ColorRGBA color;
  color.g = 1.0f;
  color.a = 1.0;
  mark_points("interest_points", ps, color);

  draw_gvd(segment_auction, ret.mapa.info);

  // Send autin info: Graph and criticals info
  segment_auction_pub.publish(segment_auction);

  take_obj_pub.publish(ret);  // publica puntos a subastar

  ROS_INFO("CENTRAL MODULE :: auction start");

  auctionTimer.start();
}

// Handler Functions

/* cuando: auctionTimer timeout */
/* que: ejecucion subasta (asignacion de tareas) y publicacion de resultados */
void timerRoutine(const ros::TimerEvent&) {
  if (centralModule.getEstado() == WaitingBids) {
    centralModule.setEstado(WaitingAuction);
    objetive_pub.publish(centralModule.assignTasks());  // ejecucion subasta (asignacion de
                                                        // tareas) y publicacion de resultados
    for(int i = 0; i<segment_assignment_pubs.size(); i++){
      segment_assignment_pubs[i].publish(centralModule.assignSegment());
    }
    ROS_INFO("CENTRAL MODULE :: auction end");
  } else {
    ROS_DEBUG("Wrong triggered");
  }
}

/* cuando: un robot te pide un objetivo */
/* que: inicia una subasta */
void handleRequest(const std_msgs::StringConstPtr& msg) {
  if (!FIN) {
    startAuction();
  }
}

/* cuando: llega un nuevo mapa */
/* que: actualizar mapa si no se estan esperando ofertas ni se termino
                y si es el decimo mapa recibido inicia una subasta*/
void handleNewMap(const tscf_exploration::mapMergedInfoConstPtr& msg) {
  if ((!FIN) && (centralModule.getEstado() != WaitingBids)) {
    /* update map */
    // ROS_INFO("CENTRAL MODULE :: update map");
    centralModule.updateMap(msg);
  }
  mapsHandled++;  // maps handled
  if (mapsHandled == 10) {
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

/* cuando: un robot tiene una oferta */
/* que: esta se gurda */
void handleReport(const tscf_exploration::frontierReportConstPtr& msg, string name) {
  if (!FIN) {
    centralModule.saveBid(msg, name);

    ROS_INFO("CENTRAL MODULE :: got bid from %s", name.c_str());
  }
}

void handleSegmentBid(const tscf_exploration::SegmentBidConstPtr& msg, string name) {
  if (!FIN) {
    //centralModule.saveSegmentBid(*msg, name);

    ROS_INFO("CENTRAL MODULE :: got segment_bid from %s", name.c_str());
  }
}


int main(int argc, char* argv[]) {
  ros::init(argc, argv, "map_merger");
  ros::NodeHandle n;

  auctionTimer = n.createTimer(ros::Duration(3.0), timerRoutine, true);

  //segmentAuctionTimer = n.createTimer(ros::Duration(3.0), timerRoutine, true);

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
    ROS_INFO(" CENTRAL MODULE :: esperando por %d robots ",
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
      string rep_topic = "/" + nombre + "/bid";
      bids[nombre] = n.subscribe<tscf_exploration::frontierReport>(
          rep_topic, 1, boost::bind(&handleReport, _1, nombre));
      
      rep_topic = "/" + nombre + "/segment_bid";
      segment_bids[nombre] = n.subscribe<tscf_exploration::SegmentBid>(
          rep_topic, 1, boost::bind(&handleSegmentBid, _1, nombre));

      segment_assignment_pubs.push_back(n.advertise<tscf_exploration::SegmentAssignment>("/" + nombre + "/segment_bid", 1));
    }
  }

  /* ROS_INFO("CENTRAL MODULE :: Numero %d", centralModule.getNumRobots()); */

  ros::spin();

  return 0;
}
