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
ros::Timer auctionResolutionTimer;
ros::Duration AuctionResolutionTimeout(0.1);//first AuctionResolutionTimeout
int succesfulBids = 0;

ros::Timer auctionStartDelayTimer;
ros::Duration auctionStartDelayTimeout(2);//1

ros::Timer auctionStartTimer;
ros::Duration auctionStartTimeout;//(5.0);

int assigned_robots = 0;
int requests = 0;

CentralModule centralModule;
bool FIN = false;
ros::Time last_auction_start;
ros::Time last_gvd_start;
ros::Time first_auction;
ros::Duration gvd_time;
ros::Duration gvd_time_increment(0);
set<string> biddingRobots;

string end_msg("END");

string gvd_file_log, increment_gvd_file_log, coverage_file_log;

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
 *  Aux Functions
 */

void startAuction() {
  
  //initialization
  map_info_type map_info = centralModule.getMap().info;

  // get aution info
  tscf_exploration::SegmentAuction segment_auction;

  last_gvd_start = ros::Time::now();
  segment_auction = centralModule.getSegmentAuctionInfo();
  ros::Duration last_gvd_time = gvd_time; 
  gvd_time =  (ros::Time::now() - last_gvd_start);
  gvd_time_increment = max(gvd_time_increment, gvd_time - last_gvd_time);
  // set markers for rviz gvd visualization
  draw_gvd(segment_auction, map_info);
  //log gvd time
  if(LOG){
    ros::Duration current_time = ros::Time::now() - first_auction;
    //string data = to_string(current_time.toSec())+" "+to_string(gvd_time.toSec());
    string time = to_string(current_time.toSec());
    string coverage_per = to_string((centralModule.cell_count/MAP_SIZE)*100);
    
    if(coverage_file_log.empty()){
      coverage_file_log = "covarage";
      coverage_file_log = LOG_FILE_PATH + coverage_file_log + to_string(STARTING_ROBOT_NUMBER) + get_current_date_and_time(); 
    }

    string data = coverage_per+" "+to_string(gvd_time.toSec()); 
    if(gvd_file_log.empty()){
      gvd_file_log = "tiemposGVD";
      gvd_file_log = LOG_FILE_PATH + gvd_file_log + to_string(STARTING_ROBOT_NUMBER) + get_current_date_and_time();
    }
   
    log_data(data, gvd_file_log);

    ros::Duration increment = (gvd_time - last_gvd_time);
    //data = to_string(current_time.toSec())+"  "+to_string(increment.toSec());
    data = coverage_per+"  "+to_string(increment.toSec());
    if(increment_gvd_file_log.empty()){
      increment_gvd_file_log = "incrementoGVD";
      increment_gvd_file_log = LOG_FILE_PATH + increment_gvd_file_log + to_string(STARTING_ROBOT_NUMBER) + get_current_date_and_time();
    }
    
    log_data(data, increment_gvd_file_log);
  }

  // Send auction info: Graph and criticals info
  
  requests = 0;
  centralModule.setEstado(WaitingFirstBid);

  segment_auction_pub.publish(segment_auction);
  ROS_INFO("Segment Auction published");
}

void resolveAuction(){
  centralModule.setEstado(Resolving);
  ros::Duration resolution_time = ros::Time::now() - last_auction_start;  
  ROS_INFO("Auction Resolution Start, expected %f, real %f",AuctionResolutionTimeout.toSec(),resolution_time.toSec());
  AuctionResolutionTimeout = max(ros::Duration(0.5)+resolution_time*2,AuctionResolutionTimeout);
  auctionResolutionTimer.setPeriod(AuctionResolutionTimeout);

  succesfulBids = 0;

  // set up 
  visualization_msgs::Marker::_points_type points;
  map_info_type map_info = centralModule.getMap().info;

  boost::unordered_map<string, tscf_exploration::SegmentAssignment> assignment = centralModule.assignSegment();
  assigned_robots = assignment.size();

   float max_estimated_time = 0;
   //float max_estimated_time_under_gvd = 0;
   //float min_estimated_time = FLT_MAX;

  for (auto it = assignment.begin(); it != assignment.end(); it++) {
    tscf_exploration::SegmentAssignment sa = it->second;
    string s = it->first;
    
    //publish
    segment_assignment_pubs[it->first].publish(it->second);

    //obtain costs
    float estimated_time = (centralModule.segment_bids[it->first][p2d_to_pos(it->second.segment)])/(ROBOT_SPEED);
    //ROS_INFO("estimated_time = %f",estimated_time);

    max_estimated_time = max(max_estimated_time,estimated_time);
    if(estimated_time > (gvd_time + gvd_time_increment + auctionStartDelayTimeout).toSec()){
      assigned_robots--;
      //max_estimated_time_under_gvd = max(max_estimated_time_under_gvd,estimated_time);
    }

    //min_estimated_time = min(min_estimated_time,estimated_time);


    for (auto it = sa.frontiers.begin(); it != sa.frontiers.end(); it++){
      points.push_back(p2d_to_p3d(*it, 0.1, map_info));
    } 
    biddingRobots.clear();
  }

  /*if (max_estimated_time < gvd_time.toSec()+1){
    auctionStartTimeout = max(minAuctionStartTimeout,gvd_time-ros::Duration(min_estimated_time));
  }else{
    auctionStartTimeout = minAuctionStartTimeout-ros::Duration(min_estimated_time);
  _}*/
  //timer = max_estimated_time_under_gvd - min_estimated_time
  //auctionStartTimeout = auctionStartDelayTimeout + max(ros::Duration(0),ros::Duration(max_estimated_time_under_gvd)-ros::Duration(min_estimated_time));
  auctionStartTimeout = (gvd_time + gvd_time_increment + auctionStartDelayTimeout);
  //auctionStartTimeout = ros::Duration((gvd_time.toSec()/8.0) + 2);
  auctionStartTimer.setPeriod(auctionStartTimeout);

  ROS_INFO("gvd time = %f , gvd estimated time = %f, max estimated time %f",(gvd_time).toSec(),(gvd_time + gvd_time_increment + auctionStartDelayTimeout).toSec(),max_estimated_time);


  std_msgs::ColorRGBA green;
  green.g = 1.0f;
  green.a = 1.0f;
  marker_pub.publish(mark_points("Frontiers", points, green));
  ROS_INFO("Auction resoved, %d robots assigned",assigned_robots);

  ros::Duration current_time = ros::Time::now() - first_auction;
  ROS_INFO("Elapsed time: %f",current_time.toSec());
  centralModule.setEstado(WaitingAuction);
}

/*
 *  Timer Functions
 */

//bool pending = false;
/* cuando: auctionResolutionTimer AuctionResolutionTimeout */
/* que: ejecucion subasta (asignacion de tareas) y publicacion de resultados */



void auctionResolutionTimerRoutine(const ros::TimerEvent&) {
  auctionResolutionTimer.stop();
  if (centralModule.getEstado() == WaitingBids) {
    resolveAuction();
  } else {
    ROS_WARN("WARNING: auction AuctionResolutionTimeout with no bids, starting a new one...");
  }
}


void auctionStartTimerRoutine(const ros::TimerEvent&) {
  auctionStartTimer.stop();
  ROS_INFO("Segment Auction triggered BECOUSE of timeout");
  startAuction();
}

void auctionStartDelayTimerRoutine(const ros::TimerEvent&) {
  ROS_INFO("Segment Auction triggered BEFORE timeout");
  auctionStartDelayTimer.stop();
  startAuction();
}

/*
 *  Handle Functions
 */


/* cuando: un robot te pide un objetivo */
/* que: inicia una subasta */
void handleRequest(const std_msgs::StringConstPtr& msg) {
  if (centralModule.getEstado() != WaitingAuction){
    ROS_INFO("Auction request from ignored, already one on course");
  }else{
    requests++;
    if(requests >= assigned_robots){
      //ROS_INFO("Auction request successful, it is the last one starting the auction");
      auctionStartTimer.stop();
      auctionStartDelayTimer.start();//delay to wait for the map
      //startAuction();
    } else {
      //ROS_INFO("Auction request successful, starting timer, %d robots left",assigned_robots-requests);

      auctionStartTimer.start();

    }
  }
}

bool first =true;
/* cuando: llega un nuevo mapa */
/* que: actualizar mapa si no se estan esperando ofertas ni se termino
                y si es el decimo mapa recibido inicia una subasta*/
void handleNewMap(const tscf_exploration::mapMergedInfoConstPtr& msg) {
  centralModule.updateMap(msg);
  if(first){
    first_auction = ros::Time::now();
    first = false;
    startAuction();
  }
}

void handleEnd(const std_msgs::StringConstPtr& msg) {
  string str1(msg->data.c_str());
  FIN = (str1.compare(end_msg) == 0);
  if (FIN) {
    //log data
    if(LOG){
      ros::Duration current_time = ros::Time::now() - first_auction;
      string robot_count = to_string(STARTING_ROBOT_NUMBER);
      string time = to_string(current_time.toSec());

      string data = "Cantidad Robots: " + robot_count+ "\n";
      data += "Tiempo de ejecucion: " + time;
      string ejecucion_file_log = "tiemposEjecucion"; 
      ejecucion_file_log = LOG_FILE_PATH + ejecucion_file_log;
      log_data(data, ejecucion_file_log);
      
      ejecucion_file_log = "tiemposEjecucionG";
      ejecucion_file_log = LOG_FILE_PATH + ejecucion_file_log;
      string g_data = robot_count+"  "+time;
      log_data(g_data, ejecucion_file_log);
      log_data(time, (ejecucion_file_log + robot_count));
      
      gnuplot p;
      p.graph_file(gvd_file_log, "Tiempo", "Tiempo de GVD");
      p.graph_file(increment_gvd_file_log, "Tiempo", "Incremento de tiempo de GVD");
      p.graph_file(ejecucion_file_log, "Numero de robots", "Tiempo de ejecucion");
    }


    ROS_INFO("Stopping CENTRAL MODULE");
    ros::Rate loop_rate(1);
    loop_rate.sleep();
    ros::shutdown();

  }
}

void handleSegmentBid(const tscf_exploration::SegmentBidConstPtr& msg, string name) {
  //ROS_INFO("Got segment_bid %d",centralModule.getEstado());
  if (centralModule.getEstado() != WaitingFirstBid && centralModule.getEstado() != WaitingBids){
    return;
  }

  bool successful = centralModule.saveSegmentBid(*msg, name);
  if(!successful || biddingRobots.find(name) != biddingRobots.end()){
    ROS_INFO("Got OLD segment_bid from %s", name.c_str());
  }else{
    biddingRobots.insert(name);
    succesfulBids++;

    if(centralModule.getEstado() == WaitingFirstBid){
      ROS_INFO("Got segment_bid from %s, id %d, starting timer", name.c_str(),msg->id);
      centralModule.setEstado(WaitingBids);
      last_auction_start=ros::Time::now();
      auctionResolutionTimer.start();

    }else if(succesfulBids == centralModule.getNumRobots()){
      auctionResolutionTimer.stop();
      resolveAuction();
      ROS_INFO("Got the last segment bid from %s, id %d, stopping timer and starting resultion", name.c_str(),msg->id);
    }else{
      ROS_INFO("Got segment_bid from %s, timer already started, id %d", name.c_str(),msg->id);
    }
  }
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "map_merger");
  ros::NodeHandle n;

  auctionResolutionTimer = n.createTimer(AuctionResolutionTimeout, auctionResolutionTimerRoutine, true,false);
  auctionStartTimer = n.createTimer(auctionStartTimeout, auctionStartTimerRoutine, true,false);
  auctionStartDelayTimer = n.createTimer(auctionStartDelayTimeout, auctionStartDelayTimerRoutine, true,false);
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
