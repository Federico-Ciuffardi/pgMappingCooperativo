#include "CentralModule.h"
#include "../lib/GVD/src/config.h"

using namespace std;

///////////////
// Variables //
///////////////

// Topics
/// Subscribers
map<string, ros::Subscriber> bids;
map<string, ros::Subscriber> segmentBidSubs;
ros::Subscriber endSub;
ros::Subscriber mapMergedSub;
ros::Subscriber requestObjetiveSub;

/// Publishers
ros::Publisher takeObjPub;
ros::Publisher objetivePub;
ros::Publisher objPub;
ros::Publisher markerPub;
ros::Publisher segmentAuctionPub;
map<string, ros::Publisher> segmentAssignmentPubs;

// timers 
ros::Timer auctionResolutionTimer;
ros::Duration AuctionResolutionTimeout(0.1);  // first AuctionResolutionTimeout

ros::Timer auctionStartDelayTimer;
ros::Duration auctionStartDelayTimeout(2);  // 1

ros::Timer auctionStartTimer;
ros::Duration auctionStartTimeout;  //(5.0);

// others
CentralModule centralModule;

int succesfulBids = 0;
int assignedRobots = 0;
int requests = 0;

bool endFlag = false;

ros::Time lastAuctionStart;
ros::Time lastGvdStart;
ros::Time firstAuction;

ros::Duration gvdTime;
ros::Duration gvdTimeIncrement(0);

string mapName;

string endMsg = "END";

string gvdFileLog;
string incrementGvdFileLog;
string coverageFileLog;

////////////////
// Rviz marks //
////////////////

// Publishes marks corresponding to the gvd to be visualized on rviz 
static void drawGvd(pgmappingcooperativo::SegmentAuction sac, map_info_type map_info) {
  // Colors
  std_msgs::ColorRGBA blue;
  blue.b = 1.0f;
  blue.a = 1.0;
  std_msgs::ColorRGBA yellow;
  yellow.r = 1.0f;
  yellow.g = 1.0f;
  yellow.a = 1.0;

  // set up critical points
  visualization_msgs::Marker::_points_type criticalPoints;
  for (int i = 0; i < sac.criticals.size(); i++) {
    criticalPoints.push_back(p2d_to_p3d(sac.criticals[i], map_info));
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
  markerPub.publish(mark_points("gvd_vertices", points, blue));
  markerPub.publish(mark_points("gvd_critical_vertices", criticalPoints, yellow));
  markerPub.publish(mark_lines("gvd_edges", edges, blue));
}

///////////////////
// Aux Functions //
///////////////////

void startAuction() {
  // initialization
  map_info_type mapInfo = centralModule.getMap().info;

  // get aution info
  pgmappingcooperativo::SegmentAuction segmentAuction;

  lastGvdStart = ros::Time::now();
  ROS_INFO("Computing the segment auction to publish");
  segmentAuction = centralModule.getSegmentAuctionInfo();
  ROS_INFO("Computed the segment auction to publish");
  ros::Duration lastGvdTime = gvdTime;
  gvdTime = (ros::Time::now() - lastGvdStart);
  gvdTimeIncrement = max(gvdTimeIncrement, gvdTime - lastGvdTime);
  // set markers for rviz gvd visualization
  drawGvd(segmentAuction, mapInfo);
  // log gvd time
  if (LOG >= 2) {
    ros::Duration currentTime = ros::Time::now() - firstAuction;
    // string data = to_string(current_time.toSec())+" "+to_string(gvd_time.toSec());
    string time = to_string(currentTime.toSec());
    string coveragePer = to_string(((float)centralModule.cell_count / MAP_SIZE) * 100);

    if (coverageFileLog.empty()) {
      coverageFileLog = "covarage";
      coverageFileLog = LOG_FILE_PATH + mapName + "_" + coverageFileLog +
                          to_string(centralModule.getNumRobots());
    }

    string data = coveragePer + " " + to_string(gvdTime.toSec());
    if (gvdFileLog.empty()) {
      gvdFileLog = "tiemposGVD";
      gvdFileLog = LOG_FILE_PATH + mapName + "_" + gvdFileLog + to_string(centralModule.getNumRobots());
    }

    log_data(data, gvdFileLog);

    ros::Duration increment = (gvdTime - lastGvdTime);
    // data = to_string(current_time.toSec())+"  "+to_string(increment.toSec());
    data = coveragePer + "  " + to_string(increment.toSec());
    if (incrementGvdFileLog.empty()) {
      incrementGvdFileLog = "incrementoGVD";
      incrementGvdFileLog = LOG_FILE_PATH + mapName + "_" + incrementGvdFileLog +
                               to_string(centralModule.getNumRobots());
    }

    log_data(data, incrementGvdFileLog);

    // gnuplot p;
    // p.graph_file(gvd_file_log, "Cubrimiento del mapa(%)", "Tiempo de GvdGraph(s)");
    // p.graph_file(increment_gvd_file_log, "Cubrimiento del mapa(%)", "Incremento de tiempo de
    // GvdGraph(s)");
  }

  // Send auction info: Graph and criticals info

  requests = 0;
  centralModule.setEstado(WaitingFirstBid);

  segmentAuctionPub.publish(segmentAuction);
  ROS_INFO("Segment Auction published");
  /* ROS_INFO("Segment Auction disabled"); */
}

void resolveAuction() {
  centralModule.setEstado(Resolving);
  ros::Duration resolutionTime = ros::Time::now() - lastAuctionStart;
  ROS_INFO("Auction Resolution Start, expected %f, real %f", AuctionResolutionTimeout.toSec(),
           resolutionTime.toSec());
  AuctionResolutionTimeout =
      max(ros::Duration(0.5) + resolutionTime * 2, AuctionResolutionTimeout);
  auctionResolutionTimer.setPeriod(AuctionResolutionTimeout);

  succesfulBids = 0;

  // set up
  visualization_msgs::Marker::_points_type points;
  map_info_type mapInfo = centralModule.getMap().info;

  boost::unordered_map<string, pgmappingcooperativo::SegmentAssignment> assignment =
      centralModule.assignSegment();
  assignedRobots = assignment.size();

  float maxEstimatedTime = 0;
  float minEstimatedTime = FLT_MAX;

  centralModule.setEstado(WaitingAuction);

  for (auto it = assignment.begin(); it != assignment.end(); it++) {
    pgmappingcooperativo::SegmentAssignment sa = it->second;
    string robot = it->first;

    // publish
    segmentAssignmentPubs[it->first].publish(it->second);

    // obtain costs
    float estimatedTime =
        (centralModule.segment_bids[robot][p2d_to_pos(sa.segment)]) / (ROBOT_SPEED);

    maxEstimatedTime = max(maxEstimatedTime, estimatedTime);
    minEstimatedTime = max(minEstimatedTime, estimatedTime);

    for (auto it = sa.frontiers.begin(); it != sa.frontiers.end(); it++) {
      points.push_back(p2d_to_p3d(*it, 0.1, mapInfo));
    }
  }

  for (auto it = assignment.begin(); it != assignment.end(); it++) {
    pgmappingcooperativo::SegmentAssignment sa = it->second;
    string robot = it->first;
    float estimatedTime =
        (centralModule.segment_bids[robot][p2d_to_pos(sa.segment)]) / (ROBOT_SPEED);
    if (estimatedTime >
        minEstimatedTime + (gvdTime + gvdTimeIncrement + auctionStartDelayTimeout).toSec()) {
      assignedRobots--;
    }
  }

  auctionStartTimeout = (gvdTime + gvdTimeIncrement + auctionStartDelayTimeout);
  auctionStartTimer.setPeriod(auctionStartTimeout);

  ROS_INFO("gvd time = %f , gvd estimated time = %f, max estimated time %f", (gvdTime).toSec(),
           (gvdTime + gvdTimeIncrement + auctionStartDelayTimeout).toSec(), maxEstimatedTime);

  std_msgs::ColorRGBA green;
  green.g = 1.0f;
  green.a = 1.0f;
  markerPub.publish(mark_points("Frontiers", points, green));
  ROS_INFO("Auction resoved, %d robots assigned", assignedRobots);

  ros::Duration currentTime = ros::Time::now() - firstAuction;
  ROS_INFO("Elapsed time: %f", currentTime.toSec());
}

/*
 *  Timer Functions
 */

// bool pending = false;
/* cuando: auctionResolutionTimer AuctionResolutionTimeout */
/* que: ejecucion subasta (asignacion de tareas) y publicacion de resultados */

void auctionResolutionTimerRoutine(const ros::TimerEvent&) {
  auctionResolutionTimer.stop();
  if (centralModule.getEstado() == WaitingBids) {
    resolveAuction();
  } else {
    ROS_INFO("WARNING: auction AuctionResolutionTimeout with no bids");
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
 *  Call Backs
 */

/* cuando: un robot te pide un objetivo */
/* que: inicia una subasta */
void requestObjectiveCallBack(const std_msgs::StringConstPtr& msg) {
  if (centralModule.getEstado() != WaitingAuction) {
    ROS_INFO("Auction request from ignored, already one on course");
  } else {
    requests++;
    if (requests >= assignedRobots) {
      // ROS_INFO("Auction request successful, it is the last one starting the auction");
      auctionStartTimer.stop();
      auctionStartDelayTimer.start();  // delay to wait for the map
      // startAuction();
    } else {
      // ROS_INFO("Auction request successful, starting timer, %d robots
      // left",assigned_robots-requests);

      auctionStartTimer.start();
    }
  }
}

int maps = 0;
bool first = true;
// When: A new map arrives
// What: Update map
void mapMergedCallBack(const pgmappingcooperativo::mapMergedInfoConstPtr& msg) {
  centralModule.updateMap(msg);
  maps++;
  if (first && maps >= 15){
    firstAuction = ros::Time::now();
    first = false;
    startAuction();
  }
}

void endCallBack(const std_msgs::StringConstPtr& msg) {
  endFlag = msg->data.compare(endMsg) == 0;
  if (endFlag) {
    // log data
    if (LOG > 0) {
      ros::Duration currentTime = ros::Time::now() - firstAuction;
      string robotCount = to_string(centralModule.getNumRobots());
      string time = to_string(currentTime.toSec());

      string data = "Cantidad Robots: " + robotCount + "\n";
      data += "Tiempo de ejecucion: " + time;
      string ejecucionFileLog = "tiemposEjecucion";
      ejecucionFileLog = LOG_FILE_PATH + mapName + "_" + ejecucionFileLog;
      log_data(data, ejecucionFileLog);

      ejecucionFileLog = "tiemposEjecucionG";
      ejecucionFileLog = LOG_FILE_PATH + mapName + "_" + ejecucionFileLog;
      string gData = robotCount + "  " + time;
      log_data(gData, ejecucionFileLog);
      log_data(time, (ejecucionFileLog + robotCount));

      gnuplot p;
      p.graph_file(ejecucionFileLog, "Numero de robots", "Tiempo de ejecucion");
      if (LOG > 1) {
        p.graph_file(gvdFileLog, "Cubrimiento del mapa(%)", "Tiempo de GvdGraph(s)");
        p.graph_file(incrementGvdFileLog, "Cubrimiento del mapa(%)", "Incremento de tiempo de GvdGraph(s)");
      }
    }

    ROS_INFO("Exploration completed, shutting down ros...");
    ros::Duration(2.5).sleep();
    ros::shutdown();
  }
}

void segmentBidCallBack(const pgmappingcooperativo::SegmentBidConstPtr& msg, string name) {
  if (!is_elem(centralModule.getEstado(), {WaitingFirstBid, WaitingBids})) return;

  bool successful = centralModule.saveSegmentBid(*msg, name);

  if (!successful) {
    ROS_DEBUG_STREAM("Got OLD segment bid from "<<name.c_str());
    return;
  }

  ROS_INFO_STREAM("Got segment bid from "<<name.c_str()<<" , id "<<msg->id);

  succesfulBids++;

  if (centralModule.getEstado() == WaitingFirstBid) {
    ROS_DEBUG_STREAM("Is the first one, starting auctionResolutionTimer");
    centralModule.setEstado(WaitingBids);
    lastAuctionStart = ros::Time::now();
    auctionResolutionTimer.start();
  }

  if (succesfulBids == centralModule.getNumRobots()) {
    ROS_DEBUG_STREAM("its the last one, stopping auctionResolutionTimer and starting resultion");
    auctionResolutionTimer.stop();
    resolveAuction();
  }
}

int main(int argc, char* argv[]) {
  // Change log level to debug
  /* if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) { */
  /*    ros::console::notifyLoggerLevelsChanged(); */
  /* } */

  // Init node
  ros::init(argc, argv, "central_module");
  ros::NodeHandle n;
  centralModule = CentralModule();

  // Load params
  /// GVD lib
  n.param<int>("/gvd_connectivity_method", GvdConfig::get()->connectivityMethod, GvdConfig::get()->connectivityMethod);
  n.param<int>("/gvd_vertex_simplification_method", GvdConfig::get()->vertexSimplificationMethod, GvdConfig::get()->vertexSimplificationMethod);
  n.param<int>("/gvd_edge_simplification_method", GvdConfig::get()->edgeSimplificationMethod, GvdConfig::get()->edgeSimplificationMethod);
  n.param<int>("/gvd_edge_simplification_allow_vertex_removal", GvdConfig::get()->edgeSimplificationAllowVertexRemoval, GvdConfig::get()->edgeSimplificationAllowVertexRemoval);
  n.param<int>("/critical_conditiont_min", GvdConfig::get()->criticalConditiontMin, GvdConfig::get()->criticalConditiontMin);

  /// Current scenario
  int startingRobotNumber;
  n.param<int>("/starting_robot_number", startingRobotNumber, STARTING_ROBOT_NUMBER);
  centralModule.setNumRobots(startingRobotNumber);

  n.param<string>("/map_name", mapName, "");

  // Initilize timers
  auctionResolutionTimer = n.createTimer(AuctionResolutionTimeout, auctionResolutionTimerRoutine, true, false);
  auctionStartTimer      = n.createTimer(auctionStartTimeout, auctionStartTimerRoutine          , true, false);
  auctionStartDelayTimer = n.createTimer(auctionStartDelayTimeout, auctionStartDelayTimerRoutine, true, false);

  // Initilize Publishers
  takeObjPub        = n.advertise<pgmappingcooperativo::takeobjetive>("/take_obj", 1);
  objetivePub       = n.advertise<pgmappingcooperativo::asignacion>("/objetive", 1);
  objPub            = n.advertise<nav_msgs::OccupancyGrid>("/debbi", 1);
  markerPub         = n.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
  segmentAuctionPub = n.advertise<pgmappingcooperativo::SegmentAuction>("/segment_auction", 1);

  // Initilize Subscribers
  mapMergedSub       = n.subscribe<pgmappingcooperativo::mapMergedInfo>("/map_merged", 1, mapMergedCallBack);
  endSub              = n.subscribe("/end", 1, endCallBack);
  requestObjetiveSub = n.subscribe<std_msgs::String>("/request_objetive", 1, &requestObjectiveCallBack);

  // Initilize Publishers/Subscribers for each robot
  /// Wait for the robots to be ready
  ros::Rate loopRate(1);
  while (true) {
    int contP3dx = 0;
    ros::master::V_TopicInfo topicInfos;
    ros::master::getTopics(topicInfos);
    for (ros::master::TopicInfo& publishedTopic : topicInfos) {
      if (publishedTopic.name.find("/pose") != string::npos) {
        contP3dx++;
      }
    }

    int remaining = centralModule.getNumRobots() - contP3dx;

    ROS_INFO_STREAM("Waiting for "<<remaining<<" out of "<<centralModule.getNumRobots()<<" robots");

    if(remaining==0) break;

    loopRate.sleep();
  }

  /// Subscribe to robot specific topics
  ros::master::V_TopicInfo topicInfos;
  ros::master::getTopics(topicInfos);
  for (ros::master::TopicInfo& publishedTopic : topicInfos) {
    if (publishedTopic.name.find("/pose") != string::npos) {
      string topicName = publishedTopic.name;                                   // name = "/robot_name/..."
      string robotName = topicName.erase(0, 1).substr(0, topicName.find('/')); // name = "robot_name"

      segmentBidSubs[robotName] = n.subscribe<pgmappingcooperativo::SegmentBid>("/"+robotName+"/segment_bid", 1, boost::bind(&segmentBidCallBack, _1, robotName));
      segmentAssignmentPubs[robotName] = n.advertise<pgmappingcooperativo::SegmentAssignment>("/"+robotName+"/segment_assigment", 1);

      ROS_DEBUG_STREAM("Extracted `robot_name = "<<robotName<<"` from `topic_name = "<<publishedTopic.name<<"`");
    }
  }

  ROS_INFO_STREAM("Initilized");

  ros::spin();

  return 0;
}
