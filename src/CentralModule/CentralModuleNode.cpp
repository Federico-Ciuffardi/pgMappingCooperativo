#include <string>
#include "CentralModule.h"
#include "../lib/GVD/src/GvdConfig.h"

using namespace std;

////////////////
// Parameters //
////////////////

/// auctionStartTimeoutMode:
///   * -1 : disabled, no timeout, starts the auction immediately after the first robot request 
///   *  0 : disabled, no timeout, starts the auction immediately after a little timeout to wait for the map update of that first robot
///   *  1 : enabled, timeout , delay the auction start to wait for the robots expected to arrive soon (estimated with gvd construction time)
///          after the first robot request
///   *  2 : enabled, timeout , delay the auction start to wait for the robots expected to arrive soon (estimated with gvd construction time)
///          after the first robot request. Reset and decrease the delay on new requests.
int auctionStartTimeoutMode = -1;

/// mapUpdateDelay: The expected delay of a map update to arrive from the robot to the central module
float mapUpdateDelay = 2;

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
ros::Publisher miscMarkerPub;
ros::Publisher gvdMarkerPub;
ros::Publisher topoMapMarkerPub;
ros::Publisher segmentAuctionPub;
map<string, ros::Publisher> segmentAssignmentPubs;

// timers 
ros::Timer auctionResolutionTimeoutTimer;
ros::Duration AuctionResolutionTimeout(0.1); // first AuctionResolutionTimeout

ros::Timer auctionStartDelayTimer;
ros::Duration auctionStartDelayTimeout;

ros::Timer auctionStartTimeoutTimer;
ros::Duration auctionStartTimeout;

// others

CentralModule centralModule;
RvizHelper rvizHelper;

int succesfulBids  = 0;
int assignedRobots = 0;
int expectedRobots = 0;
int requests       = 0;

bool endFlag = false;

ros::Time lastAuctionStart;
ros::Time lastGvdStart;
ros::Time firstAuction;

ros::Duration gvdTime;
ros::Duration gvdTimeIncrement(0);

string endMsg = "END";

string gvdFileLog;
string incrementGvdFileLog;
string coverageFileLog;

////////////////
// Rviz marks //
////////////////

float cubeHeight = 0.02;
float layerSeparation = 0.175;
// Publishes marks corresponding to the gvd to be visualized on rviz 
static void drawGvd(pgmappingcooperativo::SegmentAuction sac, mapInfoType mapInfo) {


  // gvd
  rvizHelper.topic = &gvdMarkerPub;

  /// edges
  RvizHelper::MarkerPoints edgesMarkerPoints;
  for (auto e : sac.gvd.edges) {
    edgesMarkerPoints.push_back(p2d_to_p3d(e.from, mapInfo));
    edgesMarkerPoints.push_back(p2d_to_p3d(e.to, mapInfo));
  }
  rvizHelper.color    = BLUE;
  rvizHelper.type     = RvizHelper::LINE_LIST;
  rvizHelper.scale    = makeVector3(centralModule.cellSize*0.2, centralModule.cellSize, 1);
  rvizHelper.position = makeVector3(0,0,3*layerSeparation);
  rvizHelper.mark(edgesMarkerPoints, "gvd_edges");

  /// vertices
  rvizHelper.color    = BLUE;
  rvizHelper.type     = RvizHelper::POINTS;
  rvizHelper.scale    = makeVector3(centralModule.cellSize*0.6); rvizHelper.scale.z  = cubeHeight;
  rvizHelper.position = makeVector3(0,0,1*layerSeparation);
  rvizHelper.mark(p2ds_to_p3ds(sac.gvd.vertices, mapInfo), "gvd_vertices");

  // topological map
  rvizHelper.topic = &topoMapMarkerPub;

  /// segments and frontiers

  /// delete previous segments
  rvizHelper.deleteMark("segments");

  /// set up segments and frontiers
  RvizHelper::MarkerPoints frontierMarkerPoints;
  for(auto it : centralModule.topoMap->segmenter->connectedComponents){
    int id = it.first;
    ConnectedComponents::ConectedComponent segment = it.second;

    rvizHelper.color    = getColor(id);
    rvizHelper.type     = RvizHelper::POINTS;
    rvizHelper.scale    = makeVector3(centralModule.cellSize*1); rvizHelper.scale.z  = cubeHeight;
    rvizHelper.position = makeVector3(0);
    rvizHelper.mark(toMarkerPoint(segment.members, mapInfo), "segments", id);

    accum(frontierMarkerPoints,toMarkerPoint(segment.typeMembers[Frontier],mapInfo));

    ros::Duration(0.00001).sleep(); // small sleep becouse rviz skips markers otherwise
  }

  /// set up critical points
  rvizHelper.color    = CYAN;
  rvizHelper.type     = RvizHelper::POINTS;
  rvizHelper.scale    = makeVector3(centralModule.cellSize*0.75); rvizHelper.scale.z  = cubeHeight;
  rvizHelper.position = makeVector3(0,0,2*layerSeparation);

  RvizHelper::MarkerPoints criticalMarkerPoints;
  for (auto it : centralModule.topoMap->criticalInfos) {
    Pos p = it.first;
    criticalMarkerPoints.push_back(toMarkerPoint(p, mapInfo));
  }
  rvizHelper.mark(criticalMarkerPoints, "gvd_critical_vertices");

  // topological map
  rvizHelper.topic = &miscMarkerPub;

  /// Mark frontiers
  rvizHelper.color    = GREEN;
  rvizHelper.type     = RvizHelper::POINTS;
  rvizHelper.scale    = makeVector3(centralModule.cellSize*0.5); rvizHelper.scale.z  = cubeHeight;
  rvizHelper.position = makeVector3(0,0,layerSeparation);
  rvizHelper.mark(frontierMarkerPoints, "frontiers");
}

///////////////////
// Aux Functions //
///////////////////

void startAuction() {
  // get aution info (calculate gvd + mesure time of calculation)

  lastGvdStart = ros::Time::now();

  ROS_INFO("Computing the segment auction to publish");
  pgmappingcooperativo::SegmentAuction segmentAuction;
  segmentAuction = centralModule.getSegmentAuctionInfo();
  ROS_INFO("Computed the segment auction to publish");

  ros::Duration lastGvdTime = gvdTime;
  gvdTime = (ros::Time::now() - lastGvdStart);
  gvdTimeIncrement = max(gvdTimeIncrement, gvdTime - lastGvdTime);

  // set markers for rviz gvd visualization
  drawGvd(segmentAuction, centralModule.getMap().info);

  // log gvd time
  if (centralModule.fileLogLevel >= 2) {
    ros::Duration currentTime = ros::Time::now() - firstAuction;
    string time = to_string(currentTime.toSec());
    string coveragePer = to_string(((float)centralModule.cellCount / centralModule.mapSize) * 100);

    if (coverageFileLog.empty()) {
      coverageFileLog = "covarage";
      coverageFileLog = centralModule.fileLogDir + centralModule.mapName + "_" + coverageFileLog + to_string(centralModule.getNumRobots());
    }

    string data = coveragePer + " " + to_string(gvdTime.toSec());
    if (gvdFileLog.empty()) {
      gvdFileLog = "tiemposGVD";
      gvdFileLog = centralModule.fileLogDir + centralModule.mapName + "_" + gvdFileLog + to_string(centralModule.getNumRobots());
    }

    log_data(data, gvdFileLog);

    ros::Duration increment = (gvdTime - lastGvdTime);
    data = to_string(currentTime.toSec())+"  "+to_string(increment.toSec());
    data = coveragePer + "  " + to_string(increment.toSec());
    if (incrementGvdFileLog.empty()) {
      incrementGvdFileLog = "incrementoGVD";
      incrementGvdFileLog = centralModule.fileLogDir + centralModule.mapName + "_" + incrementGvdFileLog + to_string(centralModule.getNumRobots());
    }

    log_data(data, incrementGvdFileLog);

    gnuplot p;
    p.graph_file(gvdFileLog, "Cubrimiento del mapa(%)", "Tiempo de GvdGraph(s)");
    p.graph_file(incrementGvdFileLog, "Cubrimiento del mapa(%)", "Incremento de tiempo de GvdGraph(s)");
  }

  // As the bid was started bids can be received
  /// Reset auction variables
  requests = 0;
  /// Change state
  centralModule.setState(WaitingFirstBid);

  // Send auction info so the bids can be calculated
  segmentAuctionPub.publish(segmentAuction);
  ROS_INFO("Segment Auction published");
}

void resolveAuction() {
  // Change state to reolving auction
  centralModule.setState(Resolving);

  // Show estimated auction resolution time (aution resolution timeout) and the real time
  ros::Duration resolutionTime = ros::Time::now() - lastAuctionStart;
  ROS_INFO_STREAM("Auction resolution | real time "<<resolutionTime.toSec()<<" | max estimated time "<<AuctionResolutionTimeout.toSec());

  // Set the auction resolution timeout
  AuctionResolutionTimeout = max(ros::Duration(0.5) + resolutionTime * 2, AuctionResolutionTimeout);
  auctionResolutionTimeoutTimer.setPeriod(AuctionResolutionTimeout,false);

  // Get the robot-segment assignment
  boost::unordered_map<string, pgmappingcooperativo::SegmentAssignment> assignment = centralModule.assignSegment();
  expectedRobots = assignedRobots = assignment.size();

  // As the assignment was calculated a new auction can be started
  /// Reset auction variables
  succesfulBids = 0;
  /// Change state
  centralModule.setState(WaitingAuction);

  // Parse the robot-segment assignment
  mapInfoType mapInfo = centralModule.getMap().info;

  float maxEstimatedTime = 0;
  float minEstimatedTime = FLT_MAX;

  for (auto it : assignment) {
    pgmappingcooperativo::SegmentAssignment sa = it.second;
    string robot = it.first;

    // Let the robot know about it assigned segment
    segmentAssignmentPubs[robot].publish(sa);

    // Calculate the estimated task completion time
    float estimatedTime = max(0.f,(centralModule.segmentBids[robot][p2d_to_pos(sa.segment)]) / (centralModule.robotSpeed));

    maxEstimatedTime = max(maxEstimatedTime, estimatedTime);
    minEstimatedTime = min(minEstimatedTime, estimatedTime);

  }

  // Set the timeout to start the next auction
  auctionStartTimeout = gvdTime + gvdTimeIncrement + auctionStartDelayTimeout;
  auctionStartTimeoutTimer.setPeriod(auctionStartTimeout,false);

  // Calculate the robots that are expected to complete the assigned task not long after the first auction request
  for (auto it : assignment) {
    pgmappingcooperativo::SegmentAssignment sa = it.second;
    string robot = it.first;

    float estimatedTime = max(0.f,(centralModule.segmentBids[robot][p2d_to_pos(sa.segment)]) / (centralModule.robotSpeed));
    if (estimatedTime > minEstimatedTime + auctionStartTimeout.toSec()) {
      expectedRobots--;
    }
  }

  // Show the last gvd construction the estimated and the max estimated time 
  ROS_INFO_STREAM("GVD | estimated time "<<auctionStartTimeout.toSec()<<" | first robot task completion time "<<minEstimatedTime + auctionStartTimeout.toSec()<<
                  " | last robot task completion estimated time "<<maxEstimatedTime);

  // Show auction result 
  ROS_INFO_STREAM("Auction resoved, robots assigned = "<<assignedRobots<<" | robots expected "<<expectedRobots);
}

/////////////////////
// Timer Functions //
/////////////////////

void auctionResolutionTimeoutTimerRoutine(const ros::TimerEvent&) {
  auctionResolutionTimeoutTimer.stop();
  if (centralModule.getEstado() == WaitingBids) {
    resolveAuction();
  } else {
    ROS_WARN_STREAM("Auction AuctionResolutionTimeout with no bids");
  }
}

void auctionStartTimeoutTimerRoutine(const ros::TimerEvent&) {
  auctionStartTimeoutTimer.stop();
  ROS_DEBUG("Segment Auction triggered BECOUSE of timeout");
  startAuction();
}

void auctionStartDelayTimerRoutine(const ros::TimerEvent&) {
  auctionStartDelayTimer.stop();
  ROS_DEBUG("Segment Auction triggered BEFORE timeout");
  startAuction();
}

///////////////
// CallBacks //
///////////////

int maps = 0;
bool first = true;
void mapMergedCallBack(const pgmappingcooperativo::mapMergedInfoConstPtr& msg) {
  centralModule.updateMap(msg);
  maps++;
  if (first && maps >= 15){
    firstAuction = ros::Time::now();
    first = false;
    startAuction();
  }
}

void requestObjectiveCallBack(const std_msgs::StringConstPtr& msg) {
  if (centralModule.getEstado() != WaitingAuction) {
    ROS_DEBUG_STREAM("Auction request ignored, already one on course");
    return;
  }

  if (auctionStartTimeoutMode == -1){
    ROS_INFO_STREAM("Auction request successful stating auction");
    startAuction();
    return;
  }

  requests++;

  ROS_INFO_STREAM("Auction request successful, request "<<requests<<"/"<<expectedRobots<<" (arrived/expected)");

  if (requests >= expectedRobots) {
    auctionStartTimeoutTimer.stop(); // Stop the timeout as it was not necessary

    // Delay to wait for the map update of the last robot to arrive to its objective (reset)
    ROS_DEBUG_STREAM("Start auction after the map delay "<<auctionStartDelayTimeout.toSec());
    auctionStartDelayTimer.stop();
    auctionStartDelayTimer.start();  
  } else {
    switch (auctionStartTimeoutMode) {
      case  0:
        // Delay to wait for the map update of the last robot to arrive to its objective (reset)
        ROS_DEBUG_STREAM("Start auction after the map delay "<<auctionStartDelayTimeout.toSec());
        auctionStartDelayTimer.stop(); 
        auctionStartDelayTimer.start();  
        break;
      case 1:
        // Start the timeout to wait for the other expected robots (does not reset)
        ROS_DEBUG_STREAM_COND(requests == 1,"Starting the auction timeout, delay = "<<auctionStartTimeout.toSec());
        auctionStartTimeoutTimer.start(); 
        break;
      case 2:
        // Modify timeout if more than 1 request arrived
        if(requests > 1){
          auctionStartTimeout = auctionStartTimeout*(1.0/requests);
          auctionStartTimeoutTimer.setPeriod(auctionStartTimeout);
        }
        ROS_DEBUG_STREAM("Starting/Restarting the auction timeout, delay = "<<auctionStartTimeout.toSec());
        // Restart the timeout to wait for the other expected robots 
        auctionStartTimeoutTimer.stop(); 
        auctionStartTimeoutTimer.start(); 
        break;
    }
  }
}

void segmentBidCallBack(const pgmappingcooperativo::SegmentBidConstPtr& msg, string name) {
  if (!is_elem(centralModule.getEstado(), {WaitingFirstBid, WaitingBids})) return;

  bool successful = centralModule.saveSegmentBid(*msg, name);

  if (!successful) {
    ROS_DEBUG_STREAM("Segment bid ignored from "<<name.c_str()<<" (OLD)");
    return;
  }

  ROS_INFO_STREAM("Segment bid from "<<name.c_str()<<" , id "<<msg->id);

  succesfulBids++;

  if (centralModule.getEstado() == WaitingFirstBid) {
    ROS_DEBUG_STREAM("Is the first one, starting the auction resolution timeout");
    centralModule.setState(WaitingBids);
    lastAuctionStart = ros::Time::now();
    auctionResolutionTimeoutTimer.start(); // Start the timeout to wait for the other working robots
  }

  if (succesfulBids == centralModule.getNumRobots()) {
    ROS_DEBUG_STREAM("Is the last one, starting the auction resultion before timeout");
    auctionResolutionTimeoutTimer.stop(); // Stop the timeout as it was not necessary
    resolveAuction();                     // Resolve the auction
  }
}

void endCallBack(const std_msgs::StringConstPtr& msg) {
  endFlag = msg->data.compare(endMsg) == 0;

  if (!endFlag) return;

  // log data
  if (centralModule.fileLogLevel > 0) {
    ros::Duration currentTime = ros::Time::now() - firstAuction;
    string robotCount = to_string(centralModule.getNumRobots());
    string time = to_string(currentTime.toSec());

    string data = "Cantidad Robots: " + robotCount + "\n";
    data += "Tiempo de ejecucion: " + time;
    string ejecucionFileLog = "tiemposEjecucion";
    ejecucionFileLog = centralModule.fileLogDir + centralModule.mapName + "_" + ejecucionFileLog;
    log_data(data, ejecucionFileLog);

    ejecucionFileLog = "tiemposEjecucionG";
    ejecucionFileLog = centralModule.fileLogDir + centralModule.mapName + "_" + ejecucionFileLog;
    string gData = robotCount + "  " + time;
    log_data(gData, ejecucionFileLog);
    log_data(time, (ejecucionFileLog + robotCount));

    gnuplot p;
    p.graph_file(ejecucionFileLog, "Numero de robots", "Tiempo de ejecucion");
    if (centralModule.fileLogLevel > 1) {
      p.graph_file(gvdFileLog, "Cubrimiento del mapa(%)", "Tiempo de GvdGraph(s)");
      p.graph_file(incrementGvdFileLog, "Cubrimiento del mapa(%)", "Incremento de tiempo de GvdGraph(s)");
    }
  }

  ROS_INFO("Exploration completed, shutting down ros...");
  ros::Duration(2.5).sleep();
  ros::shutdown();
}

int main(int argc, char* argv[]) {
  // Change log level to debug
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
     ros::console::notifyLoggerLevelsChanged();
  }

  // Init node
  ros::init(argc, argv, "central_module");
  ros::NodeHandle n;

  // Load params
  /// GVD lib (check ../lib/GVD/src/config.h)
  n.param<int>("/gvd_connectivity_method", GvdConfig::get()->connectivityMethod, GvdConfig::get()->connectivityMethod);
  n.param<int>("/gvd_vertex_simplification_method", GvdConfig::get()->vertexSimplificationMethod, GvdConfig::get()->vertexSimplificationMethod);
  n.param<int>("/gvd_edge_simplification_method", GvdConfig::get()->edgeSimplificationMethod, GvdConfig::get()->edgeSimplificationMethod);
  n.param<int>("/gvd_edge_simplification_allow_vertex_removal", GvdConfig::get()->edgeSimplificationAllowVertexRemoval, GvdConfig::get()->edgeSimplificationAllowVertexRemoval);
  n.param<int>("/critical_condition_min", GvdConfig::get()->criticalConditionMin, GvdConfig::get()->criticalConditionMin);

  /// Auction
  n.param<int>("/auction_start_timeout_mode", auctionStartTimeoutMode, auctionStartTimeoutMode);
  n.param<float>("/map_update_delay", mapUpdateDelay, mapUpdateDelay);

  // Frontier
  n.param<int>("/frontier_simplification_method", centralModule.frontierSimplificationMethod, centralModule.frontierSimplificationMethod);

  /// Global params
  assert(n.param<int>   ("/starting_robot_number", centralModule.robotNumber, 0));
  assert(n.param<string>("/map_name", centralModule.mapName, ""));
  assert(n.param<int>   ("/map_size", centralModule.mapSize, 0));
  assert(n.param<float> ("/cell_size", centralModule.cellSize, 0));
  assert(n.param<float> ("/robot_speed", centralModule.robotSpeed, 0));
  assert(n.param<float> ("/robot_sensor_range", centralModule.robotSpeed, 0));
  assert(n.param<string>("/file_log_dir", centralModule.fileLogDir, ""));
  assert(n.param<int>   ("/file_log_level", centralModule.fileLogLevel, 0));

  // Initilize timers
  auctionResolutionTimeoutTimer = n.createTimer(AuctionResolutionTimeout, auctionResolutionTimeoutTimerRoutine, true, false);
  auctionStartTimeoutTimer      = n.createTimer(auctionStartTimeout     , auctionStartTimeoutTimerRoutine     , true, false);

  auctionStartDelayTimeout.fromSec(mapUpdateDelay);
  auctionStartDelayTimer        = n.createTimer(auctionStartDelayTimeout, auctionStartDelayTimerRoutine       , true, false);

  // Initilize Publishers
  takeObjPub        = n.advertise<pgmappingcooperativo::takeobjetive>("/take_obj", 1);
  objetivePub       = n.advertise<pgmappingcooperativo::asignacion>("/objetive", 1);
  objPub            = n.advertise<nav_msgs::OccupancyGrid>("/debbi", 1);
  miscMarkerPub     = n.advertise<visualization_msgs::Marker>("/misc_visualization_marker", 10);
  gvdMarkerPub      = n.advertise<visualization_msgs::Marker>("/gvd_visualization_marker", 10);
  topoMapMarkerPub  = n.advertise<visualization_msgs::Marker>("/topo_map_visualization_marker", 10);
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
      string topicName = publishedTopic.name;                                  // name = "/robot_name/..."
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
