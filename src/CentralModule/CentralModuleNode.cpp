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


// Rviz params

/// Markers used to represent cells
unsigned int cellMarkerType = RvizHelper::CUBE_LIST;

/// Markers dimensions
float cubeHeight = 0.02;

/// Layers (planes orthogonal to the z axis)
//// Separation between each layer
float layerSeparation = cubeHeight;

//// Assign markers to layers
float frontierZ      = 5*layerSeparation;

float gvdVertexZ     = 4*layerSeparation;

float criticalPointZ = 3*layerSeparation;

float gvdEdgeZ       = 2*layerSeparation;

float criticalLineZ  = 1*layerSeparation;

///////////////
// Variables //
///////////////

// Topics
/// Subscribers
map<string, ros::Subscriber> bids;
map<string, ros::Subscriber> bidSubs;
ros::Subscriber endSub;
ros::Subscriber mapMergedSub;
ros::Subscriber requestObjetiveSub;

/// Publishers
ros::Publisher takeObjPub;
ros::Publisher objetivePub;
ros::Publisher miscMarkerPub;
ros::Publisher gvdMarkerPub;
ros::Publisher topoMapMarkerPub;
ros::Publisher auctionPub;
map<string, ros::Publisher> AssignmentPubs;

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

// Publishes marks to be visualized on rviz 
void setRvizMarks(pgmappingcooperativo::Auction sac, mapInfoType mapInfo) {
  // Gvd
  rvizHelper.topic = &gvdMarkerPub;

  /// mark edges
  RvizHelper::MarkerPoints edgesMarkerPoints;
  for (auto e : sac.gvd.edges) {
    edgesMarkerPoints.push_back(toPoint(e.from, mapInfo));
    edgesMarkerPoints.push_back(toPoint(e.to, mapInfo));
  }
  rvizHelper.color    = BLUE;
  rvizHelper.type     = RvizHelper::LINE_LIST;
  rvizHelper.scale    = makeVector3(centralModule.cellSize*0.25, centralModule.cellSize, 1);
  rvizHelper.position = makeVector3(0,0,gvdEdgeZ);
  rvizHelper.mark(edgesMarkerPoints, "gvd_edges");

  /// mark vertices
  rvizHelper.color    = BLUE;
  rvizHelper.type     = cellMarkerType;
  rvizHelper.scale    = makeVector3(centralModule.cellSize*0.6); rvizHelper.scale.z  = cubeHeight;
  rvizHelper.position = makeVector3(0,0,gvdVertexZ);
  rvizHelper.mark(toVecPoint3D(sac.gvd.vertices, mapInfo), "gvd_vertices");

  // Topological map
  rvizHelper.topic = &topoMapMarkerPub;

  /// segments
  rvizHelper.ns = "segments";

  //// delete previous segments
  rvizHelper.deleteMark(); 

  //// Mark segments and load frontiers
  rvizHelper.type     = RvizHelper::TRIANGLE_LIST;
  rvizHelper.scale    = makeVector3(centralModule.cellSize); rvizHelper.scale.z  = cubeHeight;
  rvizHelper.position = makeVector3(0);

  RvizHelper::MarkerPoints frontierMarkerPoints;
  for(auto it : centralModule.topoMap->segmenter->connectedComponents){
    int id = it.first;
    ConnectedComponents::ConectedComponent segment = it.second;

    rvizHelper.color    = getColor(id);

    RvizHelper::MarkerPoints segmentMarkerPoints;
    for (geometry_msgs::Point p : toMarkerPoint(segment.members, mapInfo)){ 
      float squareLength = centralModule.cellSize*1.05;
      p.x += squareLength/2.0;
      p.y += squareLength/2.0;
      geometry_msgs::Point uR = p;
      p.y -= squareLength;
      geometry_msgs::Point lR = p;
      p.x -= squareLength;
      geometry_msgs::Point lL = p;
      p.y += squareLength;
      geometry_msgs::Point uL = p;

      segmentMarkerPoints.push_back(uL);
      segmentMarkerPoints.push_back(lL);
      segmentMarkerPoints.push_back(lR);

      segmentMarkerPoints.push_back(lR);
      segmentMarkerPoints.push_back(uR);
      segmentMarkerPoints.push_back(uL);
    }
    rvizHelper.mark(segmentMarkerPoints, id);

    accum(frontierMarkerPoints,toMarkerPoint(segment.typeMembers[Frontier],mapInfo));

    ros::Duration(0.00001).sleep(); // small sleep because rviz skips markers otherwise
  }

  //// mark critical points and critical lines
  RvizHelper::MarkerPoints criticalMarkerPoints;
  RvizHelper::MarkerPoints criticalLinesMarkerPoints;
  for (auto it : centralModule.topoMap->criticalInfos) {
    Pos p = it.first;
    CriticalInfo criticalInfo = it.second;
    criticalMarkerPoints.push_back(toMarkerPoint(p, mapInfo));
    accum(criticalLinesMarkerPoints,toMarkerPoint(criticalInfo.criticalLines,mapInfo));
  }
  rvizHelper.type     = cellMarkerType;
  rvizHelper.scale    = makeVector3(centralModule.cellSize*0.75); rvizHelper.scale.z  = cubeHeight;

  rvizHelper.color    = YELLOW;
  rvizHelper.position = makeVector3(0,0,criticalPointZ);
  rvizHelper.mark(criticalMarkerPoints, "critical_vertices");

  rvizHelper.color    = makeColorRGBA(0.9); // light gray
  rvizHelper.position = makeVector3(0,0,criticalLineZ);
  rvizHelper.mark(criticalLinesMarkerPoints, "critical_lines");

  // misc
  rvizHelper.topic = &miscMarkerPub;

  /// Mark frontiers
  rvizHelper.color    = GREEN;
  rvizHelper.type     = cellMarkerType;
  rvizHelper.scale    = makeVector3(centralModule.cellSize*0.5); rvizHelper.scale.z  = cubeHeight;
  rvizHelper.position = makeVector3(0,0,frontierZ);
  rvizHelper.mark(frontierMarkerPoints, "frontiers");
}

///////////////////
// Aux Functions //
///////////////////

void startAuction() {
  // get aution info (calculate gvd + mesure time of calculation)
  ROS_INFO("Computing segment auction information");

  ros::Duration lastGvdTime = gvdTime;
  lastGvdStart = ros::Time::now();

  pgmappingcooperativo::Auction auction = centralModule.getAuctionInfo();

  gvdTime = (ros::Time::now() - lastGvdStart);
  gvdTimeIncrement = max(gvdTimeIncrement, gvdTime - lastGvdTime);

  // set markers for rviz
  setRvizMarks(auction, centralModule.occupancyGrid.info);

  // As the bid was started bids can be received
  /// Reset auction variables
  requests = 0;
  /// Change state
  centralModule.setState(WaitingFirstBid);

  // Send auction info so the bids can be calculated
  auctionPub.publish(auction);
  ROS_INFO("Segment auction information broadcasted");

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

}

void resolveAuction() {
  // Change state to reolving auction
  centralModule.setState(Resolving);

  // auction resolution start timeout (timeout to wait for robot bids)
  /// Show real auction resolution start time and the estimated time (aution resolution timeout)
  ros::Duration resolutionStartTime = ros::Time::now() - lastAuctionStart;
  ROS_INFO_STREAM("Auction resolution | real time "<<resolutionStartTime.toSec()<<" | max estimated time "<<AuctionResolutionTimeout.toSec());

  /// Set the auction resolution timeout
  AuctionResolutionTimeout = max(ros::Duration(0.5) + resolutionStartTime * 2, AuctionResolutionTimeout);
  auctionResolutionTimeoutTimer.setPeriod(AuctionResolutionTimeout,false);

  // Get the robot-segment assignment
  boost::unordered_map<string, pgmappingcooperativo::Assignment> assignment = centralModule.assign();
  expectedRobots = assignedRobots = assignment.size();

  // As the assignment was calculated a new auction can be started
  /// Reset auction variables
  succesfulBids = 0;
  /// Change state
  centralModule.setState(WaitingAuction);

  // Parse the robot-segment assignment
  float maxEstimatedTime = 0;       // max estimated task completion time
  float minEstimatedTime = FLT_MAX; // min estimated task completion time

  for (auto it : assignment) {
    pgmappingcooperativo::Assignment sa = it.second;
    string robot = it.first;

    // Let the robot know about it assigned segment
    AssignmentPubs[robot].publish(sa);

    // Calculate the estimated task completion time
    float estimatedTime = max(0.f,(centralModule.bids[robot][toPos(sa.frontier)]) / (centralModule.robotSpeed));

    maxEstimatedTime = max(maxEstimatedTime, estimatedTime);
    minEstimatedTime = min(minEstimatedTime, estimatedTime);
  }

  // Set the timeout to start the next auction
  auctionStartTimeout = gvdTime + gvdTimeIncrement + auctionStartDelayTimeout;
  auctionStartTimeoutTimer.setPeriod(auctionStartTimeout,false);

  /// Calculate the robots that are expected to complete the assigned task not long after the first auction request
  for (auto it : assignment) {
    pgmappingcooperativo::Assignment sa = it.second;
    string robot = it.first;

    float estimatedTime = max(0.f,(centralModule.bids[robot][toPos(sa.frontier)]) / (centralModule.robotSpeed));
    if (estimatedTime > minEstimatedTime + auctionStartTimeout.toSec()) {
      expectedRobots--;
    }
  }

  // Show info about the auction
  /// Show the last gvd construction the estimated and the max estimated time 
  ROS_INFO_STREAM("GVD | estimated time+map update delay"<<auctionStartTimeout.toSec()<<
                     " | first robot task completion time "<<minEstimatedTime + auctionStartTimeout.toSec()<<
                     " | last robot task completion estimated time "<<maxEstimatedTime);

  /// Show resolution
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

void bidCallBack(const pgmappingcooperativo::BidConstPtr& msg, string name) {
  if (!is_elem(centralModule.getEstado(), {WaitingFirstBid, WaitingBids})) return;

  bool successful = centralModule.saveBid(*msg, name);

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
  bool endFlag = msg->data.compare(endMsg) == 0;

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
  /* if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) { */
  /*    ros::console::notifyLoggerLevelsChanged(); */
  /* } */

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
  assert(n.param<float> ("/robot_sensor_range", centralModule.sensorRange, 0));
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
  miscMarkerPub     = n.advertise<visualization_msgs::Marker>("/misc_visualization_marker", 10);
  gvdMarkerPub      = n.advertise<visualization_msgs::Marker>("/gvd_visualization_marker", 10);
  topoMapMarkerPub  = n.advertise<visualization_msgs::Marker>("/topo_map_visualization_marker", 10);
  auctionPub        = n.advertise<pgmappingcooperativo::Auction>("/auction", 1);

  // Initilize Subscribers
  mapMergedSub       = n.subscribe<pgmappingcooperativo::mapMergedInfo>("/map_merged", 1, mapMergedCallBack);
  endSub             = n.subscribe("/end", 1, endCallBack);
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

      bidSubs[robotName] = n.subscribe<pgmappingcooperativo::Bid>("/"+robotName+"/bid", 1, boost::bind(&bidCallBack, _1, robotName));
      AssignmentPubs[robotName] = n.advertise<pgmappingcooperativo::Assignment>("/"+robotName+"/assigment", 1);

      ROS_DEBUG_STREAM("Extracted `robot_name = "<<robotName<<"` from `topic_name = "<<publishedTopic.name<<"`");
    }
  }

  // spin
  ROS_INFO_STREAM("Initilized");
  ros::spin();

  return 0;
}
