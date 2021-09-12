#include <string>
#include "CentralModule.h"
#include "../lib/GVD/src/GvdConfig.h"
#include "nav_msgs/OccupancyGrid.h"

using namespace std;

////////////////
// Parameters //
////////////////

// auctionStartTimeoutMode:
///   * -1 : disabled, no timeout, starts the auction immediately after the first robot request 
///   *  0 : disabled, no timeout, starts the auction immediately after a little timeout to wait for the map update of that first robot
///   *  1 : enabled, timeout , delay the auction start to wait for the robots expected to arrive soon (estimated with gvd construction time)
///          after the first robot request
///   *  2 : enabled, timeout , delay the auction start to wait for the robots expected to arrive soon (estimated with gvd construction time)
///          after the first robot request. Reset and decrease the delay on new requests.
int auctionStartTimeoutMode = -1;

// mapUpdateDelay: The expected delay of a map update to arrive from the robot to the central module
float mapUpdateDelay = 2;

/////////////////
// Rviz params //
/////////////////

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
map<string, ros::Subscriber> bidSubs;
ros::Subscriber endSub;
ros::Subscriber mapSub;
ros::Subscriber mapUpdateSub;
ros::Subscriber requestObjetiveSub;

/// Publishers
ros::Publisher miscMarkerPub;
ros::Publisher gvdMarkerPub;
ros::Publisher topoMapMarkerPub;
ros::Publisher mapMarkerPub;
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
void setRvizMarks(Auction& auction, mapInfoType mapInfo) {

  float cellSize = mapInfo.resolution;

  // Gvd
  Gvd& gvd = *centralModule.topoMap->gvd;
  GvdGraph& gvdGraph = *centralModule.topoMap->gvd->graphGvd;

  rvizHelper.topic = &gvdMarkerPub;

  /// get edges and vertices (vertices are disabled) marker points 
  RvizHelper::MarkerPoints edgesMarkerPoints;
  RvizHelper::MarkerPoints connectivityEdgesMarkerPoints;
  /* RvizHelper::MarkerPoints verticesMarkerPoints; */
  /* RvizHelper::MarkerPoints connectivityVerticesMarkerPoints; */
  for (GvdGraph::Vertex v : gvdGraph) {
    Pos vPos = gvdGraph[v].p;
    bool isVPosConnectivityAux = gvd.isConnectivityAux(vPos);
    /* if(isVPosConnectivityAux){ */
    /*   connectivityVerticesMarkerPoints.push_back(toPoint(vPos,mapInfo)); */
    /* }else{ */
    /*   verticesMarkerPoints.push_back(toPoint(vPos,mapInfo)); */
    /* } */
    for (GvdGraph::Vertex nv : gvdGraph.adj(v)) {
      Pos nvPos = gvdGraph[nv].p;
      if(isVPosConnectivityAux || gvd.isConnectivityAux(nvPos)){
        connectivityEdgesMarkerPoints.push_back(toPoint(vPos,mapInfo));
        connectivityEdgesMarkerPoints.push_back(toPoint(nvPos,mapInfo));
      }else{
        edgesMarkerPoints.push_back(toPoint(vPos,mapInfo));
        edgesMarkerPoints.push_back(toPoint(nvPos,mapInfo));
      }
    }
  }

  // mark edges
  rvizHelper.type     = RvizHelper::LINE_LIST;
  rvizHelper.scale    = makeVector3(cellSize*0.25, cellSize, 1);
  rvizHelper.position = makeVector3(0,0,gvdEdgeZ);

  rvizHelper.color = BLUE;
  rvizHelper.mark(edgesMarkerPoints, "gvd_edges");
  rvizHelper.color = makeColorRGBA(0.2, 0.505, 1,0.5);
  rvizHelper.mark(connectivityEdgesMarkerPoints, "gvd_conntivity_aux_edges");

  /// mark vertices (DISABLED)
  /* rvizHelper.type     = cellMarkerType; */
  /* rvizHelper.scale    = makeVector3(cellSize*0.6); rvizHelper.scale.z  = cubeHeight; */
  /* rvizHelper.position = makeVector3(0,0,gvdVertexZ); */

  /* rvizHelper.color    = BLUE; */
  /* rvizHelper.mark(verticesMarkerPoints, "gvd_vertices"); */
  /* rvizHelper.color    = makeColorRGBA(0.2, 0.505, 1,0.5); */
  /* rvizHelper.mark(connectivityVerticesMarkerPoints, "gvd_connectivity_aux_vertices"); */

  // Topological map
  rvizHelper.topic = &topoMapMarkerPub;

  /// segments
  rvizHelper.ns = "segments";

  //// delete previous segments
  rvizHelper.deleteMark(); 

  //// Mark segments and load frontiers
  rvizHelper.type     = RvizHelper::TRIANGLE_LIST;
  rvizHelper.scale    = makeVector3(1); rvizHelper.scale.z  = cubeHeight;
  rvizHelper.position = makeVector3(0);

  RvizHelper::MarkerPoints frontierMarkerPoints;
  for(auto it : centralModule.topoMap->segmenter->connectedComponents){
    int id = it.first;
    ConnectedComponents::Component segment = it.second;

    rvizHelper.color    = getColor(id);

    RvizHelper::MarkerPoints segmentMarkerPoints;
    for (Point p : toMarkerPoint(segment.members, mapInfo)){ 
      float squareLength = cellSize;
      p.x += squareLength/2.0;
      p.y += squareLength/2.0;
      Point uR = p;
      p.y -= squareLength;
      Point lR = p;
      p.x -= squareLength;
      Point lL = p;
      p.y += squareLength;
      Point uL = p;

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
  rvizHelper.scale    = makeVector3(cellSize*0.75); rvizHelper.scale.z  = cubeHeight;

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
  rvizHelper.scale    = makeVector3(cellSize*0.5); rvizHelper.scale.z  = cubeHeight;
  rvizHelper.position = makeVector3(0,0,frontierZ);
  rvizHelper.mark(frontierMarkerPoints, "frontiers");

  /// map not an rviz marker but used just to mark
  mapMarkerPub.publish(centralModule.occupancyGrid);
}

///////////////////
// Aux Functions //
///////////////////

void startAuction() {
  // get aution info (calculate gvd + mesure time of calculation)
  ROS_INFO("Computing Auction information");

  ros::Duration lastGvdTime = gvdTime;
  lastGvdStart = ros::Time::now();

  Auction auction = centralModule.getAuctionInfo();

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
  ROS_INFO("Auction information broadcasted");

  // log gvd time
  if (centralModule.fileLogLevel >= 2) {
    ros::Duration currentTime = ros::Time::now() - firstAuction;
    string time = to_string(currentTime.toSec());
    string coveragePer = to_string(((float)centralModule.cellCount / centralModule.mapSize) * 100);

    if (coverageFileLog.empty()) {
      coverageFileLog = "covarage";
      coverageFileLog = centralModule.fileLogDir + centralModule.mapName + "_" + coverageFileLog + to_string(centralModule.robotNumber);
    }

    string data = coveragePer + " " + to_string(gvdTime.toSec());
    if (gvdFileLog.empty()) {
      gvdFileLog = "tiemposGVD";
      gvdFileLog = centralModule.fileLogDir + centralModule.mapName + "_" + gvdFileLog + to_string(centralModule.robotNumber);
    }

    log_data(data, gvdFileLog);

    ros::Duration increment = (gvdTime - lastGvdTime);
    data = to_string(currentTime.toSec())+"  "+to_string(increment.toSec());
    data = coveragePer + "  " + to_string(increment.toSec());
    if (incrementGvdFileLog.empty()) {
      incrementGvdFileLog = "incrementoGVD";
      incrementGvdFileLog = centralModule.fileLogDir + centralModule.mapName + "_" + incrementGvdFileLog + to_string(centralModule.robotNumber);
    }

    log_data(data, incrementGvdFileLog);

    gnuplot p;
    p.graph_file(gvdFileLog, "Cubrimiento del map(%)", "Tiempo de GvdGraph(s)");
    p.graph_file(incrementGvdFileLog, "Cubrimiento del map(%)", "Incremento de tiempo de GvdGraph(s)");
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

  // Get the robot-frontier assignment
  boost::unordered_map<string, Assignment> assignment = centralModule.assign();
  expectedRobots = assignedRobots = assignment.size();

  // As the assignment was calculated a new auction can be started
  /// Reset auction variables
  succesfulBids = 0;
  /// Change state
  centralModule.setState(WaitingAuction);

  // Parse the robot-frontier assignment
  float maxEstimatedTime = 0;       // max estimated task completion time
  float minEstimatedTime = FLT_MAX; // min estimated task completion time

  for (auto it : assignment) {
    Assignment sa = it.second;
    string robot = it.first;

    // Let the robot know about it assigned frontier
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
    Assignment sa = it.second;
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
  if (centralModule.getState() == WaitingBids) {
    resolveAuction();
  } else {
    ROS_WARN_STREAM("Auction AuctionResolutionTimeout with no bids");
  }
}

void auctionStartTimeoutTimerRoutine(const ros::TimerEvent&) {
  auctionStartTimeoutTimer.stop();
  ROS_DEBUG("Auction triggered BECOUSE of timeout");
  startAuction();
}

void auctionStartDelayTimerRoutine(const ros::TimerEvent&) {
  auctionStartDelayTimer.stop();
  ROS_DEBUG("Auction triggered BEFORE timeout");
  startAuction();
}

///////////////
// CallBacks //
///////////////

bool first = true;
void mapCallBack(const OccupancyGridConstPtr& msg) {
  centralModule.updateMap(*msg);
  if (first){
    firstAuction = ros::Time::now();
    first = false;
    startAuction();
  }
}

void mapUpdateCallBack(const OccupancyGridUpdateConstPtr& msg) {
  centralModule.updateMap(msg);
}

void requestObjectiveCallBack(const std_msgs::StringConstPtr& msg) {
  if (centralModule.getState() != WaitingAuction) {
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

void bidCallBack(const BidConstPtr& msg, string name) {
  if (!is_elem(centralModule.getState(), {WaitingFirstBid, WaitingBids})) return;

  bool successful = centralModule.saveBid(*msg, name);

  if (!successful) {
    ROS_DEBUG_STREAM("Bid ignored from "<<name.c_str()<<" (OLD)");
    return;
  }

  ROS_INFO_STREAM("Bid from "<<name.c_str()<<" , id "<<msg->id);

  succesfulBids++;

  if (centralModule.getState() == WaitingFirstBid) {
    ROS_DEBUG_STREAM("Is the first one, starting the auction resolution timeout");
    centralModule.setState(WaitingBids);
    lastAuctionStart = ros::Time::now();
    auctionResolutionTimeoutTimer.start(); // Start the timeout to wait for the other working robots
  }

  if (succesfulBids == centralModule.robotNumber) {
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
    string robotCount = to_string(centralModule.robotNumber);
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
      p.graph_file(gvdFileLog, "Cubrimiento del map(%)", "Tiempo de GvdGraph(s)");
      p.graph_file(incrementGvdFileLog, "Cubrimiento del map(%)", "Incremento de tiempo de GvdGraph(s)");
    }
  }

  ROS_INFO("Exploration completed, shutting down ros...");
  ros::Duration(2.5).sleep();
  ros::shutdown();
}

//////////
// main //
//////////

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
  float cell_size;
  FAIL_IFN(n.param<float>   ("/cell_size", cell_size, cell_size));

  FAIL_IFN(n.param<float> ("/robot_sensor_range", centralModule.sensorRange, 0));
  centralModule.sensorRange /= cell_size;

  FAIL_IFN(n.param<int>   ("/starting_robot_number", centralModule.robotNumber, 0));
  FAIL_IFN(n.param<string>("/map_name", centralModule.mapName, ""));
  FAIL_IFN(n.param<int>   ("/map_size", centralModule.mapSize, 0));
  FAIL_IFN(n.param<float> ("/robot_speed", centralModule.robotSpeed, 0));
  FAIL_IFN(n.param<string>("/file_log_dir", centralModule.fileLogDir, ""));
  FAIL_IFN(n.param<int>   ("/file_log_level", centralModule.fileLogLevel, 0));

  // Initilize timers
  auctionResolutionTimeoutTimer = n.createTimer(AuctionResolutionTimeout, auctionResolutionTimeoutTimerRoutine, true, false);
  auctionStartTimeoutTimer      = n.createTimer(auctionStartTimeout     , auctionStartTimeoutTimerRoutine     , true, false);

  auctionStartDelayTimeout.fromSec(mapUpdateDelay);
  auctionStartDelayTimer        = n.createTimer(auctionStartDelayTimeout, auctionStartDelayTimerRoutine       , true, false);

  // Initilize Publishers
  miscMarkerPub     = n.advertise<visualization_msgs::Marker>("/misc_visualization_marker", 10);
  gvdMarkerPub      = n.advertise<visualization_msgs::Marker>("/gvd_visualization_marker", 10);
  topoMapMarkerPub  = n.advertise<visualization_msgs::Marker>("/topo_map_visualization_marker", 10);
  mapMarkerPub      = n.advertise<OccupancyGrid>("/map_visualization_marker", 10); 
  auctionPub        = n.advertise<Auction>("/auction", 1);

  // Initilize Subscribers
  mapSub             = n.subscribe<OccupancyGrid>("/map", 1, mapCallBack);
  mapUpdateSub       = n.subscribe<OccupancyGridUpdate>("/map_update", 1, mapUpdateCallBack);
  endSub             = n.subscribe("/end", 1, endCallBack);
  requestObjetiveSub = n.subscribe<std_msgs::String>("/request_objetive", 1, &requestObjectiveCallBack);

  // Initilize Publishers/Subscribers for each robot

  /// Wait for the robots to be ready
  ros::Rate loopRate(1);
  while (true) {
    int readyRobots = 0;

    ros::master::V_TopicInfo topicInfos;
    ros::master::getTopics(topicInfos);
    for (ros::master::TopicInfo& publishedTopic : topicInfos) {
      if (publishedTopic.name.find("/odom") != string::npos) {
        readyRobots++;
      }
    }

    int remaining = centralModule.robotNumber - readyRobots;

    ROS_INFO_STREAM("Waiting for "<<remaining<<" out of "<<centralModule.robotNumber<<" robots");

    if(remaining==0) break;

    loopRate.sleep();
  }

  /// Subscribe to robot specific topics
  ros::master::V_TopicInfo topicInfos;
  ros::master::getTopics(topicInfos);
  for (ros::master::TopicInfo& publishedTopic : topicInfos) {
    if (publishedTopic.name.find("/odom") != string::npos) {
      string topicName = publishedTopic.name;                                  // name = "/robot_name/..."
      string robotName = topicName.erase(0, 1).substr(0, topicName.find('/')); // name = "robot_name"

      bidSubs[robotName] = n.subscribe<Bid>("/"+robotName+"/bid", 1, boost::bind(&bidCallBack, _1, robotName));
      AssignmentPubs[robotName] = n.advertise<Assignment>("/"+robotName+"/assigment", 1);

      ROS_DEBUG_STREAM("Extracted `robot_name = "<<robotName<<"` from `topic_name = "<<publishedTopic.name<<"`");
    }
  }

  // spin
  ROS_INFO_STREAM("Initilized");
  ros::spin();

  return 0;
}
