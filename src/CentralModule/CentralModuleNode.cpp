#include <string>
#include "CentralModule.h"
#include "pgmappingcooperativo/RobotReport.h"
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
///
///   if the time to contruct the bids is grater than 4 seconds all the modes act like the mode 2
int auctionStartTimeoutModeParam = -1;
int auctionStartTimeoutMode = auctionStartTimeoutModeParam;
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
map<string, ros::Subscriber> robotReportSubs;
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
ros::Publisher endPub;
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

ros::Duration auctionInfoTime;
ros::Duration estimatedAuctionInfoTimeIncrement(0);

string endMsg = "END";

gnuplot gplot;

bool firstMap = true;
ros::Time firstMapTime;

boost::unordered_map<string,RobotReport> robotReports;

string auctionInfoTimeLog;         
string auctionInfoTimeIncrementLog;
string metersTraveledLog;         

bool endFlag = false;

////////////////
// Rviz marks //
////////////////

// Publishes marks to be visualized on rviz 
void setRvizMarks(Auction& auction, mapInfoType mapInfo) {

  float cellSize = mapInfo.resolution;

  // Gvd
  Gvd& gvd = *centralModule.topoMap->gvd;
  GvdGraph& gvdGraph = centralModule.topoMap->gvd->graphGvd;

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
    ConnectedComponents::Component &segment = it.second;

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
    CriticalInfo &criticalInfo = it.second;
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
  mapMarkerPub.publish(centralModule.map.occupancyGrid);
}

///////////////////
// Aux Functions //
///////////////////
void tryToShutdown(){
  // return if not meeting the ending conditions
  if(!endFlag || robotReports.size() < centralModule.robotNumber) return;

  for(auto it : robotReports){
    RobotReport robotReport = it.second;
    logAppend(metersTraveledLog, to_string(robotReport.metersTraveled));
  }

  gplot.graph_file(auctionInfoTimeLog, "Explored cells", "Time to get the auction information");
  gplot.graph_file(auctionInfoTimeIncrementLog, "Explored cells", "Time diference to get the auction information");

  std::system(("rosrun map_server map_saver --occ 50 --free 49 -f "+ centralModule.fileLogDir +"/map map:=map_visualization_marker &").c_str());
  ros::Duration(5).sleep();
  mapMarkerPub.publish(centralModule.map.occupancyGrid);
  ros::Duration(5).sleep();

  ROS_INFO("Shutting down ros...");
  ros::Duration(2.5).sleep();
  ros::shutdown();
}

void startAuction() {
  // get aution info (calculate gvd + mesure time of calculation)
  ROS_INFO("Computing Auction information");

  ros::Duration lastauctionInfoTime = auctionInfoTime;

  ros::Time auctionInfoStart = ros::Time::now();
  Auction auction = centralModule.getAuctionInfo();
  auctionInfoTime = (ros::Time::now() - auctionInfoStart);

  estimatedAuctionInfoTimeIncrement = max(estimatedAuctionInfoTimeIncrement, auctionInfoTime - lastauctionInfoTime);

  cout<<"debug :: ended getAuctionInfo"<<endl;

  if(auction.frontiers.empty()){
    ROS_INFO("No frontiers to explore");
    // Notify termination to other nodes, the end module won't do it becouse it will never reach the coverge needed to end
    // (no frontiers means no new cells explored)
    string explorationTime = to_string((ros::Time::now() - firstMapTime).toSec());
    string exploredCells = to_string(centralModule.map.coveredIndices.size());
    endExploration(centralModule.fileLogDir, endPub, exploredCells, explorationTime);
  }else{
    // change timeout mode if took too long to compute the auction info
    if(auctionInfoTime.toSec() > 4){
      auctionStartTimeoutMode = 2;
    }else if(auctionStartTimeoutModeParam < 2){
      auctionStartTimeoutMode = auctionStartTimeoutModeParam;
    }

    // As the bid was started bids can be received
    /// Reset auction variables
    requests = 0;
    /// Change state
    centralModule.setState(WaitingFirstBid);
    // Send auction info so the bids can be calculated
    auctionPub.publish(auction);
    ROS_INFO("Auction information broadcasted");
  }

  // set markers for rviz
  cout<<"debug :: setting rviz marks"<<endl;
  setRvizMarks(auction, centralModule.map.occupancyGrid.info);

  // log auction info construction time
  cout<<"debug :: log auction info construction time"<<endl;
  if (centralModule.fileLogLevel >= 2) {
    string exploredCells = to_string((float)centralModule.map.coveredIndices.size());

    // log auctionInfoTime
    string time = to_string(auctionInfoTime.toSec());
    logAppend(auctionInfoTimeLog, exploredCells + "  " + time);

    // log auctionInfoIncrementalTime
    string timeIncrement = to_string((auctionInfoTime - lastauctionInfoTime).toSec());
    logAppend(auctionInfoTimeIncrementLog, exploredCells + "  " + timeIncrement);
  }

  cout<<"debug :: auction started successfully"<<endl;
}

void resolveAuction() {
  cout<<"debug :: resolution starting"<<endl;

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
  cout<<"debug :: assignment starting"<<endl;
  boost::unordered_map<string, Assignment> assignments = centralModule.assign();
  cout<<"debug :: assignment end"<<endl;

  // As the assignment was calculated a new auction can be started
  /// Reset auction variables
  succesfulBids = 0;
  /// Change state
  centralModule.setState(WaitingAuction);

  // Parse the robot-frontier assignment
  float maxEstimatedTime = 0;       // max estimated task completion time
  float minEstimatedTime = FLT_MAX; // min estimated task completion time

  assignedRobots = centralModule.robotNumber;

  for (auto it : assignments) {
    RobotId robot = it.first;
    Assignment assignment = it.second;

    // Let the robot know about it assignment
    AssignmentPubs[robot].publish(assignment);

    if(assignment.assigned){
      // Calculate the estimated task completion time
      float estimatedTime = max(0.f,(centralModule.bids[robot][toPos(assignment.frontier)]) / (centralModule.robotSpeed));

      maxEstimatedTime = max(maxEstimatedTime, estimatedTime);
      minEstimatedTime = min(minEstimatedTime, estimatedTime);
    }else{
      assignedRobots--;
    }
  }

  if(assignedRobots > 0){
    // Set the timeout to start the next auction
    auctionStartTimeout = auctionInfoTime + estimatedAuctionInfoTimeIncrement + auctionStartDelayTimeout;
    auctionStartTimeoutTimer.setPeriod(auctionStartTimeout,false);

    /// Calculate the robots that are expected to complete the assigned task not long after the first auction request
    expectedRobots = assignedRobots;
    for (auto it : assignments) {
      Assignment assignment = it.second;

      if(assignment.assigned){
        string robot = it.first;

        float estimatedTime = max(0.f,(centralModule.bids[robot][toPos(assignment.frontier)]) / (centralModule.robotSpeed));
        if (estimatedTime > minEstimatedTime + auctionStartTimeout.toSec()) {
          expectedRobots--;
        }
      }
    }

    // Show info about the auction
    /// Show the last gvd construction the estimated and the max estimated time 
    ROS_INFO_STREAM("GVD | estimated time+map update delay"<<auctionStartTimeout.toSec()<<
                       " | first robot task completion time "<<minEstimatedTime + auctionStartTimeout.toSec()<<
                       " | last robot task completion estimated time "<<maxEstimatedTime);

    /// Show resolution
    ROS_INFO_STREAM("Auction resoved, robots assigned = "<<assignedRobots<<" | robots expected "<<expectedRobots);
  }else{
    ROS_INFO("No path to frontiers");
    // Notify termination to other nodes, the end module won't do it becouse it will never reach the coverge needed to end
    // (no robots assinged means no new cells explored)
    string explorationTime = to_string((ros::Time::now() - firstMapTime).toSec());
    string exploredCells = to_string(centralModule.map.coveredIndices.size());
    endExploration(centralModule.fileLogDir, endPub,exploredCells, explorationTime);
  }

  cout<<"debug :: resolution end"<<endl;
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

void mapCallBack(const OccupancyGridConstPtr& msg) {
  centralModule.updateMap(msg);
  if (firstMap){
    firstMapTime = ros::Time::now();
    firstMap = false;
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

void robotReportCallBack(const RobotReportConstPtr& msg, string name) {
  robotReports[name] = *msg;

  if(robotReports.size() == centralModule.robotNumber){
    tryToShutdown();
  }
}

void endCallBack(const std_msgs::StringConstPtr& msg) {
  endFlag = msg->data.compare(endMsg) == 0;

  if(endFlag){
    tryToShutdown();
  }
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
  n.param<int>("/auction_start_timeout_mode", auctionStartTimeoutModeParam, auctionStartTimeoutModeParam);
  n.param<float>("/map_update_delay", mapUpdateDelay, mapUpdateDelay);

  /// Frontier
  n.param<int>("/frontier_simplification_method", centralModule.frontierSimplificationMethod, centralModule.frontierSimplificationMethod);

  /// incremental
  n.param<int>("/auction_info_incremental", centralModule.auctionInfoIncremental, centralModule.auctionInfoIncremental);


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

  // log files
  auctionInfoTimeLog          = centralModule.fileLogDir + "/auction_info_time";
  auctionInfoTimeIncrementLog = centralModule.fileLogDir + "/auction_info_time_diff";
  metersTraveledLog           = centralModule.fileLogDir + "/meters_traveled";

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
  endPub            = n.advertise<std_msgs::String>("/end", 1);

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

      robotReportSubs[robotName] = n.subscribe<RobotReport>("/"+robotName+"/report", 1, boost::bind(&robotReportCallBack, _1, robotName));
      bidSubs[robotName] = n.subscribe<Bid>("/"+robotName+"/bid", 1, boost::bind(&bidCallBack, _1, robotName));
      AssignmentPubs[robotName] = n.advertise<Assignment>("/"+robotName+"/assigment", 1);

      centralModule.allRobots.insert(robotName);

      ROS_DEBUG_STREAM("Extracted `robot_name = "<<robotName<<"` from `topic_name = "<<publishedTopic.name<<"`");
    }
  }

  // Store rosparams
  std::system(("rosparam dump "+centralModule.fileLogDir+"/rosparams.yaml").c_str());

  // spin
  ROS_INFO_STREAM("Initilized");
  ros::spin();

  return 0;
}
