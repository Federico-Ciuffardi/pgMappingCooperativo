#include "Robot.h"

//////////////////
// Constructors //
//////////////////

Robot::Robot() { }

///////////////////
// Aux Functions //
///////////////////

/////////
// API //
/////////
Bid Robot::getBid(Auction msg) {
  // turn rosmsg Graph to GVD lib Graph
  cout<<"gvd = toGraph<GvdVecGraph>(msg.gvd);"<<endl;
  gvd = toGraph<GvdVecGraph>(msg.gvd);

  // Convert the occupancy grid into Map
  cout<<"Map map = toMap(occupancyGrid);"<<endl;
  Map map = toMap(occupancyGrid);

  // Robot position
  /// Get the current Robot pos on the occupancy map frame
  Vector2<Float> robotVector2 = toVector2<Float>(position, occupancyGrid.info);
  robotBidPos                 = toPos(position, occupancyGrid.info);

  /// add the robot to the gvd (if this is not possible, then add the robot as
  /// the only vertex in the GVD, this is necesary due to the navigation taking
  /// place on the GVD)
  cout<<"gvd.addV(robotBidPos);"<<endl;
  if(!addToGraph(robotBidPos, gvd, map)){
    gvd.addV(robotBidPos);
  }

  // Frontiers
  /// convert vector<Point2D> forntiers to PosSet
  cout<<"PosSet frontiers = toPosSet(msg.frontiers);"<<endl;
  PosSet frontiers = toPosSet(msg.frontiers);

  // Add trivial frontiers to the Bid
  cout<<"Add trivial frontiers to the Bid"<<endl;
  Float maxPathEntryYawPenaltyUsed = 0;
  Bid bid;
  nonTrivialFrontiers.clear();
  for(Pos frontier : frontiers){
    if(unobstructedLine(robotBidPos,frontier,occupancyGrid,meterToCells)){
      bid.frontiers.push_back(toPoint2D(frontier));
      bid.pathLength.push_back(robotBidPos.distanceTo(frontier)*occupancyGrid.info.resolution);

      Float robotYaw = toFloat(orientation);
      Float yawToPath = (toVector2<Float>(frontier) - robotVector2).angle();
      Float pathEntryYaw = abs(minAngleRep(robotYaw-yawToPath));
      Float pathEntryYawPenalty = (pathEntryYaw/M_PI)*pathEntryYawMaxPenalty;
      bid.pathEntryYaw.push_back(pathEntryYawPenalty);
      maxPathEntryYawPenaltyUsed = max(maxPathEntryYawPenaltyUsed, pathEntryYawPenalty);
      /* bid.pathEntryYaw.push_back(0); */
    }else{
      nonTrivialFrontiers.insert(frontier);
    }
  }

  if(!nonTrivialFrontiers.empty()){
    /// add the non-trivial frontiers to the gvd
    cout<<"addToGraph(nonTrivialFrontiers, gvd, map);"<<endl;
    addToGraph(nonTrivialFrontiers, gvd, map);

    // get the path from the robotBidPos to each non-trivial frontier
    cout<<"boost::tie(nonTrivialPaths, nonTrivialPathLenght) = gvd.getMultiPath(robotBidPos, nonTrivialFrontiers);"<<endl;
    boost::tie(nonTrivialPaths, nonTrivialPathLenght) = gvd.getMultiPath(robotBidPos, nonTrivialFrontiers);
  }

  // Add non trivial frontiers to the Bid
  cout<<"Add non trivial frontiers to the Bid"<<endl;
  for (auto &it : nonTrivialPathLenght) {
    Pos frontier = it.first; 
    Float cost = it.second;

    bid.frontiers.push_back(toPoint2D(frontier));
    bid.pathLength.push_back(nonTrivialPathLenght[frontier]*occupancyGrid.info.resolution);
    bid.pathEntryYaw.push_back(maxPathEntryYawPenaltyUsed);
    /* bid.pathEntryYaw.push_back(0); */
  }

  // Set the robot position
  bid.robotPosition = toPoint2D(robotBidPos);

  return bid;
}

GoalList Robot::getPathTo(Pos frontier) {
  GoalList goalList;
  goalList.id = lastAssignmentId;

  if(is_elem(frontier,nonTrivialPaths)){

    if (nonTrivialPaths[frontier].size() == 0) {
      ROS_WARN("Empty path!");
      return goalList;
    }

    for (GvdVecGraph::Vertex v : nonTrivialPaths[frontier]) {
      goalList.goals.push_back( toPoint(gvd[v].p, occupancyGrid.info) );
    }

  }else if(!is_elem(frontier,nonTrivialFrontiers)){

    goalList.goals.push_back( toPoint(robotBidPos, occupancyGrid.info) );
    goalList.goals.push_back( toPoint(frontier, occupancyGrid.info) );

  }else{
    FAIL("No path to assigned objective. This is a bug, halting execution");
  }
  return goalList;
}

/////////
// Aux //
/////////
  
bool addToGraphBase(list<Pos> &path, GvdVecGraph &graph) {
  if(path.empty()) return false;

  // get first pos
  Pos pos = *path.begin();
  GvdVecGraph::Vertex vertex = graph.idVertexMap[pos];

  // Construct path 
  for(auto it = ++path.begin(); it != path.end(); it++){
    Pos nextPos = *it;

    // add vertex
    GvdVecGraph::Vertex nextVertex;
    bool inserted;
    boost::tie(nextVertex, inserted) = graph.addV(nextPos);

    float d = pos.distanceTo(nextPos);
    // add edge to graph
    graph.addE(vertex, nextVertex, d);
    graph.addE(nextVertex, vertex, d);

    // go to next in the path 
    pos = nextPos;
    vertex = nextVertex;
  }

  return true;
}

// Generates a connection from each p in posSet to the graph that complies with the 
// traversability stablished on the Map.
//
// Returns the set of points from where the graph is accesible. 
void Robot::addToGraph(PosSet& posSet, GvdVecGraph& graph, Map& map) {
  list<list<Pos>> paths;
  for (Pos p : posSet) {
    paths.push_back(graph.findPath(p, map, {Occupied}));
  }
  for (list<Pos> path : paths){
    addToGraphBase(path, graph);
  }
}

// Generates a connection from p to the graph that complies with the traversability stablished
// on the Map.
//
// Returns true if graph is accesible from p and false otherwise 
bool Robot::addToGraph(Pos p, GvdVecGraph& graph, Map& map ) {
  list<Pos> path = graph.findPath(p, map, {Occupied});

  return addToGraphBase(path, graph);
}
