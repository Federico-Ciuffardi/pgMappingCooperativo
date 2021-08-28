#include "Robot.h"

//////////////////
// Constructors //
//////////////////

Robot::Robot() { }

///////////////
// Get / Set //
///////////////

/////////
// API //
/////////
Bid Robot::getBid(Auction msg) {
  // turn rosmsg Graph to GVD lib Graph
  gvd = toGraph<GvdVecGraph>(msg.gvd);

  // Convert the occupancy grid into stateGrid
  StateGrid stateGrid = toStateGrid(occupancyGrid);

  // Robot position
  /// Get the current Robot pos on the occupancy map frame
  Pos robotPos = toPos(position, occupancyGrid.info);

  /// add the robot to the gvd (if this is not possible, then add the robot as
  /// the only vertex in the GVD, this is necesary due to the navigation taking
  /// place on the GVD)
  if(!addToGraph(robotPos, gvd, stateGrid)){
    gvd.addV(robotPos);
  }

  // Frontiers
  /// convert vector<Point2D> forntiers to PosSet
  PosSet frontiers = toPosSet(msg.frontiers);

  /// add the frontiers to the gvd
  addToGraph(frontiers, gvd, stateGrid);

  // DEBUG: visuzlize map on std output
  /* stateGrid[robotPos] = Critical; */
  /* for( auto it : gvd.idVertexMap){ */
  /*   Pos vPos = it.first; */
  /*   stateGrid[vPos] = CriticalLine; */
  /* } */
  /* for( Pos fPos  : frontiers){ */
  /*   stateGrid[fPos] = Frontier; */
  /* } */
  /* cout<<stateGrid<<endl; */

  // get the path from the robotPos to each frontier in frontiers
  boost::tie(paths, pathCosts) = gvd.getMultiPath(robotPos, frontiers);
  /* cout<<"Calculated path costs: "<<pathCosts<<endl; //DEBUG */

  // Construct bid rosmsg 
  Bid bid;
  for (auto &it : pathCosts) {
    Pos frontier = it.first; 
    Float cost = it.second;

    bid.frontiers.push_back(toPoint2D(frontier));
    bid.values.push_back(pathCosts[frontier]);
  }

  return bid;
}

GoalList Robot::getPathTo(Pos frontier) {
  GoalList goalList;

  if(!is_elem(frontier,paths)){
    FAIL("No path to assigned objective. This is a bug, halting execution");
  }

  if (paths[frontier].size() == 0) {
    ROS_WARN("Empty path!");
    return goalList;
  }

  goalList.id = lastAssignmentId;

  for (GvdVecGraph::Vertex v : paths[frontier]) {
    Point p3d = toPoint(gvd[v].p, occupancyGrid.info);

    goalList.goals.push_back(p3d);
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
// traversability stablished on the stateGrid.
//
// Returns the set of points from where the graph is accesible. 
void Robot::addToGraph(PosSet& posSet, GvdVecGraph& graph, StateGrid& stateGrid) {
  list<list<Pos>> paths;
  for (Pos p : posSet) {
    paths.push_back(graph.findPath(p, stateGrid, {Occupied}));
  }
  for (list<Pos> path : paths){
    addToGraphBase(path, graph);
  }
}

// Generates a connection from p to the graph that complies with the traversability stablished
// on the stateGrid.
//
// Returns true if graph is accesible from p and false otherwise 
bool Robot::addToGraph(Pos p, GvdVecGraph& graph, StateGrid& stateGrid ) {
  list<Pos> path = graph.findPath(p, stateGrid, {Occupied});

  return addToGraphBase(path, graph);
}
