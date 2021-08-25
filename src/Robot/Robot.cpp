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
  StateGrid stateGrid = toStateGrid(mapMerged.occupancyGrid);

  // Robot position
  /// Get the current Robot pos on the occupancy map frame
  Pos robotPos = toPos(position, mapMerged.occupancyGrid.info);

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

  goalList.id = 1;  // TODO poner bien el id

  for (GvdVecGraph::Vertex v : paths[frontier]) {
    Point p3d = toPoint(gvd[v].p, mapMerged.occupancyGrid.info);

    goalList.goals.push_back(p3d);
  }

  return goalList;
}

/////////
// Aux //
/////////

// Generates a connection from each p in posSet to the graph that complies with the 
// traversability stablished on the stateGrid.
//
// Returns the set of points from where the graph is accesible. 
void Robot::addToGraph(PosSet& posSet, GvdVecGraph& graph, StateGrid& stateGrid) {
  for (Pos p : posSet) {
    addToGraph(p, graph,stateGrid);
  }
}

// Generates a connection from p to the graph that complies with the traversability stablished
// on the stateGrid.
//
// Returns true if graph is accesible from p and false otherwise 
bool Robot::addToGraph(Pos p, GvdVecGraph& graph, StateGrid& stateGrid ) {
  if (is_elem(p, gvd.idVertexMap)) return true;

  boost::unordered_map<Pos, Pos> predecessor;
  Pos connection;
  boost::tie(predecessor, connection) = graph.findPath(p, stateGrid, {Unknown, Occupied});

  Pos currPos = connection;
  GvdVecGraph::Vertex currVertex = gvd.idVertexMap[currPos];

  if(currPos == NULL_POS){
    /* cout<<"No path from "<<p<<" to the graph"<<endl; // DEGUG */
    return false;
  }

  do{
    // get the next position in the path from the graph to p
    Pos nextPos = predecessor[currPos];

    // check for errors
    if(nextPos == NULL_POS){
      FAIL("NULL_POS in the middle of a path to the graph. This is a Bug, halting execution.");
    }

    if(currPos == nextPos){
      FAIL("Loop on path to graph. This is a Bug, halting execution.");
    }

    // add vertex to graph
    GvdVecGraph::Vertex nextVertex;
    bool inserted;
    boost::tie(nextVertex, inserted) = gvd.addV(nextPos);

    // add edge to graph
    float d = currPos.distanceTo(nextPos);
    gvd.addE(currVertex, nextVertex, d);
    gvd.addE(nextVertex, currVertex, d);

    // update for the next loop
    currVertex = nextVertex;
    currPos = nextPos;

  } while (p != currPos);

  return true;
}
