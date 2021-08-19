#include "Robot.h"


///////////////////
// Aux functions //
///////////////////
Pos Robot::getOffset(){
 return toPos(map_merged.mapa.info.origin.position);
}

/////////////////////
// Robot functions //
/////////////////////


Robot::Robot() {
  sensor_range = 6.0;
  lado = 1;
}

geometry_msgs::Point Robot::getPosition() {
  return position;
};

Pos Robot::getGVDPos() {
  geometry_msgs::Point p3d = getPosition();

  // Pos adjustment(signo((int)position.x), 0);
  Pos adjustment(0, 0);

  return toPos(p3d) - getOffset() + adjustment;
}

void Robot::setNombre(std::string nom) {
  nombreRobot = nom;
};

std::string Robot::getNombre() {
  return nombreRobot;
};

void Robot::savePose(const geometry_msgs::Pose msg) {
  position.x = msg.position.x;
  position.y = msg.position.y;
}

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
    ROS_INFO("No path to graph.");
    return false;
  }

  do{
    // get the next position in the path from the graph to p
    Pos nextPos = predecessor[currPos];

    // check for errors
    if(nextPos == NULL_POS){
      ROS_INFO("NULL_POS in the middle of a path to the graph. This is a Bug, halting execution.");
      exit(1);
    }

    if(currPos == nextPos){
      cout<<"Loop on path to graph. This is a Bug, halting execution."<<endl;
      exit(1);
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

pgmappingcooperativo::Bid Robot::getBid(pgmappingcooperativo::Auction msg) {
  // turn rosmsg Graph to GVD lib Graph
  gvd = toGraph<GvdVecGraph>(msg.gvd);

  // Convert the occupancy grid into stateGrid
  StateGrid stateGrid = toStateGrid(map_merged.mapa);

  // Robot position
  /// Get the current Robot pos on the occupancy map frame
  Pos robotPos = getGVDPos();

  /// add the robot to the gvd (if this is not possible, then add the robot as
  /// the only vertex in the GVD, this is necesary due to the navigation taking
  /// place on the GVD)
  cout<<"Add robotPos to graph"<<endl;
  if(!addToGraph(robotPos, gvd, stateGrid)){
    gvd.addV(robotPos);
  }

  // Frontiers
  /// convert vector<Point2D> forntiers to PosSet
  PosSet frontiers = toPosSet(msg.frontiers);

  /// add the frontiers to the gvd
  cout<<"Add frontiers to graph"<<endl;
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
  cout<<"Calculated path costs: "<<pathCosts<<endl;

  // Construct bid rosmsg 
  pgmappingcooperativo::Bid bid;
  for (auto &it : pathCosts) {
    Pos frontier = it.first; 
    Float cost = it.second;

    bid.frontiers.push_back(toPoint2D(frontier));
    bid.values.push_back(pathCosts[frontier]);
  }

  return bid;
}

geometry_msgs::Point Robot::pos_to_real_p3d(Pos p) {
  geometry_msgs::Point p3d = toPoint(p + getOffset());
  p3d.x += 0.5;
  p3d.y += 0.5;
  return p3d;
}

int Robot::getRobotId() {
  return stoi(nombreRobot.substr(5));
}

pgmappingcooperativo::goalList Robot::getPathTo(Pos frontier) {
  pgmappingcooperativo::goalList goalList;

  if (paths[frontier].size() == 0) {
    ROS_WARN("Empty path!");
    return goalList;
  }

  goalList.indice = 1;  // TODO poner bien el indice

  for (GvdVecGraph::Vertex v : paths[frontier]) {
    geometry_msgs::Point p3d = pos_to_real_p3d(gvd[v].p);

    goalList.listaGoals.push_back(p3d);
  }

  return goalList;
}
