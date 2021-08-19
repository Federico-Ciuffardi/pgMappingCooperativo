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

// Generates a path from each p in posSet to the graph that complies with the 
// traversability stablished on the stateGrid.
//
// Returns the set of points from where the graph is accesible. 
void Robot::addToGraph(PosSet& posSet, GvdVecGraph& graph, StateGrid& stateGrid) {
  for (Pos p : posSet) {
    addToGraph(p, graph,stateGrid);
  }
}

// Generates a path from p to the graph that complies with the traversability stablished
// on the stateGrid.
//
// Returns true if graph is accesible from p and false otherwise 
bool Robot::addToGraph(Pos p, GvdVecGraph& graph, StateGrid& stateGrid ) {
  if (is_elem(p, gvd.idVertexMap)) return true;

  boost::unordered_map<Pos, Pos> predecessor;
  Pos connection;
  boost::tie(predecessor, connection) = graph.findPath(p, stateGrid, {Unknown, Occupied});

  Pos currPos = connection;

  do{
    if(currPos == NULL_POS){
      ROS_INFO("No path to graph.");
      return false;
    }

    Pos nextPos = predecessor[currPos];

    if(currPos==nextPos){
      cout<<"Loop on path to graph. This is a Bug, halting execution."<<endl;
      exit(1);
    }

    // add vertex to graph
    GvdVecGraph::Vertex nextV;
    bool inserted;
    boost::tie(nextV, inserted) = gvd.addV(nextPos);
    gvd[nextV].segment = gvd[connection].segment;

    // add edge to graph
    float d = currPos.distanceTo(nextPos);
    GvdVecGraph::Edge e;
    boost::tie(e, inserted) = gvd.addE(gvd.idVertexMap[currPos], nextV, d);
    boost::tie(e, inserted) = gvd.addE(nextV, gvd.idVertexMap[currPos], d);

    currPos = nextPos;
  } while (p != currPos);

  return true;
}

pgmappingcooperativo::Bid Robot::getBid(pgmappingcooperativo::Auction msg) {
  // Get the current Robot pos on the occupancy map frame
  Pos robotPos = getGVDPos();

  // turn msg Graph to GVD lib Graph
  gvd = toGraph<GvdVecGraph>(msg.gvd);

  // Process occupancy grid (get offset and turn to stateGrid)
  StateGrid  stateGrid = toStateGrid(map_merged.mapa);

  // add the robot to the gvd
  cout<<"Add robotPos to graph"<<endl;
  addToGraph(robotPos, gvd, stateGrid);

  // convert vector<Point2D> forntiers to PosSet
  PosSet frontiers = toPosSet(msg.frontiers);

  // add the frontiers to the gvd
  cout<<"Add frontiers to graph"<<endl;
  addToGraph(frontiers, gvd, stateGrid);

  // visuzlize map on std output
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
  cout<<"Path costs: "<<pathCosts<<endl;

  // Construct bid rosmsg 
  pgmappingcooperativo::Bid bid;
  for (Pos frontier : frontiers) {
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
    ROS_WARN("Empty path to frontier!");
    return goalList;
  }

  goalList.indice = 1;  // TODO poner bien el indice

  for (GvdVecGraph::Vertex v : paths[frontier]) {
    geometry_msgs::Point p3d = pos_to_real_p3d(gvd[v].p);

    goalList.listaGoals.push_back(p3d);
  }

  return goalList;
}
