#include "Robot.h"

Robot::Robot() {
  // first = true;
  sensor_range = 6.0;
  lado = 1;
  auction_robots = 0;
  // dist_info_gain_obst = lado / sqrt(2);
}

void Robot::setPosition(int x, int y) {
  position.x = x;
  position.y = y;
};

geometry_msgs::Point Robot::getPosition() {
  return position;
};

Pos Robot::getGVDPos() {
  geometry_msgs::Point p3d = getPosition();

  // Pos adjustment(signo((int)position.x), 0);
  Pos adjustment(0, 0);

  return p3d_to_pos(p3d) - offset + adjustment;
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

void Robot::set_grid() {
  grid = og2gt(map_merged.mapa);
}

// this should be moved to gvd.h, this function could me sub by a smarter solution(using the fuction
// dis) but we did it to finish
/* void Robot::add_to_gvd(Pos f_pos) { */
/*   float min = -1; */
/*   float min_aux; */
/*   GvdVecGraph::Vertex v_min; */
/*   GvdVecGraph::Vertex v; */
/*   bool inserted; */
/*   GvdVecGraph::Edge e; */

/*   // ROS_INFO("romi VOY A ENCONTRAR EL MAS CERCANO"); */

/*   GvdVecGraph::VertexIterator v_it, v_it_end; */
/*   for (tie(v_it, v_it_end) = vertices(gvd.g); v_it != v_it_end; v_it++) { */
/*     Pos v_pos = gvd.g[*v_it].p; */
/*     min_aux = f_pos.distance_to(v_pos); */

/*     if (min < 0 || min_aux < min) { */
/*       min = min_aux; */
/*       v_min = *v_it; */
/*     } */
/*   } */

/*   if (min != 0) { */
/*     // ROS_INFO("romi VOY A AGREGAR LA FRONTERA AL GVD"); */
/*     boost::tie(v, inserted) = gvd.add_v(f_pos); */
/*     // ROS_INFO("romi vertice %d",inserted); */
/*     boost::tie(e, inserted) = gvd.add_e(v, v_min, min); */
/*     boost::tie(e, inserted) = gvd.add_e(v_min, v, min); */
/*     // ROS_INFO("romi arista %d",inserted); */
/*     // ROS_INFO("romi arista es: %d,%d - %d,%d */
/*     // ",gvd.g[v].p.first,gvd.g[v].p.second,gvd.g[v_min].p.first,gvd.g[v_min].p.second); */
/*     // ROS_INFO("romi f_pos vetex: %d",gvd.vertices[f_pos]); */
/*   } */
/* } */

void Robot::add_to_gvd(PosSet p_set) {
  for (Pos pToAdd : p_set) {
    add_to_gvd(pToAdd);
  }
}

void Robot::add_to_gvd(Pos pToAdd) {
  if (is_elem(pToAdd,gvd.vertices)) return;

  boost::unordered_map<Pos, Pos> predecessor;
  Pos connection;
  boost::tie(predecessor, connection) = gvd.findPath(pToAdd, grid, {Unknown, Occupied});

  Pos currPos = connection;

  do{
    if(currPos == NULL_POS){
      ROS_INFO("No path to GVD");
      exit(1);
    }

    Pos nextPos = predecessor[currPos];

    if(currPos==nextPos){
      cout<<"Loop on path to GVD"<<endl;
      exit(1);
    }

    GvdVecGraph::Vertex nextV;
    bool inserted;
    boost::tie(nextV, inserted) = gvd.add_v(nextPos);
    gvd[nextV].segment = gvd[connection].segment;

    float d = currPos.distance_to(nextPos);
    GvdVecGraph::Edge e;
    boost::tie(e, inserted) = gvd.add_e(gvd.vertices[currPos], nextV, d);
    boost::tie(e, inserted) = gvd.add_e(nextV, gvd.vertices[currPos], d);

    currPos=nextPos;
  } while (pToAdd != currPos);
}

// Robot process graph, creates boost gvd also adds Pos->gvd
GvdVecGraph Robot::getGVD(pgmappingcooperativo::Graph g, vector<pgmappingcooperativo::Point2D> vertex_segment) {
  // std::cout<<"arranca el get gvd"<<endl;
  GvdVecGraph gvd;
  Pos v_pos;
  float min = -1;
  float min_aux;
  GvdVecGraph::Vertex v;
  bool inserted;
  graph_traits<GvdVecGraph::GraphType>::edge_descriptor e;
  int segment = -1;

  // std::cout<<"Antes de agregar vertices"<<g.vertices.size()<<endl;
  for (int i = 0; i < g.vertices.size(); i++) {
    v_pos = p2d_to_pos(g.vertices[i]);
    boost::tie(v, inserted) = gvd.add_v(v_pos);

    // min_aux = dist(r_pos, v_pos);
    // std::cout<<"min_aux "<<min_aux<<endl;
    // std::cout<<"min "<<min<<endl;
    /*if (min < 0 || min_aux < min) {
      min = min_aux;
      segment = i;
      // std::cout<<segment<<endl;
    }*/
    gvd.g[v].segment = p2d_to_pos(vertex_segment[i]);
  }
  // std::cout<<"Antes de agregar aristas: "<<g.edges.size()<<endl;
  // auto weights = get(edge_weight,gvd.g);
  for (int i = 0; i < g.edges.size(); i++) {
    Pos from_p = p2d_to_pos(g.edges[i].from);
    Pos to_p = p2d_to_pos(g.edges[i].to);
    GvdVecGraph::Vertex from_v = gvd.vertices[from_p];
    GvdVecGraph::Vertex to_v = gvd.vertices[to_p];

    boost::tie(e, inserted) = gvd.add_e(from_v, to_v, from_p.distance_to( to_p));
    // sqrt(pow(from_p.first - to_p.first, 2) + pow(from_p.second - to_p.second, 2));
  }
  // std::cout<<"Termine de agregar todas las aristas"<<endl;
  return gvd;
}

// Return true if robot is in segment assigned_segment and false if it isnt
bool Robot::is_in_segment(Pos my_segment, Pos my_pos, Pos assigned_segment, Pos f_pos) {
  float f_r_dist = f_pos.distance_to( my_pos);
  float f_c_dist = f_pos.distance_to( assigned_segment);
  return (f_r_dist < f_c_dist);
}

pgmappingcooperativo::SegmentBid Robot::getSegmentBid(pgmappingcooperativo::SegmentAuction msg) {
  // ROS_INFO("tiempos arranca el getSegmentBid---------------------");
  // std::cout << "declaracion de segment bid en la prox lienea" << endl;
  pgmappingcooperativo::SegmentBid segment_bid;
  // std::cout << "getGVDPOS en la prox lienea" << endl;
  offset = p2d_to_pos(msg.offset);
  my_pos = getGVDPos();
  // std::cout << "Consegui my Pos: "<<endl;
  Pos r_pos = my_pos;

  Pos r_segment, c_pos, f_pos;

  gvd = getGVD(msg.gvd, msg.vertex_segment);
  set_grid();

  PosSet r_set;
  r_set.insert(r_pos);
  add_to_gvd(r_set);
  // std::cout << "este es el seg: " << seg << endl;
  r_segment = gvd.g[gvd.vertices[r_pos]].segment;
  my_segment = r_segment;
  // std::cout<<"se logro calcular el seg: "<<r_segment.first<<","<<r_segment.first<<endl;

  Pos my_segment_frontier;
  PosSet criticals_and_frontiers;
  PosSet frontiers;
  int my_segment_frontier_index;
  for (int i = 0, j = 0; i < msg.criticals.size(); i++) {
    Pos c_pos = p2d_to_pos(msg.criticals[i]);
    criticals_and_frontiers.insert(c_pos);
    bool first = true;

    for (; (j < msg.frontiers_segment.size()) && (msg.criticals[i] == msg.frontiers_segment[j]);
         j++) {
      // if is the closest frontier o a frontier of my segment
      Pos f_segment = p2d_to_pos(msg.frontiers_segment[j]);
      f_pos = p2d_to_pos(msg.frontiers[j]);

      if (first || r_segment == f_segment) {
        // if is in my segment and also the closest one to the critical
        if (first && r_segment == f_segment) {
          my_segment_frontier = f_pos;
          my_segment_frontier_index = j;
        }

        first = false;
        // add the frontier to the set
        criticals_and_frontiers.insert(f_pos);
        frontiers.insert(f_pos);
      }
    }
  }
  // add the frontiers to the gvd
  add_to_gvd(frontiers);

  // boost::unordered_map<Pos,float> paths_costs;

  // ROS_INFO("tiempos arranca el multipath");
  boost::tie(paths, paths_costs) = gvd.getMultiPath(r_pos, criticals_and_frontiers);
  // ROS_INFO("tiempos termina el multipath");

  list<GvdVecGraph::Vertex> path_to_frontier = paths[my_segment_frontier];
  bool in_segment = find(path_to_frontier.begin(), path_to_frontier.end(),
                         gvd.vertices[r_segment]) == path_to_frontier.end();
  // if robot in segment
  float cost_my_segment_frontier;
  if (in_segment) {
    cost_my_segment_frontier = -1;
    for (int i = my_segment_frontier_index;
         (i < msg.frontiers_segment.size()) && (r_segment == p2d_to_pos(msg.frontiers_segment[i]));
         i++) {
      f_pos = p2d_to_pos(msg.frontiers[i]);
      float f_r_dist = paths_costs[f_pos];
      // get the closes frontier to the robot
      if ((cost_my_segment_frontier == -1) || (f_r_dist < cost_my_segment_frontier)) {
        cost_my_segment_frontier = f_r_dist;
      }
    }
  }

  for (int i = 0; i < msg.criticals.size(); i++) {
    segment_bid.criticals.push_back(msg.criticals[i]);
    c_pos = p2d_to_pos(msg.criticals[i]);
    float cost = msg.mind_f[i] + paths_costs[c_pos];
    // float cost = paths_costs[p2d_to_pos(msg.minp_f[i])];

    float in_seg = 0;
    if (in_segment && (r_segment == c_pos)) {
      in_seg = paths_costs[c_pos];
      cost = cost_my_segment_frontier;
    }
    // crit a la frontera + (robot al critico)*c
    segment_bid.values.push_back(cost - in_seg);
    // segment_bid.values.push_back(cost);
  }
  // std::cout<<"Termino!"<<endl;
  // ROS_INFO("Termino!");
  // ROS_INFO("tiempos termina el getSegmentBid -----------------");
  return segment_bid;
}

geometry_msgs::Point Robot::pos_to_real_p3d(Pos p) {
  geometry_msgs::Point p3d = pos_to_p3d(p + offset);
  p3d.x += 0.5;
  p3d.y += 0.5;
  return p3d;
}

// set on the robot variables paths and paths_costs to the frontiers on the array
void Robot::set_my_paths_to_frontiers(vector<pgmappingcooperativo::Point2D> points) {
  PosSet f_set;
  for (int i = 0; i < points.size(); i++) {
    Pos f_pos = p2d_to_pos(points[i]);
    // add_to_gvd(f_pos);
    // add_to_gvd(f_pos);
    f_set.insert(f_pos);
  }
  add_to_gvd(f_set);

  // add_to_gvd(f_set, assigned_segment);
  // maybe i could recalculate for every frontier to be more exact
  if (true) {  // assigned_segment == my_segment){
    boost::tie(paths, paths_costs) = gvd.getMultiPath( my_pos, f_set);
    // ROS_INFO("romi calcule el camino a las fronteras");
  } else {
    boost::unordered_map<Pos, list<GvdVecGraph::Vertex>> f_paths;
    boost::unordered_map<Pos, float> f_paths_costs;
    boost::tie(f_paths, f_paths_costs) = gvd.getMultiPath(assigned_segment, f_set);
    // duplicated information (robot->segment)
    // where should i assigned the path, under key assigned_segment or under frontier?
    for (auto it = f_set.begin(); it != f_set.end(); ++it) {
      // paths[assigned_segment].splice(paths[assigned_segment].end(), f_paths[*it]);
      // f_paths[*it].pop_front();
      f_paths[*it].splice(f_paths[*it].begin(), paths[assigned_segment]);
      // paths[*it] = paths[assigned_segment];
      paths[*it] = f_paths[*it];
      paths_costs[*it] = paths_costs[assigned_segment] + f_paths_costs[*it];
    }
  }
}

int Robot::getRobotId() {
  int id = 0;
  // for(int i = nombreRobot.size()-1; i>= 0; i--){
  // if(!isdigit(nombreRobot[i])){
  return stoi(nombreRobot.substr(5));
  //}
  //}
  // return -1;
}

pgmappingcooperativo::FrontierBid Robot::getFrontierBid(vector<pgmappingcooperativo::Point2D> frontiers) {
  set_my_paths_to_frontiers(frontiers);
  pgmappingcooperativo::FrontierBid msg;
  for (auto it = frontiers.begin(); it != frontiers.end(); ++it) {
    msg.frontiers.push_back(*it);
    msg.values.push_back(paths_costs[p2d_to_pos(*it)]);
  }
  msg.robotId = getRobotId();
  msg.id = last_segment_assignment_id;
  last_frontier_auction_id = last_segment_auction_id;
  return msg;
}

bool Robot::saveFrontierBid(pgmappingcooperativo::FrontierBid fb) {
  // segment_bids[name].clear();
  for (int i = 0; i < fb.frontiers.size(); i++) {
    // segment_bids[name][p2d_to_pos(sb.criticals[i])] = sb.values[i];
    Pos f_pos = p2d_to_pos(fb.frontiers[i]);
    string name = to_string(fb.robotId);
    add_bid(bids_pq, name, f_pos, fb.values[i]);
    // auction_segment_frontiers_num[f_pos] = cis[segment].frontiers.size();
  }
  auction_robots = auction_robots + 1;
  return true;
}

Pos Robot::assignFrontier() {
  int total_robots = auction_robots;
  int total_frontiers = paths.size();
  string id = to_string(getRobotId());
  boost::unordered_map<string, Pos> robot_frontiers =
      resolve_auction(bids_pq, total_robots, total_frontiers);
  return robot_frontiers[id];
}

// First solution, didnot work properly
// Adds extra points so the robot would walk a straight line from current_pos till f_pos(frontier)
void Robot::add_intermidiate_points(Pos f_pos,
                                    Pos current_pos,
                                    pgmappingcooperativo::goalList& g_list,
                                    float min_dist) {
  int division_count = f_pos.distance_to(current_pos) / min_dist;
  // the formula is (x,y) = (x1 + k(x2-x1), y1+k(y2-y1))
  for (int i = 1; i < division_count; i++) {
    // current_pos.first = current_pos.first + i/division_count*(f_pos.first - current_pos.first);
    // current_pos.second = current_pos.second + i/division_count*(f_pos.second -
    // current_pos.second);
    current_pos = current_pos + (i * (f_pos - current_pos)) / division_count;
    // add to path
    geometry_msgs::Point p3d = pos_to_real_p3d(current_pos);
    g_list.listaGoals.push_back(p3d);
    // ROS_INFO("way intermedio %f,%f,%f", p3d.x, p3d.y, p3d.z);
  }
}

pgmappingcooperativo::goalList Robot::getPathToSegment(Pos frontier) {
  // ROS_INFO("romi ENTREEEEEEEEEEEEEEEEEE");
  // Pos f_pos = p2d_to_pos(frontier);

  // ROS_INFO("tiempos forntera al gvd");
  // add_to_gvd(f_pos);
  // ROS_INFO("tiempos fin forntera al gvd");
  // ROS_INFO("romi ingrese la frontera al gvd");

  // ROS_INFO("tiempos voy a calcular el camino");
  // std::list<GvdVecGraph::Vertex> path;
  // float cost;

  // ROS_INFO("tiempos comienza de calcular el camino");
  // boost::tie(path, cost) = get_single_path(gvd, my_pos, f_pos);
  // ROS_INFO("tiempos termine de calcular el camino");

  if (paths[frontier].size() == 0) {
    ROS_WARN("Empty path");
  }

  pgmappingcooperativo::goalList g_list;
  g_list.indice = 1;  // TODO poner bien el indice

  list<GvdVecGraph::Vertex> v_list = paths[frontier];
  geometry_msgs::Point p3d;

  // ROS_INFO("romi iterar en los nodos para pasarlos a p3d");

  for (auto it = v_list.begin(); it != v_list.end(); it++) {
    p3d = pos_to_real_p3d(gvd.g[*it].p);

    g_list.listaGoals.push_back(p3d);

    // ROS_INFO("romi way puntos en el camino %f,%f,%f", p3d.x, p3d.y, p3d.z);
  }

  // ROS_INFO("TERMINEEEEEEEEEEEEE");
  return g_list;
}
