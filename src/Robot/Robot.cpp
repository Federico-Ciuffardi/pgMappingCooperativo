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

pos Robot::getGVDPos() {
  geometry_msgs::Point p3d = getPosition();

  pos adjustment(signo((int)position.x), 0);

  return p3d_to_pos(p3d) - offset + adjustment;
}

void Robot::setNombre(std::string nom) {
  nombreRobot = nom;
};

std::string Robot::getNombre() {
  return nombreRobot;
};

void Robot::savePose(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  position.x = msg->pose.position.x;
  position.y = msg->pose.position.y;
}

void Robot::set_grid(){
  grid = og2gt(map_merged.mapa);
}

//this should be moved to gvd.h, this function could me sub by a smarter solution(using the fuction dis) but we did it to finish
void Robot::add_to_gvd(pos f_pos){
  float min = -1;
  float min_aux;
  VecGVD::Vertex v_min;
  VecGVD::Vertex v;
  bool inserted;
  VecGVD::Edge e;
  
  //ROS_INFO("romi VOY A ENCONTRAR EL MAS CERCANO");

  VecGVD::VertexIterator v_it, v_it_end;
  for (tie(v_it, v_it_end) = vertices(gvd.g); v_it != v_it_end; v_it++) {
    pos v_pos = gvd.g[*v_it].p;
    min_aux = dist(f_pos, v_pos);

    if (min < 0 || min_aux < min) {
      min = min_aux;
      v_min = *v_it;
    }
  }

  if (min != 0) {
    //ROS_INFO("romi VOY A AGREGAR LA FRONTERA AL GVD");
    boost::tie(v, inserted) = gvd.add_v(f_pos);
    //ROS_INFO("romi vertice %d",inserted);
    boost::tie(e, inserted) = gvd.add_e(v, v_min, min);
    boost::tie(e, inserted) = gvd.add_e(v_min,v, min);
    //ROS_INFO("romi arista %d",inserted);
    //ROS_INFO("romi arista es: %d,%d - %d,%d ",gvd.g[v].p.first,gvd.g[v].p.second,gvd.g[v_min].p.first,gvd.g[v_min].p.second);
    //ROS_INFO("romi f_pos vetex: %d",gvd.positions[f_pos]);
  }

}

void Robot::add_to_gvd(pos_set p_set){
  boost::unordered_map<pos,boost::unordered_map<pos,pos>> v_paths;
  boost::unordered_map<pos,pos> v_connection;
  VecGVD::Vertex v_pred;
  VecGVD::Vertex v;
  bool inserted;
  VecGVD::Edge e;

  for(auto it = p_set.begin(); it != p_set.end(); ++it){
    ROS_INFO("romi voy a calcular el camino");
    boost::tie(v_paths[*it], v_connection[*it]) = find_paths_to_gvd(grid, gvd, *it);
    ROS_INFO("romi termine de calcular el camino");
    if(v_connection[*it] == pos() || v_paths[*it].size() == 0){
      ROS_INFO("romi ERROR");
    }
  }
  for(auto it = p_set.begin(); it != p_set.end(); ++it){
    pos v_pred_pos;
    pos v_pos = v_connection[*it];
    v = gvd.positions[v_pos];
    boost::unordered_map<pos,pos> v_predecessor = v_paths[*it];
    for(v_pred_pos = v_predecessor[v_pos]; true; v_pred_pos = v_predecessor[v_pos]){
      ROS_INFO("romi voy a recorerer el camino");
      boost::tie(v_pred, inserted) = gvd.add_v(v_pred_pos);
      ROS_INFO("romi vertice %d",inserted);
      float d = dist(v_pred_pos, v_pos);
      boost::tie(e, inserted) = gvd.add_e(v, v_pred, d);
      boost::tie(e, inserted) = gvd.add_e(v_pred,v, d);
      v = v_pred;
      v_pos = v_pred_pos;
      if(v_pos == (*it)){
        ROS_INFO("romi UNION AL GVD, inserte el punto(frontera o robot) vetex: %d", gvd.positions[v_pos]);
        break;
      } 
      //ROS_INFO("romi arista %d",inserted);
      //ROS_INFO("romi arista es: %d,%d - %d,%d ",gvd.g[v].p.first,gvd.g[v].p.second,gvd.g[v_min].p.first,gvd.g[v_min].p.second);
      //ROS_INFO("romi f_pos vetex: %d",gvd.positions[f_pos]);
    }
  }
}


// Robot process graph, creates boost gvd also adds pos->gvd
boost::tuple<int, VecGVD> Robot::getGVD(tscf_exploration::Graph g, pos r_pos) {
  // std::cout<<"arranca el get gvd"<<endl;
  VecGVD gvd;
  pos v_pos;
  float min = -1;
  float min_aux;
  VecGVD::Vertex v;
  bool inserted;
  graph_traits<VecGVD::Graph>::edge_descriptor e;
  int segment = -1;

  // std::cout<<"Antes de agregar vertices"<<g.vertices.size()<<endl;
  for (int i = 0; i < g.vertices.size(); i++) {
    v_pos = p2d_to_pos(g.vertices[i]);
    boost::tie(v, inserted) = gvd.add_v(v_pos);

    min_aux = dist(r_pos, v_pos);
    // std::cout<<"min_aux "<<min_aux<<endl;
    // std::cout<<"min "<<min<<endl;
    if (min < 0 || min_aux < min) {
      min = min_aux;
      segment = i;
      // std::cout<<segment<<endl;
    }
  }
  // std::cout<<"Antes de agregar aristas: "<<g.edges.size()<<endl;
  // auto weights = get(edge_weight,gvd.g);
  for (int i = 0; i < g.edges.size(); i++) {
    pos from_p = p2d_to_pos(g.edges[i].from);
    pos to_p = p2d_to_pos(g.edges[i].to);
    VecGVD::Vertex from_v = gvd.positions[from_p];
    VecGVD::Vertex to_v = gvd.positions[to_p];

    boost::tie(e, inserted) = gvd.add_e(from_v, to_v, dist(from_p, to_p));
    // sqrt(pow(from_p.first - to_p.first, 2) + pow(from_p.second - to_p.second, 2));
  }
  // std::cout<<"Termine de agregar todas las aristas"<<endl;
  return boost::make_tuple(segment, gvd);
}

// Return true if robot is in segment assigned_segment and false if it isnt
bool Robot::is_in_segment(pos my_segment, pos my_pos, pos assigned_segment, pos f_pos){
  float f_r_dist = dist(f_pos, my_pos);
  float f_c_dist = dist(f_pos, assigned_segment);
  return (my_segment == assigned_segment) && (f_r_dist < f_c_dist);
}

tscf_exploration::SegmentBid Robot::getSegmentBid(tscf_exploration::SegmentAuction msg) {
  //ROS_INFO("tiempos arranca el getSegmentBid---------------------");
  //std::cout << "declaracion de segment bid en la prox lienea" << endl;
  tscf_exploration::SegmentBid segment_bid;
  //std::cout << "getGVDPOS en la prox lienea" << endl;
  offset = p2d_to_pos(msg.offset);
  my_pos = getGVDPos();
  //std::cout << "Consegui my pos: "<<endl;
  pos r_pos = my_pos;

  pos r_segment, c_pos, f_pos;

  int seg;

  boost::tie(seg, gvd) = getGVD(msg.gvd, r_pos);
  
  set_grid();
  pos_set r_set;
  r_set.insert(r_pos);
  //add_to_gvd(r_pos);
  add_to_gvd(r_set);
  //std::cout << "este es el seg: " << seg << endl;
  r_segment = p2d_to_pos(msg.vertex_segment[seg]);
  my_segment = r_segment;
  // std::cout<<"se logro calcular el seg: "<<r_segment.first<<","<<r_segment.first<<endl;

  pos_set criticals;
  for (int i = 0; i < msg.criticals.size(); i++) {
    criticals.insert(p2d_to_pos(msg.criticals[i]));
  }
  //boost::unordered_map<pos,float> paths_costs;

  //ROS_INFO("tiempos arranca el multipath");
  boost::tie(paths, paths_costs) = get_multi_path(gvd,r_pos,criticals);
  //ROS_INFO("tiempos termina el multipath");

  for (int i = 0; i < msg.criticals.size(); i++) {
    segment_bid.criticals.push_back(msg.criticals[i]);

    float cost = paths_costs[p2d_to_pos(msg.criticals[i])] ;
   
    c_pos = p2d_to_pos(msg.criticals[i]);
    //here i should get the frontier
    f_pos = p2d_to_pos(msg.minp_f[i]);
    // std::cout<<"Antes de calclular el camino"<<endl;
    //boost::tie(paths[c_pos], cost) = get_path(gvd, r_pos, c_pos);
    // std::cout<<"Despues de calclular el camino"<<endl;
    // descount factor
    //here i should check if i am on the segment
    bool in_segment = is_in_segment(r_segment, r_pos, c_pos, f_pos);

    float in_seg = 0;
    if (in_segment) {
      in_seg = cost*2;
    }
    // crit a la frontera + (robot al critico)*c
    segment_bid.values.push_back(msg.mind_f[i] + cost - in_seg);
    //segment_bid.values.push_back(cost);
  }
  // std::cout<<"Termino!"<<endl;
  // ROS_INFO("Termino!");
  //ROS_INFO("tiempos termina el getSegmentBid -----------------");
  return segment_bid;
}

geometry_msgs::Point Robot::pos_to_real_p3d(pos p) {
  geometry_msgs::Point p3d = pos_to_p3d(p + offset);
  p3d.x += 0.5;
  p3d.y += 0.5;
  return p3d;
}

//set on the robot variables paths and paths_costs to the frontiers on the array
void Robot::set_my_paths_to_frontiers(vector<tscf_exploration::Point2D> points){
  pos_set f_set;
  for(int i=0; i<points.size(); i++){
    pos f_pos = p2d_to_pos(points[i]);
    //add_to_gvd(f_pos);
    //add_to_gvd(f_pos);
    f_set.insert(f_pos);
  }
  add_to_gvd(f_set);
  
  //add_to_gvd(f_set, assigned_segment);
  //maybe i could recalculate for every frontier to be more exact
  if(true){//assigned_segment == my_segment){
    boost::tie(paths, paths_costs) = get_multi_path(gvd, my_pos, f_set);
    //ROS_INFO("romi calcule el camino a las fronteras");
  }else{
    boost::unordered_map<pos,list<VecGVD::Vertex>> f_paths;
    boost::unordered_map<pos,float> f_paths_costs;
    boost::tie(f_paths, f_paths_costs) = get_multi_path(gvd, assigned_segment, f_set);
    //duplicated information (robot->segment)
    //where should i assigned the path, under key assigned_segment or under frontier?
    for(auto it = f_set.begin(); it != f_set.end(); ++it){
      //paths[assigned_segment].splice(paths[assigned_segment].end(), f_paths[*it]);
      //f_paths[*it].pop_front();
      f_paths[*it].splice(f_paths[*it].begin(), paths[assigned_segment]); 
      //paths[*it] = paths[assigned_segment];
      paths[*it] = f_paths[*it];
      paths_costs[*it] = paths_costs[assigned_segment] + f_paths_costs[*it];
    }
  }
}

int Robot::getRobotId(){
  int id = 0;
  //for(int i = nombreRobot.size()-1; i>= 0; i--){
    //if(!isdigit(nombreRobot[i])){
  return stoi(nombreRobot.substr(4));
    //}
  //}
  //return -1;
}

tscf_exploration::FrontierBid Robot::getFrontierBid(vector<tscf_exploration::Point2D> frontiers){
  set_my_paths_to_frontiers(frontiers);
  tscf_exploration::FrontierBid msg;
  for(auto it = frontiers.begin(); it != frontiers.end(); ++it){
    msg.frontiers.push_back(*it);
    msg.values.push_back(paths_costs[p2d_to_pos(*it)]); 
  }
  msg.robotId = getRobotId();
  msg.id = last_segment_assignment_id;
  last_frontier_auction_id = last_segment_auction_id; 
  return msg;
}


bool Robot::saveFrontierBid(tscf_exploration::FrontierBid fb) {
  // segment_bids[name].clear();
  for (int i = 0; i < fb.frontiers.size(); i++) {
    // segment_bids[name][p2d_to_pos(sb.criticals[i])] = sb.values[i];
    pos f_pos = p2d_to_pos(fb.frontiers[i]);
    string name =  to_string(fb.robotId);
    add_bid(bids_pq, name, f_pos, fb.values[i]);
    //auction_segment_frontiers_num[f_pos] = cis[segment].frontiers.size();
  }
  auction_robots =auction_robots+1;
  return true;
}

pos Robot::assignFrontier() {
  int total_robots = auction_robots;
  int total_frontiers = paths.size();
  string id = to_string(getRobotId());
  boost::unordered_map<string, pos> robot_frontiers =
  resolve_auction(bids_pq, total_robots,total_frontiers);
  return robot_frontiers[id];
}

//First solution, didnot work properly
// Adds extra points so the robot would walk a straight line from current_pos till f_pos(frontier)
void Robot::add_intermidiate_points(pos f_pos, pos current_pos,tscf_exploration::goalList & g_list, float min_dist){ 
  int division_count = dist(f_pos, current_pos)/min_dist;
  //the formula is (x,y) = (x1 + k(x2-x1), y1+k(y2-y1)) 
  for(int i = 1; i < division_count; i++){
    //current_pos.first = current_pos.first + i/division_count*(f_pos.first - current_pos.first);
    //current_pos.second = current_pos.second + i/division_count*(f_pos.second - current_pos.second);
    current_pos = current_pos + (i*(f_pos - current_pos))/division_count; 
    //add to path
    geometry_msgs::Point p3d = pos_to_real_p3d(current_pos);
    g_list.listaGoals.push_back(p3d);
    //ROS_INFO("way intermedio %f,%f,%f", p3d.x, p3d.y, p3d.z);
  }
}

tscf_exploration::goalList Robot::getPathToSegment(pos frontier) {
  
  //ROS_INFO("romi ENTREEEEEEEEEEEEEEEEEE");
  //pos f_pos = p2d_to_pos(frontier);

  //ROS_INFO("tiempos forntera al gvd");
  //add_to_gvd(f_pos);
  //ROS_INFO("tiempos fin forntera al gvd");
  //ROS_INFO("romi ingrese la frontera al gvd");
  
  //ROS_INFO("tiempos voy a calcular el camino");
  //std::list<VecGVD::Vertex> path;
  //float cost; 

  //ROS_INFO("tiempos comienza de calcular el camino");
  //boost::tie(path, cost) = get_single_path(gvd, my_pos, f_pos);
  //ROS_INFO("tiempos termine de calcular el camino");

  if(paths[frontier].size() == 0){
    ROS_WARN("Empty path");
  }

  tscf_exploration::goalList g_list;
  g_list.indice = 1;  // TODO poner bien el indice

  list<VecGVD::Vertex> v_list = paths[frontier];
  geometry_msgs::Point p3d;
  
  //ROS_INFO("romi iterar en los nodos para pasarlos a p3d");

  for (auto it = v_list.begin(); it != v_list.end(); it++) {
    
    p3d = pos_to_real_p3d(gvd.g[*it].p);

    g_list.listaGoals.push_back(p3d);

    //ROS_INFO("romi way puntos en el camino %f,%f,%f", p3d.x, p3d.y, p3d.z);
  }

  //ROS_INFO("TERMINEEEEEEEEEEEEE");
  return g_list;
}

/*void Robot::reset_bid(){
  paths.clear();
  paths_costs.clear();
  clear_bids(bids_pq);
  auction_robots = 0;
}*/

/*void Robot::setErrorCont(int i) {
  Robot::errorCont = i;
};

int CANT_ERRR = 4;

bool Robot::isFinByError() {
  ROS_INFO("Cant Error %s - %d", Robot::nombreRobot.c_str(), Robot::errorCont);
  return (Robot::errorCont >= CANT_ERRR);
};

float Robot::getErrorAverage() {
  return Robot::error_average;
};

void Robot::setErrorAverage(float err) {
  Robot::error_average = err;
};

float Robot::addErrorAverage(int err) {
  Robot::cant_errors = Robot::cant_errors + 1;
  Robot::error_average = Robot::error_average + ((err - Robot::error_average) / Robot::cant_errors);
  return Robot::error_average;
};

void Robot::resetCountError() {
  Robot::cant_errors = 0.0;
};

void Robot::setLastInfoGain(std::vector<int> nom) {
  Robot::last_info_gain = nom;
};

std::vector<int> Robot::getLastInfoGain() {
  return Robot::last_info_gain;
};

std::list<int> Robot::getCentrosF() {
  return Robot::centros_de_frontera;
};

std::string Robot::getRealInfoGain() {
  int cont_libres = 0;
  int cont_ocupadas = 0;
  int cont_totales = 0;
  std::vector<int>::iterator it_puntos;
  // ROS_INFO("width %d", control_map.info.width);
  // ROS_INFO("height %d", control_map.info.height);
  for (it_puntos = last_info_gain.begin(); it_puntos != last_info_gain.end(); it_puntos++) {
    cont_totales++;
    if (Robot::control_map.data[*it_puntos] != -1) {
      if (Robot::control_map.data[*it_puntos] != 100) {
        cont_ocupadas++;
      } else {
        cont_libres++;
      }
    }
  }

  std::stringstream ret;
  // int error = 0;
  int error = cont_totales - (cont_libres + cont_ocupadas);
  //if (error <= 3) {
  //  Robot::errorCont += 1;
  //} else {
    Robot::setErrorCont(0);
  //}
  float errave = Robot::addErrorAverage(error);
  ret << cont_libres << " " << cont_ocupadas << " " << cont_totales << " " << errave;
  return ret.str();
}

void Robot::setCentrosF(std::vector<int> newFrontera) {
  Robot::centros_de_frontera.clear();
  std::vector<int>::iterator it_puntos;
  // ROS_INFO("Guardo %d nuevos centros de frontera", newFrontera.size());
  for (it_puntos = newFrontera.begin(); it_puntos != newFrontera.end(); it_puntos++) {
    if (*it_puntos != -1) {
      // /std::cout << "Centro nuevo  ---> " << *it_puntos;
      Robot::centros_de_frontera.push_back(*it_puntos);
    }
  }
};*/

/*void Robot::saveGlobalMap(nav_msgs::OccupancyGrid msg) {
  // ROS_INFO("Guardo nuevo mapa");
  if (first) {
    Robot::width = msg.info.width;
    Robot::height = msg.info.height;
    Robot::y_origin = msg.info.origin.position.x;
    Robot::x_origin = msg.info.origin.position.y;
    Robot::indice_origen = (abs(Robot::y_origin) * Robot::width) + abs(Robot::x_origin);
    for (int i = 0; i < Robot::width * Robot::height; i++) {
      int fila = i % Robot::width;
      int columna = i / Robot::width;
      float a = ((Robot::x_origin + fila) + 0.5);
      float b = ((Robot::y_origin + columna) + 0.5);
      Robot::map_points[i] = cv::Point2f(a, b);
    }
    Robot::first = false;
  }
  Robot::global_map.info = msg.info;
  Robot::global_map.header = msg.header;
  Robot::global_map.data = msg.data;
};

nav_msgs::OccupancyGrid Robot::getGlobalMap() {
  return Robot::global_map;
};

nav_msgs::OccupancyGrid Robot::getControlMap() {
  return Robot::control_map;
};

void Robot::saveControlMap(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  Robot::control_map.info = msg->info;
  Robot::control_map.header = msg->header;
  Robot::control_map.data = msg->data;
};

//Funcion que me devuelve para cada centro de frontera una oleada
//Funcion que me devuelve para cada centro de frontera las distancias entre
//   dicho centro y todas las celdas, si estas distancias son menores o iguales
//       a la distancia entre dicho centro y robot
std::boost::unordered_map<int, std::vector<int> > Robot::crearOleadas(nav_msgs::OccupancyGrid msg,
                                                     int fin,
                                                     std::list<int> centros_def,
                                                     nav_msgs::OccupancyGrid& p) {
  // ROS_INFO("Creo nueva oleada");
  std::boost::unordered_map<int, std::vector<int> > retorno;
  std::boost::unordered_set<int> nivel_actual;
  std::boost::unordered_set<int> nivel_siguiente;
  // ros::Rate loop_rate(0.1);
  std::list<int>::iterator it;
  for (it = centros_def.begin(); it != centros_def.end(); it++) {
    bool terminar = false;
    std::vector<int> oleada(msg.data.size(), -1);
    nivel_siguiente.insert(*it);
    oleada[*it] = 0;
    while (!nivel_siguiente.empty()) {
      nivel_actual.clear();
      nivel_actual = nivel_siguiente;
      nivel_siguiente.clear();

      std::boost::unordered_set<int>::iterator it_nivel_actual;
      for (it_nivel_actual = nivel_actual.begin(); it_nivel_actual != nivel_actual.end();
           ++it_nivel_actual) {
        for (int i = 0; i < 9; i++) {
          int pos_vecino = posicionRelativa(*it_nivel_actual, i, Robot::width);
          // si esta libre
          if ((msg.data[pos_vecino] == 0) && (i != 4) && (oleada[pos_vecino] == -1)) {
            oleada[pos_vecino] = oleada[*it_nivel_actual] + 1;
            nivel_siguiente.insert(pos_vecino);
          }
        }
      }
      std::boost::unordered_set<int>::iterator buscador;

      buscador = nivel_actual.find(fin);
      if (buscador != nivel_actual.end()) {
        nivel_siguiente.clear();
        // ROS_INFO("%s - Objetivo de oleada  ---> %d, con costo
        // %d",Robot::nombreRobot.c_str(), *it, oleada[fin]);
      }
    }
    std::vector<int>::iterator f;
    retorno[*it] = oleada;
  }
  return retorno;
}

tscf_exploration::frontierReport Robot::consultarCostosInfo(
    std::boost::unordered_map<int, std::vector<int> > oleadas,
    int posicionActual,
    nav_msgs::OccupancyGrid& p) {
  tscf_exploration::frontierReport ret;
  ret.idRobot = nombreRobot;
  ret.infoCentros.clear();
  std::list<int>::iterator it;
  // ROS_INFO("Tengo costo - info");
  for (it = centros_de_frontera.begin(); it != centros_de_frontera.end(); it++) {
    p.data[*it] = 100;
    tscf_exploration::infoCentro inf;
    inf.centro = (*it);
    inf.cost = oleadas[*it][posicionActual];
    // ROS_INFO("InfoCentro  ---> %d , %d",inf.centro, inf.cost );
    ret.infoCentros.push_back(inf);
  }
  ret.cant_centros = ret.infoCentros.size();
  return ret;
}

tscf_exploration::frontierReport Robot::processMap() {
  // Lista de puntos de frontera

  nav_msgs::OccupancyGrid p;
  p.info = global_map.info;
  p.header = global_map.header;
  p.data = global_map.data;

  int posicionActual =
      Robot::indice_origen +
      (((int)Robot::position.x + signo((int)Robot::position.x) * 1) +
       ((int)Robot::position.y) * Robot::width);

  std::boost::unordered_map<int, std::vector<int> > oleadas =
      Robot::crearOleadas(global_map, posicionActual, centros_de_frontera, p);

  tscf_exploration::frontierReport frontRep =
      Robot::consultarCostosInfo(oleadas, posicionActual, p);

  // Robot::printFrontierReport(frontRep);

  return frontRep;
}*/

/* devuelve el objetivo con tu id o sea tu objetivo
int Robot::getobjetive(const tscf_exploration::asignacionConstPtr& msg) {
  int cant = 0;
  int centro = -1;
  bool encontre = false;
  while ((!encontre) && (cant < msg->cantidad)) {
    encontre = msg->asignaciones[cant].idRobot.compare(nombreRobot) == 0;
    if (encontre) {
      centro = msg->asignaciones[cant].objetivo;
      Robot::setLastInfoGain(msg->asignaciones[cant].gain_cels);
    }
    cant++;
  }
  // ROS_INFO(" %s :: Recibo Objetivo  ---> %d", nombreRobot.c_str(), centro);
  return centro;
};*/
/*void Robot::ajustarParedes(int centro, nav_msgs::OccupancyGrid& p, std::vector<int> obstaculos) {
  // ROS_INFO("Ajusto Paredes");
  std::vector<int>::iterator it;
  //Para cada obstaculo o en obstaculos, pone obstaculo en los vecinos de o
  for (it = obstaculos.begin(); it != obstaculos.end(); it++) {
    if (p.data[*it] == 100) {
      for (int j = 0; j < 9; j++) {
        int vecino = posicionRelativa(*it, j, Robot::width);
        if ((j != 4) && (vecino != centro)) {
          global_map.data[vecino] = 100;
        }
      }
    }
  }
}

//Funcion que dada una grilla de costos asociada a un centro de frontera, nos
//devuelve el camino de menor costo
std::list<int> Robot::caminoAfrontera(std::vector<int> oleada,
                                      int obj,
                                      int start,
                                      nav_msgs::OccupancyGrid& p) {
  std::list<int> camino;
  // ros::Rate loop_rate(0.5);
  int actual = start;
  int paso_actual = oleada[start];
  // ROS_INFO("Busco Camino a Frontera");
  while (actual != obj) {
    int candidato = -1;
    int paso_candidato = paso_actual;
    for (int i = 0; i < 9; i++) {
      if (i != 4) {
        int pos_vecino = posicionRelativa(actual, i, Robot::width);
        if (oleada[pos_vecino] != -1) {
          if ((candidato == -1) || (oleada[pos_vecino] < paso_candidato) ||
              ((oleada[pos_vecino] == paso_candidato) &&
               (distacia2Puntos(map_points[actual], map_points[pos_vecino]) <
                distacia2Puntos(map_points[actual], map_points[candidato])))) {
            candidato = pos_vecino;
            paso_candidato = oleada[pos_vecino];
          }
        }
      }
    }
    // ROS_INFO("Paso :::: %d", candidato);
    if (candidato < 0) {
      // ROS_INFO("ALgo salio mal. :::: %d", candidato);
      actual = obj;
    } else {
      actual = candidato;
      p.data[actual] = 100;
      paso_actual = paso_candidato;
      camino.push_back(candidato);
    }
  }
  camino.push_back(obj);
  return camino;
}
//Funcion que calcula los caminos y obtiene el indice de el punto de frontera
// mas cercano
std::boost::unordered_map<int, std::list<int> > Robot::obtenerCaminos(int& camino_mas_cercano,
                                                     std::boost::unordered_map<int, std::vector<int> > oleadas,
                                                     int posicionActual,
                                                     std::list<int> centros_def,
                                                     nav_msgs::OccupancyGrid& p) {
  // ros::Rate loop_rate(0.1);
  std::boost::unordered_map<int, std::list<int> > caminos;
  int costo_mas_cercano = 10000;
  std::list<int>::iterator it;
  for (it = centros_def.begin(); it != centros_def.end(); it++) {
    p.data[*it] = 100;
    caminos[*it] = Robot::caminoAfrontera(oleadas[*it], *it, posicionActual, p);
    int posible_largo = caminos[*it].size();
    if (posible_largo <= costo_mas_cercano) {
      camino_mas_cercano = *it;
      costo_mas_cercano = posible_largo;
    }
  }
  return caminos;
}*/

/*Funcion que a partir de una lista de indices obtiene un goal_path
tscf_exploration::goalList Robot::getGoalPath(std::list<int> list_camino,
                                              nav_msgs::OccupancyGrid& p) {
  ROS_INFO("way otro");
  geometry_msgs::Point p3d = getPosition();
  ROS_INFO("way from p3d %f,%f,%f",p3d.x,p3d.y,p3d.z);
  tscf_exploration::goalList list;
  list.listaGoals.clear();

  std::list<int>::iterator it_camino;
  for (it_camino = list_camino.begin(); it_camino != (--list_camino.end()); it_camino++) {
    cv::Point2f ps = map_points[*it_camino];
    geometry_msgs::Point pos;
    pos.x = ps.x;
    pos.y = ps.y;
    pos.z = 0.0;
    if (!(*it_camino > width * (height - 1)) &&
        ((((global_map.data[*it_camino - 1 + width] == 100) &&
           (global_map.data[*it_camino + width] == 100)) ||
          ((global_map.data[*it_camino + width] == 100) &&
           (global_map.data[*it_camino + 1 + width] == 100))))) {
      pos.y = ps.y - 1.0;
    }
    if (!(*it_camino % width == 0) && ((((global_map.data[*it_camino - 1 + width] == 100) &&
                                         (global_map.data[*it_camino - 1] == 100)) ||
                                        ((global_map.data[*it_camino - 1] == 100) &&
                                         (global_map.data[*it_camino - 1 - width] == 100))))) {
      pos.x = ps.x + 1.0;
    }
    if (!(*it_camino < width) && ((((global_map.data[*it_camino - 1 - width] == 100) &&
                                    (global_map.data[*it_camino - width] == 100)) ||
                                   ((global_map.data[*it_camino - width] == 100) &&
                                    (global_map.data[*it_camino + 1 - width] == 100))))) {
      pos.y = ps.y + 1.0;
    }
    if (!(*it_camino % width == width - 1) &&
        ((((global_map.data[*it_camino + 1 - width] == 100) &&
           (global_map.data[*it_camino + 1] == 100)) ||
          ((global_map.data[*it_camino + 1] == 100) &&
           (global_map.data[*it_camino + 1 + width] == 100))))) {
      pos.x = ps.x - 1.0;
    }
    ROS_INFO("way: %f,%f,%f",pos.x,pos.y,pos.z);
    list.listaGoals.push_back(pos);
    p.data[(*it_camino)] = 100;
  }
  return list;
}

/*centro es el objetivo
tscf_exploration::goalList Robot::getPathToObjetive(int centro,
                                                    std::vector<int> obstaculos,
                                                    nav_msgs::OccupancyGrid& p) {
  p.info = global_map.info;
  p.header = global_map.header;
  p.data = global_map.data;

  int posicionActual =
      Robot::indice_origen +
      (((int)Robot::position.x + signo((int)Robot::position.x) * 1) +
       ((int)Robot::position.y) * Robot::width);

  std::list<int> list;
  list.push_back(centro);
  Robot::ajustarParedes(centro, p, obstaculos);
  std::boost::unordered_map<int, std::vector<int> > oleadas =
      Robot::crearOleadas(global_map, posicionActual, list, p);
  int camino_mas_cercano = -1;
  // ROS_INFO("obtenerCaminos");
  std::boost::unordered_map<int, std::list<int> > caminos =
      Robot::obtenerCaminos(camino_mas_cercano, oleadas, posicionActual, list, p);

  for (std::list<int>::iterator it = caminos[camino_mas_cercano].begin();
       it != caminos[camino_mas_cercano].end(); it++) {
    p.data[(*it)] = 100;
  }
  // ROS_INFO("getGoalPath");
  tscf_exploration::goalList pathlist = getGoalPath(caminos[camino_mas_cercano], p);
  return pathlist;
};*/
