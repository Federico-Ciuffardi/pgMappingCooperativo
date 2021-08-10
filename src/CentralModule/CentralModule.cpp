#include "CentralModule.h"

//////////////////
// Constructors //
//////////////////
CentralModule::CentralModule() {
  state = WaitingAuction;
  first = true;
  lastSegmentAssignmentId = 0;
  segmentAuctionId = 0;
  sensorRange = 6.0;
  // dist_info_gain_obst = 1.0 / sqrt(2);
}

///////////////
// Get / Set //
///////////////
void CentralModule::setFrontiers(boost::unordered_set<int> newFrontiers) {
  frontiers = newFrontiers;
};

vector<int> CentralModule::getFrontierCenters() {
  return frontierCenters;
};

void CentralModule::setFrontierCenters(vector<int> newFrontierCenters) {
  frontierCenters = newFrontierCenters;
};

centralMouleState CentralModule::getEstado() {
  return state;
};

void CentralModule::setState(centralMouleState newState) {
  state = newState;
};

nav_msgs::OccupancyGrid CentralModule::getMap() {
  return mapMerged;
};

int CentralModule::getNumRobots() {
  return robotNumber;
};

void CentralModule::setNumRobots(int newNumRobots) {
  robotNumber = newNumRobots;
};

///////////
// Other //
///////////

void CentralModule::updateMap(const pgmappingcooperativo::mapMergedInfoConstPtr& newMap) {
  // store occGrid
  nav_msgs::OccupancyGrid occGrid = newMap->mapa;
  if (first) {
    uint width = occGrid.info.width;
    uint height = occGrid.info.height;
    int y_origin = occGrid.info.origin.position.x;
    int x_origin = occGrid.info.origin.position.y;
    for (int i = 0; i < width * height; i++) {
      int fila = i % width;
      int columna = i / width;
      float a = ((x_origin + fila) + 0.5);
      float b = ((y_origin + columna) + 0.5);
      mapPoints[i] = cv::Point2f(a, b);
    }
    first = false;
  }
  mapMerged.info   = occGrid.info;
  mapMerged.header = occGrid.header;
  mapMerged.data   = occGrid.data;

  // store frontiers
  boost::unordered_set<int> set(newMap->frontera.begin(), newMap->frontera.end());
  setFrontiers(set);
}

// Get the info to start an Auction
pgmappingcooperativo::SegmentAuction CentralModule::getSegmentAuctionInfo() {
  // reset the state for a new auction
  cout << "debug :: clear data from previous auction" << endl;
  segmentBids.clear();
  cis.clear();
  clear_bids(bidsPQ);
  auctionSegmentFrontiersNum.clear();
  auctionRobots.clear();

  // Get the map offset (where does the origin of the map on the world coordinate system)
  cout << "debug :: get offset" << endl;
  pgmappingcooperativo::SegmentAuction segment_auction;
  nav_msgs::OccupancyGrid map = getMap();
  segment_auction.offset = p3d_to_p2d(map.info.origin.position);

  // Get the significant frontiers taking into account the vision range of the robots
  cout << "debug :: get significant frontiers" << endl;
  switch (frontierSimplificationMethod) {
    case 0:
      gt = og2gt(map, toVec(frontiers), &cellCount);
      break;
    case 1:
      aplicarKmeans(frontiers);
      gt = og2gt(map, getFrontierCenters(), &cellCount);
      break;
  }

  // criticals_info cis_aux;
  cout << "debug :: gvd and cis" << endl;
  
  if(!topoMap){
    topoMap = new TopoMap(gt);
  }
  topoMap->update();
  cout << "debug :: topoMap updated" << endl;

  cis = topoMap->cis;
  GvdGraph& gvd = *topoMap->gvd->graphGvd;

  cout << "debug :: gvd to rosmsg" << endl;
  GvdGraph::VertexIterator v_it, v_it_end;
  for (boost::tie(v_it, v_it_end) = boost::vertices(gvd.g); v_it != v_it_end; v_it++) {
    segment_auction.gvd.vertices.push_back(pos_to_p2d(gvd.g[*v_it].p));
    segment_auction.vertex_segment.push_back(pos_to_p2d(gvd.g[*v_it].segment));
  }

  GvdGraph::EdgeIterator e_it, e_it_end;
  for (boost::tie(e_it, e_it_end) = boost::edges(gvd.g); e_it != e_it_end; e_it++) {
    pgmappingcooperativo::Edge e;
    e.from = pos_to_p2d(gvd.g[(*e_it).m_source].p);
    e.to = pos_to_p2d(gvd.g[(*e_it).m_target].p);
    segment_auction.gvd.edges.push_back(e);
  }

  cout << "debug :: cis to rosmsg" << endl;
  for (auto it = cis.begin(); it != cis.end(); it++) {
    Pos segment = it->first;
    CriticalInfo segment_info = it->second;

    segment_auction.criticals.push_back(pos_to_p2d(segment));

    for (auto it_f = segment_info.frontiers.begin(); it_f != segment_info.frontiers.end(); ++it_f) {
      segment_auction.frontiers.push_back(pos_to_p2d(*it_f));
      segment_auction.frontiers_segment.push_back(pos_to_p2d(segment));
    }
    // ROS_INFO("critical: %d , %d", segment.first, segment.second);
    segment_auction.mind_f.push_back(segment_info.mindToF);
    // segment_auction.minp_f.push_back(pos_to_p2d(segment_info.frontiers[0]));
    /*for(int i = 0; i < segment_info.frontiers.size(); i++){
      segment_auction.frontier.push_back(pos_to_p2d(segment_info.frontiers[i]));
      segment_auction.frontier_segment.push_back(pos_to_p2d(segment));
    }*/
  }
  segment_auction.id = segmentAuctionId;
  segmentAuctionId++;
  return segment_auction;
}

// Save the bid of a robot (name) for an ongoing auction
bool CentralModule::saveSegmentBid(pgmappingcooperativo::SegmentBid sb, string name) {
  if (lastSegmentAssignmentId != sb.id) {
    return false;
  }
  // segmentBids[name].clear();
  for (int i = 0; i < sb.criticals.size(); i++) {
    segmentBids[name][p2d_to_pos(sb.criticals[i])] = sb.values[i];

    Pos segment = p2d_to_pos(sb.criticals[i]);

    add_bid(bidsPQ, name, segment, sb.values[i]);
    auctionSegmentFrontiersNum[segment] = cis[segment].frontiers.size();
    auctionRobots.insert(name);
  }
  return true;
}

// Resolve the auction given the current bids
boost::unordered_map<string, pgmappingcooperativo::SegmentAssignment> CentralModule::assignSegment() {
  int total_robots = auctionRobots.size();
  int total_segments = cis.size();

  boost::unordered_map<string, Pos> robot_segment =
      resolve_auction(bidsPQ, total_robots, total_segments, &auctionSegmentFrontiersNum);

  boost::unordered_map<Pos, int> robots_nums;
  for (auto it = robot_segment.begin(); it != robot_segment.end(); it++) {
    Pos seg = it->second;
    robots_nums[seg]++;
  }

  // cout<<"termina la sasignacion"<<endl;
  boost::unordered_map<string, pgmappingcooperativo::SegmentAssignment> ret;

  // boost::unordered_map<Pos,pgmappingcooperativo::Point2D> frontier2d;
  for (auto it = auctionRobots.begin(); it != auctionRobots.end(); it++) {
    string r_name = *it;
    pgmappingcooperativo::SegmentAssignment sa;
    sa.id = lastSegmentAssignmentId;
    if (robot_segment.find(r_name) != robot_segment.end()) {
      Pos seg = robot_segment[r_name];
      sa.segment = pos_to_p2d(seg);
      sa.frontiers = pos_to_p2d(cis[seg].frontiers);
      // data for frontier auction
      sa.robots_num = robots_nums[seg];
      sa.assigned = 1;

    } else {
      sa.assigned = 0;
    }
    ret[r_name] = sa;
  }
  lastSegmentAssignmentId++;
  return ret;
}

// TODO Refactor all the code below

/*Funcion que clasifica los puntos de frontiers en clases de equivalencia*/
int CentralModule::dividirFront(boost::unordered_set<int> f, dict_clusters& clusters) {
  int count = 1;
  boost::unordered_set<int> f_copy = f;
  list<int> vecinos;
  boost::unordered_set<int>::iterator it;
  list<int> aux;
  while (!f_copy.empty()) {
    it = f_copy.begin();
    // obtengo el primer elemento de la lista de pendientes
    int actual = *it;
    // lo borro de mi lista de pendientes
    f_copy.erase(it);
    // lo agrego a un clusters
    aux.push_back(actual);
    // actualizo sus vecinos
    for (int i = 1; i < 9; i += 2) {
      int Pos = posicionRelativa(actual, i, mapMerged.info.width);
      boost::unordered_set<int>::iterator itv = f_copy.find(Pos);
      if ((i != 4) && (itv != f_copy.end())) {
        vecinos.push_back(*itv);
        f_copy.erase(itv);
      };
    }
    // mientras sus vecinos no sea vacio
    while (!vecinos.empty()) {
      int primero = vecinos.front();
      for (int i = 1; i < 9; i += 2) {
        int Pos = posicionRelativa(primero, i, mapMerged.info.width);
        boost::unordered_set<int>::iterator itv = f_copy.find(Pos);
        if ((i != 4) && (itv != f_copy.end())) {
          vecinos.push_back(*itv);
          f_copy.erase(itv);
        };
      }
      aux.push_back(primero);
      vecinos.pop_front();
    }
    // if (aux.size() >= 3){
    clusters[count] = aux;
    // }
    aux.clear();
    count++;
  }
  return count - 1;
}

/*Funcion que realiza la asignacion del metodo k means*/
vector<list<int> > CentralModule::asignacionKmean(int k, list<int> puntos, vector<cv::Point2f> centros) {
  list<int>::iterator it_puntos;
  vector<list<int> > puntos_de_centros(k);

  for (it_puntos = puntos.begin(); it_puntos != puntos.end(); it_puntos++) {
    int centro = -1;
    float dist_a_centro = 1000.0;
    /* ROS_DEBUG("Punto ---> (%f , %f) ", mapPoints[(*it_puntos)].x, mapPoints[(*it_puntos)].y); */
    for (int i = 0; i < k; i++) {
      float dist = distacia2Puntos(centros[i], mapPoints[(*it_puntos)]);
      /* ROS_DEBUG("    Distancia a centro %d ---> %f ", i, dist); */
      if (dist_a_centro > dist) {
        centro = i;
        dist_a_centro = dist;
      }
    }
    /* ROS_DEBUG("    Asignado a %d ", centro); */
    puntos_de_centros[centro].push_back(*it_puntos);
  }
  return puntos_de_centros;
}

/*Funcion que realiza la actualizacion de la posicion de los centrso en k-means*/
vector<cv::Point2f> CentralModule::actualizacionKmean(vector<list<int> > puntos_de_centros,
                                                      int cant_centros) {
  list<int>::iterator it_puntos;
  vector<cv::Point2f> centros_nuevos(cant_centros);
  int cont = 0;
  float sum_x = 0.0;
  float sum_y = 0.0;

  for (int i = 0; i < cant_centros; i++) {
    cont = 0;
    sum_x = 0.0;
    sum_y = 0.0;
    /* ROS_DEBUG("   X2 Puntos de %d ", i); */
    for (it_puntos = puntos_de_centros[i].begin(); it_puntos != puntos_de_centros[i].end();
         it_puntos++) {
      /* ROS_DEBUG("      X ---> %f ", mapPoints[(*it_puntos)].x); */
      /* ROS_DEBUG("      Y ---> %f ", mapPoints[(*it_puntos)].y); */
      sum_x += mapPoints[(*it_puntos)].x;
      sum_y += mapPoints[(*it_puntos)].y;
      cont++;
    }
    centros_nuevos[i] = cv::Point2f(sum_x / cont, sum_y / cont);
  }
  return centros_nuevos;
}

/*Funcion que me determina cuando el error en k mean es lo suficientemente chico como para finalizar*/
bool CentralModule::finalizarPorErrorKmean(vector<cv::Point2f> centros_viejos,
                                           vector<cv::Point2f> centros_nuevos,
                                           float dist_lim) {
  bool ret = true;
  int i = 0;
  int lim = centros_viejos.size();
  while ((i < lim) && (ret)) {
    /* ROS_DEBUG("Lugar %d", i); */
    /* ROS_DEBUG("   Diff ---> , %f", distacia2Puntos(centros_viejos[i], centros_nuevos[i])); */
    ret = !(distacia2Puntos(centros_viejos[i], centros_nuevos[i]) >= dist_lim);
    i++;
  }
  return ret;
}

/*Funcion que me lleva los centros obtenidos por kmeans a los puntos de mi frontiers mas cercanos*/
list<int> CentralModule::nearestPoint(vector<cv::Point2f> centros_nuevos, list<int> puntos) {
  list<int> points;

  for (int j = 0; j < centros_nuevos.size(); j++) {
    int te = -1;
    float dist_a_centro = 100.0;
    list<int>::iterator it_puntos;
    for (it_puntos = puntos.begin(); it_puntos != puntos.end(); it_puntos++) {
      float dist = distacia2Puntos(centros_nuevos[j], mapPoints[(*it_puntos)]);

      if (dist_a_centro >= dist) {
        te = (*it_puntos);
        dist_a_centro = dist;
      }
    }
    points.push_back(te);
  }
  return points;
}

/*Funcion que aplica kmean para un grupo de puntos*/
pair<list<int>, vector<list<int> > > CentralModule::kmeans(int k, list<int> puntos, vector<cv::Point2f> centros, float dist_lim) {
  vector<cv::Point2f> centros_viejos;
  vector<cv::Point2f> centros_nuevos = centros;
  vector<list<int> > puntos_de_centros;
  int i = 0;
  do {
    i++;
    /* ROS_DEBUG("Cant. ciclos ---> %d !!", i); */
    centros_viejos.clear();
    centros_viejos = centros_nuevos;
    centros_nuevos.clear();

    puntos_de_centros.clear();
    puntos_de_centros = asignacionKmean(k, puntos, centros_viejos);

    centros_nuevos = actualizacionKmean(puntos_de_centros, centros_viejos.size());

    if (i == 100) {
      ROS_INFO(" nombreRobot :: Maxima cantidad de ciclos alcanzada");
    }

  } while ((i <= 100) && (!finalizarPorErrorKmean(centros_viejos, centros_nuevos, dist_lim)));
  /* ROS_DEBUG(" Ciclos realizados %d", i); */

  pair<list<int>, vector<list<int> > > retorno(nearestPoint(centros_nuevos, puntos),
                                               puntos_de_centros);

  return retorno;
}

/*dadas 2 celdas me indica si son vecinas o no*/
bool CentralModule::esVecino(int celda, int vecino) {
  return (
      (vecino == celda - 1 - mapMerged.info.width) || (vecino == celda - mapMerged.info.width) ||
      (vecino == celda + 1 - mapMerged.info.width) || (vecino == celda - 1) ||
      (vecino == celda + 1) || (vecino == celda - 1 + mapMerged.info.width) ||
      (vecino == celda + mapMerged.info.width) || (vecino == celda + 1 + mapMerged.info.width));
}

/*dada una celda y una lista de celdas, me dice si la celda es vecina de alguna
 * de las celdas de la lista.*/
bool CentralModule::esVecinoDeSet(int celda, boost::unordered_set<int> lista_de_celdas) {
  boost::unordered_set<int>::iterator it = lista_de_celdas.begin();
  bool ret = false;
  while (!ret && (it != lista_de_celdas.end())) {
    ret = esVecino(celda, *it);
    it++;
  }
  return ret;
}

/*Funcion que me devuelve la distancia de un punto p a una recta formada por i y f*/
float CentralModule::distanciaArecta(int inicio, int fin, int punto) {
  float dist = abs(
      ((mapPoints[inicio].y - mapPoints[fin].y) / (mapPoints[inicio].x - mapPoints[fin].x)) * mapPoints[punto].x - mapPoints[punto].y +
      (mapPoints[inicio].y - ((mapPoints[inicio].y - mapPoints[fin].y) / (mapPoints[inicio].x - mapPoints[fin].x)) * mapPoints[inicio].x));
  dist = dist / sqrt(pow(((mapPoints[inicio].y - mapPoints[fin].y) / (mapPoints[inicio].x - mapPoints[fin].x)), 2) + 1);
  return dist;
}

vector<int> CentralModule::aplicarKmeans(boost::unordered_set<int> frontiers) {
  frontierCenters.clear();
  // info_gain.clear();
  dict_clusters clusters;
  int cantidad_fronteras = dividirFront(frontiers, clusters);

  for (int i = 1; i <= cantidad_fronteras; i++) {
    int k = 1;

    vector<cv::Point2f> centros;
    vector<list<int> > puntos_de_centros;
    list<int> centros_nuevo;
    bool frontera_completa = false;
    while (!frontera_completa) {
      // Limpio informacion vieja
      centros.clear();
      centros_nuevo.clear();
      puntos_de_centros.clear();
      // Inicializo los cnetros con un elemento mas, de la misma forma que las
      // inicializaciones anteriores
      list<int>::iterator it = clusters[i].begin();
      for (int pe = 0; pe < k; pe++) {
        centros.push_back(mapPoints[*it]);
        it++;
      }
      // Aplico kmeans
      pair<list<int>, vector<list<int> > > retornoKmean = kmeans(k, clusters[i], centros, 0.1);
      // Obtengo los nuevos centros y puntos de centos.
      centros_nuevo = retornoKmean.first;
      puntos_de_centros = retornoKmean.second;
      list<int>::iterator it_centros = centros_nuevo.begin();
      list<int>::reverse_iterator it_puntos;
      int cont = 0;
      bool fin_iteracion = false;
      while (!fin_iteracion && (it_centros != centros_nuevo.end())) {
        it_puntos = puntos_de_centros[cont].rbegin();
        while (!fin_iteracion && (it_puntos != puntos_de_centros[cont].rend())) {
          fin_iteracion = (distacia2Puntos(mapPoints[*it_puntos], mapPoints[*it_centros]) > 6.0);
          it_puntos++;
        }
        cont++;
        it_centros++;
      }
      if (fin_iteracion) {
        k++;
      } else {
        frontera_completa = true;
        for (it_puntos = centros_nuevo.rbegin(); it_puntos != centros_nuevo.rend(); it_puntos++) {
          // cout<<"centro --->"<<*it_puntos<<endl;
          // boost::unordered_set<int> infoGain = getGainInfo((*it_puntos));
          // info_gain.insert(pair<int, boost::unordered_set<int> >((*it_puntos), infoGain));
          frontierCenters.push_back((*it_puntos));
        }
      }
    }
  }
  return frontierCenters;
}

CentralModule::~CentralModule(){
  delete topoMap;
}
