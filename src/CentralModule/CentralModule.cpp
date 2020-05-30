#include "CentralModule.h"

CentralModule::CentralModule() {
  CentralModule::estado = WaitingAuction;
  CentralModule::first = true;
  CentralModule::indice = 0;
  CentralModule::number_robots = 3;
  CentralModule::sensor_range = 6.0;
  CentralModule::dist_info_gain_obst = 1.0 / sqrt(2);
  ;
}

void CentralModule::updateMap(const tscf_exploration::mapMergedInfoConstPtr& newMap) {
  saveMap(newMap->mapa);
  setObstaculos(newMap->obstaculos);
  std::set<int> set(newMap->frontera.begin(), newMap->frontera.end());
  setFrontera(set);
}

void CentralModule::resetArray(std::map<std::string, bool> map) {
  for (std::map<std::string, bool>::iterator it = map.begin(); it != map.end(); it++) {
    map[it->first] = false;
  }
}

std::string bttooi(bool b) {
  if (b) {
    return "true";
  } else {
    return "false";
  }
}

void CentralModule::resetArrivals() {
  CentralModule::resetArray(CentralModule::bids_arrivals);
}

std::vector<int> CentralModule::getObstaculos() {
  return obstaculos;
};

void CentralModule::setObstaculos(std::vector<int> newObstaculos) {
  CentralModule::obstaculos = newObstaculos;
};

std::set<int> CentralModule::getFrontera() {
  return frontera;
};

void CentralModule::setFrontera(std::set<int> newFrontera) {
  CentralModule::frontera = newFrontera;
};

std::vector<int> CentralModule::getCentrosF() {
  return CentralModule::centros_de_frontera;
};

void CentralModule::setCentrosF(std::vector<int> newCentrosF) {
  CentralModule::centros_de_frontera = newCentrosF;
};

/* De ocupancy grid a state grid*/
grid_type og2gt(nav_msgs::OccupancyGrid og, vector<int> frontera) {
  uint mapWidth = og.info.width;
  uint mapHeight = og.info.height;
  grid_type res;
  for (int x = 0; x < mapWidth; x++) {
    res.push_back(row_type());
    for (int y = 0; y < mapHeight; y++) {
      cell_type ct = Unknown;
      switch (og.data[y * mapWidth + x]) {
        case 0:
          ct = Free;
          break;
        case 100:
          ct = Occupied;
          break;
        case -1:
          ct = Unknown;
          break;
        default:
          ct = (cell_type)-1;
      }
      res[x].push_back(ct);
    }
  }

  for (auto it = frontera.begin(); it != frontera.end(); it++) {
    int pos = *it;
    res[pos % mapWidth][pos / mapWidth] = Frontier;
  }
  return res;
}

boost::tuple<tscf_exploration::takeobjetive, GVD> CentralModule::getObjetiveMap() {
  tscf_exploration::takeobjetive ret;
  
  ret.mapa = CentralModule::getMap();
  //taking into account the vision range of robot and leaving only significants frontiers 
  CentralModule::aplicarKmeans(CentralModule::frontera);

  grid_type gt = og2gt(ret.mapa, CentralModule::getCentrosF());

  criticals_info cis;
  GVD gvd;
  boost::tie(cis, gvd) = get_points_of_interest(gt);
  

  set<pos> poi;
  std::vector<int> objs;
  for(auto it = cis.begin(); it != cis.end(); it++){
    poi.insert(it->second.frontiers[0]);
  }

  if(poi.size() >= 0){
    info_gain.clear();
    centros_de_frontera.clear();
    for(auto it = poi.begin(); it != poi.end(); it++){
      int odpos = it->second*ret.mapa.info.width + it->first; 
      objs.push_back(odpos);
      std::set<int> infoGain = CentralModule::getGainInfo(odpos);
      info_gain.insert(std::pair<int, std::set<int> >(odpos, infoGain));
      centros_de_frontera.push_back(odpos);
    }

    setCentrosF(objs);
  }

  ret.centrosf = CentralModule::getCentrosF();
  ret.sizecf = CentralModule::getCentrosF().size();

  ret.indice = CentralModule::indice;
  CentralModule::indice++;
  
  return boost::make_tuple(ret, gvd);
}

centralMouleState CentralModule::getEstado() {
  return CentralModule::estado;
};

void CentralModule::setEstado(centralMouleState newEstado) {
  CentralModule::estado = newEstado;
};

nav_msgs::OccupancyGrid CentralModule::getMap() {
  return CentralModule::map_merged;
};

int CentralModule::getNumRobots() {
  return CentralModule::number_robots;
};

void CentralModule::setNumRobots(int newNumRobots) {
  CentralModule::number_robots = newNumRobots;
};

void CentralModule::saveMap(const nav_msgs::OccupancyGrid map) {
  if (first) {
    uint width = map.info.width;
    uint height = map.info.height;
    int y_origin = map.info.origin.position.x;
    int x_origin = map.info.origin.position.y;
    /*inicializo un mapa de puntos donde para cada indice me devuelve el punto
     * (x,y) que le corresponde en un mapa.*/
    for (int i = 0; i < width * height; i++) {
      int fila = i % width;
      int columna = i / width;
      float a = ((x_origin + fila) + 0.5);
      float b = ((y_origin + columna) + 0.5);
      CentralModule::map_points[i] = cv::Point2f(a, b);
    }
    CentralModule::first = false;
  }
  CentralModule::map_merged.info = map.info;
  CentralModule::map_merged.header = map.header;
  CentralModule::map_merged.data = map.data;
}

void CentralModule::saveBid(const tscf_exploration::frontierReportConstPtr& msg, std::string name) {
  if (CentralModule::estado == 2) {
    std::map<int, std::set<int> >::iterator itm;
    // ROS_INFO(" CENTRAL MODULE :: Guardo Informe del robot %s con nombre %s y
    // cantidad de %d centros", msg->idRobot.c_str(),
    // name.c_str(),msg->cant_centros);
    for (int i = 0; i < msg->cant_centros; i++) {
      CentralModule::cost_saved[msg->idRobot][msg->infoCentros[i].centro] =
          msg->infoCentros[i].cost;
      for (itm = info_gain.begin(); itm != info_gain.end(); ++itm)
        if (itm->first != -1) {
          CentralModule::info_gain_saved[msg->idRobot][itm->first] = itm->second;
        }
      // ROS_INFO(" -------------------- centro %d ->, costo %d",
      // msg->infoCentros[i].centro, msg->infoCentros[i].cost);
    }
    CentralModule::bids_arrivals[name] = true;
    CentralModule::asignations[name] = false;
  }
}

float CentralModule::calcularUtilidad(int info_gain_celda, int cost_celda) {
  float flo = ((float)info_gain_celda) / ((float)cost_celda);
  // float flo = (float)(2000.0 - (float)cost_celda);
  return flo;
}

bool CentralModule::checkMap(std::map<std::string, bool> mymap) {
  bool ret = true;
  std::map<std::string, bool>::iterator it = mymap.begin();
  // ROS_INFO(" CENTRAL MODULE :: Check map ");
  while ((ret) && (it != mymap.end())) {
    ret = ret && it->second;
    // ROS_INFO(" CENTRAL MODULE :: ------%s esta %d", it->first.c_str(),
    // it->second);
    ++it;
  }
  // ROS_INFO(" CENTRAL MODULE :: Check map fin");
  return ret;
}
// a set 1 le resto set 2
void CentralModule::setDifference(std::set<int>& set1, std::set<int>& set2) {
  for (std::set<int>::iterator it = set2.begin(); it != set2.end(); it++) {
    std::set<int>::iterator lugar = set1.find(*it);
    if (lugar != set1.end()) {
      set1.erase(lugar);
    }
  }
}

void CentralModule::updateInfoGain(tscf_exploration::asignacionCelda info) {
  std::set<int> s = CentralModule::info_gain_saved[info.idRobot][info.objetivo];

  for (std::map<std::string, std::map<int, std::set<int> > >::iterator it =
           CentralModule::info_gain_saved.begin();
       it != CentralModule::info_gain_saved.end(); it++) {
    it->second.erase(info.objetivo);
    for (std::map<int, std::set<int> >::iterator it3 = it->second.begin(); it3 != it->second.end();
         it3++) {
      if (distacia2Puntos(map_points[it3->first], map_points[info.objetivo]) < 12) {
        setDifference(it3->second, s);
      }
    }
  }
  for (std::map<std::string, std::map<int, int> >::iterator it2 = CentralModule::cost_saved.begin();
       it2 != CentralModule::cost_saved.end(); it2++) {
    it2->second.erase(info.objetivo);
  }
}

tscf_exploration::asignacionCelda CentralModule::getMaxUtility() {
  tscf_exploration::asignacionCelda info;
  info.idRobot = "";
  info.objetivo = -1;
  float best_utility = -1000;
  int best_info_gain_celda;
  int best_cost_celda = 1000;
  int cont = 0;
  std::map<std::string, std::map<int, std::set<int> > >::iterator it;
  for (it = CentralModule::info_gain_saved.begin(); it != CentralModule::info_gain_saved.end();
       it++) {
    std::map<int, std::set<int> >::iterator it_set;
    for (it_set = it->second.begin(); it_set != it->second.end(); it_set++) {
      int info_gain_celda = it_set->second.size();
      int cost_celda = CentralModule::cost_saved[it->first][it_set->first];
      int dos = it_set->first;
      // ROS_INFO("CENTRAL MODULE :: robot %s, InfoG %d, Costo %d, Best info %f,
      // best costo %d, candidato %d, actual %d", (it->first).c_str(),
      // info_gain_celda,cost_celda, best_utility, best_cost_celda, dos,
      // info.objetivo );
      cont++;
      float utility_aux = calcularUtilidad(info_gain_celda, cost_celda);
      if (((utility_aux > best_utility) && (!it->first.compare("") == 0) &&
           (!CentralModule::asignations[info.idRobot])) ||
          ((utility_aux == best_utility) && (best_cost_celda > cost_celda) &&
           (!it->first.compare("") == 0) && (!CentralModule::asignations[info.idRobot]))) {
        best_utility = utility_aux;
        best_info_gain_celda = info_gain_celda;
        best_cost_celda = cost_celda;
        info.info_gain = info_gain_celda;
        info.costo = cost_celda;
        info.idRobot = it->first;
        info.objetivo = it_set->first;
        info.gain_cels = std::vector<int>(it_set->second.begin(), it_set->second.end());
      }
    }
  }
  if (info.objetivo != -1) {
    // ROS_INFO("CENTRAL MODULE :: Candidato Asignado  %s va a %d con COSTO: %d,
    // INFO_G: %d, UTILIDAD %f", (info.idRobot ).c_str(), info.objetivo,
    // best_cost_celda,best_info_gain_celda, best_utility);
  }
  return info;
}

tscf_exploration::asignacion CentralModule::assignTasks() {
  int gain = 0;
  int cont = 0;
  std::vector<tscf_exploration::asignacionCelda> asignacion_todos;
  asignacion_todos.clear();
  CentralModule::resetArray(CentralModule::asignations);

  while ((!checkMap(CentralModule::asignations)) && (gain != -1)) {
    tscf_exploration::asignacionCelda info = getMaxUtility();
    if ((info.objetivo != -1) && (!CentralModule::asignations[info.idRobot])) {
      CentralModule::asignations[info.idRobot] = true;
      updateInfoGain(info);
      info_gain_saved[info.idRobot].clear();
      asignacion_todos.push_back(info);
      cont++;
    }
    gain = info.objetivo;
  }
  tscf_exploration::asignacion asign;
  asign.asignaciones = asignacion_todos;
  asign.cantidad = cont;
  CentralModule::asignations.clear();
  asign.obstaculos = CentralModule::getObstaculos();
  asign.indice = CentralModule::indice - 1;
  return asign;
}

/*Funcion que clasifica los puntos de frontera en clases de equivalencia*/
int CentralModule::dividirFront(std::set<int> f, dict_clusters& clusters) {
  int count = 1;
  std::set<int> f_copy = f;
  std::list<int> vecinos;
  std::set<int>::iterator it;
  std::list<int> aux;
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
      int pos = posicionRelativa(actual, i, map_merged.info.width);
      std::set<int>::iterator itv = f_copy.find(pos);
      if ((i != 4) && (itv != f_copy.end())) {
        vecinos.push_back(*itv);
        f_copy.erase(itv);
      };
    }
    // mientras sus vecinos no sea vacio
    while (!vecinos.empty()) {
      int primero = vecinos.front();
      for (int i = 1; i < 9; i += 2) {
        int pos = posicionRelativa(primero, i, map_merged.info.width);
        std::set<int>::iterator itv = f_copy.find(pos);
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
std::vector<std::list<int> > CentralModule::asignacionKmean(int k,
                                                            std::list<int> puntos,
                                                            std::vector<cv::Point2f> centros) {
  std::list<int>::iterator it_puntos;
  std::vector<std::list<int> > puntos_de_centros(k);

  for (it_puntos = puntos.begin(); it_puntos != puntos.end(); it_puntos++) {
    int centro = -1;
    float dist_a_centro = 1000.0;
    ROS_DEBUG("Punto ---> (%f , %f) ", map_points[(*it_puntos)].x, map_points[(*it_puntos)].y);
    for (int i = 0; i < k; i++) {
      float dist = distacia2Puntos(centros[i], map_points[(*it_puntos)]);
      ROS_DEBUG("    Distancia a centro %d ---> %f ", i, dist);
      if (dist_a_centro > dist) {
        centro = i;
        dist_a_centro = dist;
      }
    }
    ROS_DEBUG("    Asignado a %d ", centro);
    puntos_de_centros[centro].push_back(*it_puntos);
  }
  return puntos_de_centros;
}
/*Funcion que realiza la actualizacion de la posicion de los centrso en k
 * means*/
std::vector<cv::Point2f> CentralModule::actualizacionKmean(
    std::vector<std::list<int> > puntos_de_centros,
    int cant_centros) {
  std::list<int>::iterator it_puntos;
  std::vector<cv::Point2f> centros_nuevos(cant_centros);
  int cont = 0;
  float sum_x = 0.0;
  float sum_y = 0.0;

  for (int i = 0; i < cant_centros; i++) {
    cont = 0;
    sum_x = 0.0;
    sum_y = 0.0;
    ROS_DEBUG("   X2 Puntos de %d ", i);
    for (it_puntos = puntos_de_centros[i].begin(); it_puntos != puntos_de_centros[i].end();
         it_puntos++) {
      ROS_DEBUG("      X ---> %f ", map_points[(*it_puntos)].x);
      ROS_DEBUG("      Y ---> %f ", map_points[(*it_puntos)].y);
      sum_x += map_points[(*it_puntos)].x;
      sum_y += map_points[(*it_puntos)].y;
      cont++;
    }
    centros_nuevos[i] = cv::Point2f(sum_x / cont, sum_y / cont);
  }
  return centros_nuevos;
}
/*Funcion que me determina cuando el error en k mean es lo suficientemente chico
 * como para finalizar*/
bool CentralModule::finalizarPorErrorKmean(std::vector<cv::Point2f> centros_viejos,
                                           std::vector<cv::Point2f> centros_nuevos,
                                           float dist_lim) {
  bool ret = true;
  int i = 0;
  int lim = centros_viejos.size();
  while ((i < lim) && (ret)) {
    ROS_DEBUG("Lugar %d", i);
    ROS_DEBUG("   Diff ---> , %f", distacia2Puntos(centros_viejos[i], centros_nuevos[i]));
    ret = !(distacia2Puntos(centros_viejos[i], centros_nuevos[i]) >= dist_lim);
    i++;
  }
  return ret;
}
/*Funcion que me lleva los centros obtenidos por kmeans a los puntos de mi
 * frontera mas cercanos*/
std::list<int> CentralModule::nearestPoint(std::vector<cv::Point2f> centros_nuevos,
                                           std::list<int> puntos) {
  std::list<int> points;

  for (int j = 0; j < centros_nuevos.size(); j++) {
    int te = -1;
    float dist_a_centro = 100.0;
    std::list<int>::iterator it_puntos;
    for (it_puntos = puntos.begin(); it_puntos != puntos.end(); it_puntos++) {
      float dist = distacia2Puntos(centros_nuevos[j], map_points[(*it_puntos)]);

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
std::pair<std::list<int>, std::vector<std::list<int> > > CentralModule::kmeans(
    int k,
    std::list<int> puntos,
    std::vector<cv::Point2f> centros,
    float dist_lim) {
  std::vector<cv::Point2f> centros_viejos;
  std::vector<cv::Point2f> centros_nuevos = centros;
  std::vector<std::list<int> > puntos_de_centros;
  int i = 0;
  do {
    i++;
    ROS_DEBUG("Cant. ciclos ---> %d !!", i);
    centros_viejos.clear();
    centros_viejos = centros_nuevos;
    centros_nuevos.clear();

    puntos_de_centros.clear();
    puntos_de_centros = CentralModule::asignacionKmean(k, puntos, centros_viejos);

    centros_nuevos = CentralModule::actualizacionKmean(puntos_de_centros, centros_viejos.size());

    if (i == 100) {
      ROS_INFO(" nombreRobot :: Maxima cantidad de ciclos alcanzada");
    }

  } while ((i <= 100) &&
           (!CentralModule::finalizarPorErrorKmean(centros_viejos, centros_nuevos, dist_lim)));
  ROS_DEBUG(" Ciclos realizados %d", i);

  std::pair<std::list<int>, std::vector<std::list<int> > > retorno(
      CentralModule::nearestPoint(centros_nuevos, puntos), puntos_de_centros);

  return retorno;
}
/*dadas 2 celdas me indica si son vecinas o no*/
bool CentralModule::esVecino(int celda, int vecino) {
  return (
      (vecino == celda - 1 - map_merged.info.width) || (vecino == celda - map_merged.info.width) ||
      (vecino == celda + 1 - map_merged.info.width) || (vecino == celda - 1) ||
      (vecino == celda + 1) || (vecino == celda - 1 + map_merged.info.width) ||
      (vecino == celda + map_merged.info.width) || (vecino == celda + 1 + map_merged.info.width));
}
/*dada una celda y una lista de celdas, me dice si la celda es vecina de alguna
 * de las celdas de la lista.*/
bool CentralModule::esVecinoDeSet(int celda, std::set<int> lista_de_celdas) {
  std::set<int>::iterator it = lista_de_celdas.begin();
  bool ret = false;
  while (!ret && (it != lista_de_celdas.end())) {
    ret = CentralModule::esVecino(celda, *it);
    it++;
  }
  return ret;
}
/*Funcion que me devuelve la distancia de un punto p a una recta formada por i y
 * f*/
float CentralModule::distanciaArecta(int inicio, int fin, int punto) {
  float dist = abs(
      ((map_points[inicio].y - map_points[fin].y) / (map_points[inicio].x - map_points[fin].x)) *
          map_points[punto].x -
      map_points[punto].y +
      (map_points[inicio].y -
       ((map_points[inicio].y - map_points[fin].y) / (map_points[inicio].x - map_points[fin].x)) *
           map_points[inicio].x));
  dist = dist / sqrt(pow(((map_points[inicio].y - map_points[fin].y) /
                          (map_points[inicio].x - map_points[fin].x)),
                         2) +
                     1);
  return dist;
}

/*Funcion que obtiene la informacion ganada en una celda.*/
std::set<int> CentralModule::getGainInfo(int celda) {
  // ros::Rate loop_rate(0.5);
  std::map<int, std::pair<std::set<int>, std::set<int> > > obstaculos;
  std::set<int> nivel_actual;
  std::set<int> nivel_siguiente;
  std::set<int> visitadas;
  std::vector<int> info_gain;
  nivel_siguiente.insert(celda);
  int fre = 0;
  while (!nivel_siguiente.empty()) {
    nivel_actual.clear();
    nivel_actual = nivel_siguiente;
    nivel_siguiente.clear();

    std::set<int>::iterator it_nivel_actual;
    for (it_nivel_actual = nivel_actual.begin(); it_nivel_actual != nivel_actual.end();
         ++it_nivel_actual) {
      for (int i = 0; i < 9; i++) {
        int pos_vecino = posicionRelativa(*it_nivel_actual, i, map_merged.info.width);
        if (visitadas.find(pos_vecino) == visitadas.end()) {
          visitadas.insert(pos_vecino);
          if (map_merged.data[pos_vecino] == 100) {
            // agregar a mapa de obstaculos
            int key = -1;
            std::map<int, std::pair<std::set<int>, std::set<int> > >::iterator it =
                obstaculos.begin();
            while ((key == -1) && it != obstaculos.end()) {
              if (CentralModule::esVecinoDeSet(pos_vecino, it->second.first)) {
                key = it->first;
              }
              it++;
            }
            if (key == -1) {
              std::pair<std::set<int>, std::set<int> > nuevo;
              nuevo.first.insert(pos_vecino);
              obstaculos[pos_vecino] = nuevo;
            } else {
              obstaculos[key].first.insert(pos_vecino);
            }
          } else {
            int key = -1;
            std::map<int, std::pair<std::set<int>, std::set<int> > >::iterator it =
                obstaculos.begin();
            while ((key == -1) && it != obstaculos.end()) {
              if (CentralModule::esVecinoDeSet(pos_vecino, it->second.second)) {
                key = it->first;
              }
              it++;
            }
            if (key != -1) {
              std::set<int>::iterator it_obst = obstaculos[key].first.begin();
              bool puedo = true;
              while (puedo && (it_obst != obstaculos[key].first.end())) {
                puedo = ((map_merged.data[pos_vecino] == -1) &&
                         (distacia2Puntos(map_points[pos_vecino], map_points[celda]) <=
                          sensor_range)) &&
                        (CentralModule::distanciaArecta(celda, pos_vecino, *it_obst) >
                         dist_info_gain_obst);
                it_obst++;
              }
              if (puedo) {
                info_gain.push_back(pos_vecino);
                nivel_siguiente.insert(pos_vecino);
              } else {
                obstaculos[key].second.insert(pos_vecino);
              }
            } else {
              std::map<int, std::pair<std::set<int>, std::set<int> > >::iterator it =
                  obstaculos.begin();
              while ((key == -1) && it != obstaculos.end()) {
                if (CentralModule::esVecinoDeSet(pos_vecino, it->second.first)) {
                  key = it->first;
                }
                it++;
              }
              if (key != -1) {
                std::set<int>::iterator it_obst = obstaculos[key].first.begin();
                bool puedo = true;
                while (puedo && (it_obst != obstaculos[key].first.end())) {
                  puedo = ((map_merged.data[pos_vecino] == -1) &&
                           (distacia2Puntos(map_points[pos_vecino], map_points[celda]) <=
                            sensor_range)) &&
                          (CentralModule::distanciaArecta(celda, pos_vecino, *it_obst) >
                           dist_info_gain_obst);
                  it_obst++;
                }
                if (puedo) {
                  info_gain.push_back(pos_vecino);
                  nivel_siguiente.insert(pos_vecino);
                } else {
                  obstaculos[key].second.insert(pos_vecino);
                }
              } else {
                if ((map_merged.data[pos_vecino] == -1) &&
                    (distacia2Puntos(map_points[pos_vecino], map_points[celda]) <= sensor_range)) {
                  info_gain.push_back(pos_vecino);
                  nivel_siguiente.insert(pos_vecino);
                }
              }
            }
          }
        }
      }
    }
  }
  std::set<int> rrreee(info_gain.begin(), info_gain.end());
  return rrreee;
}

std::vector<int> CentralModule::aplicarKmeans(std::set<int> frontera) {
  centros_de_frontera.clear();
  info_gain.clear();
  dict_clusters clusters;
  int cantidad_fronteras = CentralModule::dividirFront(frontera, clusters);

  for (int i = 1; i <= cantidad_fronteras; i++) {
    int k = 1;

    std::vector<cv::Point2f> centros;
    std::vector<std::list<int> > puntos_de_centros;
    std::list<int> centros_nuevo;
    bool frontera_completa = false;
    while (!frontera_completa) {
      // Limpio informacion vieja
      centros.clear();
      centros_nuevo.clear();
      puntos_de_centros.clear();
      // Inicializo los cnetros con un elemento mas, de la misma forma que las
      // inicializaciones anteriores
      std::list<int>::iterator it = clusters[i].begin();
      for (int pe = 0; pe < k; pe++) {
        centros.push_back(map_points[*it]);
        it++;
      }
      // Aplico kmeans
      std::pair<std::list<int>, std::vector<std::list<int> > > retornoKmean =
          CentralModule::kmeans(k, clusters[i], centros, 0.1);
      // Obtengo los nuevos centros y puntos de centos.
      centros_nuevo = retornoKmean.first;
      puntos_de_centros = retornoKmean.second;
      std::list<int>::iterator it_centros = centros_nuevo.begin();
      std::list<int>::reverse_iterator it_puntos;
      int cont = 0;
      bool fin_iteracion = false;
      while (!fin_iteracion && (it_centros != centros_nuevo.end())) {
        it_puntos = puntos_de_centros[cont].rbegin();
        while (!fin_iteracion && (it_puntos != puntos_de_centros[cont].rend())) {
          fin_iteracion = (distacia2Puntos(map_points[*it_puntos], map_points[*it_centros]) > 6.0);
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
          ROS_DEBUG("centro ---> %d !!", (*it_puntos));
          std::set<int> infoGain = CentralModule::getGainInfo((*it_puntos));
          info_gain.insert(std::pair<int, std::set<int> >((*it_puntos), infoGain));
          centros_de_frontera.push_back((*it_puntos));
        }
      }
    }
  }
  return centros_de_frontera;
}
