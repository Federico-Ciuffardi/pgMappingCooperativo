#include "MapMerger.h"

MapMerger::MapMerger() {
  init = false;
}

/*obtengo el indice de la gridmap de la posiion actual*/
int MapMerger::getIndicePosicionActual(float x_ahora, float y_ahora) {
  return (indice_origen + (((int)x_ahora + signo((int)x_ahora)) + ((int)y_ahora) * width));
}

/*Guardo la informacion del map particular para cada robot*/
void MapMerger::saveRobotMap(const OccupancyGridConstPtr& msg, std::string name) {
  robotMaps[name] = *msg;
}

/*inicializa el map mergeado y toda la info*/
void MapMerger::initMapMerger(const OccupancyGridConstPtr& msg) {
  map_merged.info = msg->info;
  map_merged.header = msg->header;
  map_merged.data = msg->data;

  /*Guardo informacion sobre el oigen y el largo y ancho del map QUE ASUMO SON SIEMPRE IGUALES*/
  y_origin = map_merged.info.origin.position.x;
  x_origin = map_merged.info.origin.position.y;
  width = map_merged.info.width;
  height = map_merged.info.height;
  indice_origen = (abs(y_origin) * width) + abs(x_origin);
  /*inicializo un map de puntos donde para cada indice me devuelve el punto (x,y) que le corresponde en un map.*/
  for (int i = 0; i < width * height; i++) {
    int fila = i % width;
    ;
    int columna = i / MapMerger::width;
    float a = ((x_origin + fila) + 0.5);
    float b = ((y_origin + columna) + 0.5);
    MapMerger::map_points[i] = cv::Point2f(a, b);
  }
  init = true;
}

/*Funcion que dado un punto me dice si existe un robot a una distancia menor a la que se le pasa.*/
bool MapMerger::isAnyRobotCloser(float dist, int ind, std::string name) {
  boost::unordered_map<std::string, geometry_msgs::PoseStamped>::iterator it = MapMerger::positions.begin();
  bool hay_mas_cerca = false;

  while ((!hay_mas_cerca) && (it != positions.end())) {
    if (((it->first).compare(name) != 0) && (robotMaps[it->first].data[ind] != -1)) {
      int posicionAlguno = getIndicePosicionActual((it->second).pose.position.x, (it->second).pose.position.y);
      float dist_alguno = distacia2Puntos(map_points[ind], map_points[posicionAlguno]);
      hay_mas_cerca = dist > dist_alguno;
    }
    it++;
  }
  return hay_mas_cerca;
}

/*Actualizo el map mergeado*/
OccupancyGrid MapMerger::updateMap(const OccupancyGridConstPtr& msg, std::string name) {
  saveRobotMap(msg, name);

  if (!init) {
    initMapMerger(msg);
  } else {
    int posicionActual = getIndicePosicionActual(positions[name].pose.position.x, positions[name].pose.position.y);

    for (int i = -sensorRange; i < (sensorRange + 1); i++) {
      for (int j = posicionActual - sensorRange * MapMerger::width; j < posicionActual + sensorRange * MapMerger::width; j += MapMerger::width) {
        int ind = i + j;
        if (msg->data[ind] != -1) {
          if (MapMerger::map_merged.data[ind] == -1) {
            MapMerger::map_merged.data[ind] = robotMaps[name].data[ind];
          } else {
            float dist = distacia2Puntos(map_points[ind], map_points[posicionActual]);
            bool hay_mas_cerca = isAnyRobotCloser(dist, ind, name);
            if (!hay_mas_cerca) {
              MapMerger::map_merged.data[ind] = robotMaps[name].data[ind];
            }
          }
        }
      }
    }
  }

  return MapMerger::map_merged;
}

void MapMerger::updatePose(geometry_msgs::PoseStamped newPose, std::string name) {
  positions[name].header = newPose.header;
  positions[name].pose = newPose.pose;
}

cv::Point2f MapMerger::getPoint2f(int ind) {
  return map_points[ind];
}
