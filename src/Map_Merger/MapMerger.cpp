#include "MapMerger.h"

MapMerger::MapMerger(){
	range = 7;
  init = false;
}

/*Funcion que establece si todos los mapas de los distintos robots son desconocidos o concuerdan en la misma celda.*/
bool MapMerger::sonIgualesCeldas(int ind){
  bool ret = true;
  std::map< std::string, nav_msgs::OccupancyGrid >::iterator iterator = maps_by_robots.begin();
  while((iterator != maps_by_robots.end()) && (-1 != iterator->second.data[ind])){
    iterator++;
  }
  int actual = iterator->second.data[ind];
  while( (ret) && (iterator != maps_by_robots.end())){
    ret = ( ( ret && (actual == iterator->second.data[ind]) ) || ( ret && (-1 == iterator->second.data[ind]) ) );
    iterator++;
  }
  return ret;
}

/*obtengo el indice de la gridmap de la posiion actual*/
int MapMerger::getIndicePosicionActual(float x_ahora, float y_ahora){
  return (MapMerger::MapMerger::indice_origen + (((int)x_ahora + signo((int)x_ahora)) + ((int)y_ahora) * MapMerger::width));
}

/*Guardo la informaci[on del mapa particular para cada rovot]*/
void MapMerger::saveRobotMap(const nav_msgs::OccupancyGridConstPtr& msg,std::string name){
  MapMerger::maps_by_robots[name].info = msg->info;
  MapMerger::maps_by_robots[name].header = msg->header;
  MapMerger::maps_by_robots[name].data = msg->data;
}

/*inicializa el mapa mergeado y toda la info*/
void MapMerger::initMapMerger(const nav_msgs::OccupancyGridConstPtr& msg){
  MapMerger::map_merged.info = msg->info;
  MapMerger::map_merged.header = msg->header;
  MapMerger::map_merged.data = msg->data;

  /*Guardo informacion sobre el oigen y el largo y ancho del mapa QUE ASUMO SON SIEMPRE IGUALES*/
  MapMerger::y_origin = MapMerger::map_merged.info.origin.position.x;
  MapMerger::x_origin = MapMerger::map_merged.info.origin.position.y;
  MapMerger::width = MapMerger::map_merged.info.width;
  MapMerger::height = MapMerger::map_merged.info.height;
  MapMerger::indice_origen = abs(MapMerger::y_origin * MapMerger::width) + abs(MapMerger::x_origin);
  /*inicializo un mapa de puntos donde para cada indice me devuelve el punto (x,y) que le corresponde en un mapa.*/
  for (int i = 0; i < MapMerger::width*MapMerger::height; i++){
    int fila = i % MapMerger::width;;
    int columna = i / MapMerger::width;
    float a = ((MapMerger::x_origin + fila) + 0.5);
    float b = ((MapMerger::y_origin + columna)+0.5);
    MapMerger::map_points[i] = cv::Point2f(a, b);
  }
  init = true;

}

/*Funcion que dado un punto me dice si existe un robot a una distancia menor a la que se le pasa.*/
bool MapMerger::isAnyRobotCloser(float dist, int ind,std::string name){
  std::map< std::string, geometry_msgs::PoseStamped >::iterator it = MapMerger::positions.begin();
  bool hay_mas_cerca = false;

  while ((!hay_mas_cerca) && (it != MapMerger::positions.end())) {
    if (((it->first).compare(name) != 0) && (maps_by_robots[it->first].data[ind] != -1) ) {
      int posicionAlguno = getIndicePosicionActual((it->second).pose.position.x,(it->second).pose.position.y);
      float dist_alguno = distacia2Puntos(map_points[ind], map_points[posicionAlguno]);
      hay_mas_cerca = dist > dist_alguno ;
    }
    it++;
  }
  return hay_mas_cerca;
}

/*Actualizo el mapa mergeado*/
nav_msgs::OccupancyGrid MapMerger::updateMap(const nav_msgs::OccupancyGridConstPtr& msg,std::string name){
  saveRobotMap(msg, name);
  if (!init){
    initMapMerger(msg);
  }else{
    int posicionActual = getIndicePosicionActual(positions[name].pose.position.x, positions[name].pose.position.y);

    for (int i = -range; i < (range +1); i++){
  		for(int j = posicionActual - range*MapMerger::width; j < posicionActual + range*MapMerger::width; j +=MapMerger::width){
  			int ind = i+j;
        if (msg->data[ind] != -1){
          if ( MapMerger::map_merged.data[ind] == -1 ) {
            MapMerger::map_merged.data[ind] = maps_by_robots[name].data[ind];
          }else {
            float dist = distacia2Puntos(map_points[ind], map_points[posicionActual]);
            bool hay_mas_cerca = isAnyRobotCloser(dist, ind, name);
            if (!hay_mas_cerca){
              MapMerger::map_merged.data[ind] = maps_by_robots[name].data[ind];
            }
          }
        }
  		}
  	}
  }

  return MapMerger::map_merged;
}

/*Funcion que indica si un punto e frontera o no, ademas calcula su fila y columna.*/
bool MapMerger::esFrontera(int indice, const nav_msgs::OccupancyGrid msg){
	bool ret = false;

  bool posiciones_adyacentes[9];

	int fila = indice % MapMerger::width;;
	int columna = indice / MapMerger::width;

	if (msg.data[indice] == 0){
		for (int i = 0; i < 9; i++){
			posiciones_adyacentes[i] = true;
		}

		if (fila == (MapMerger::height-1)){
		  for (int i = 6; i < 9; i++){
				posiciones_adyacentes[i] = false;
			}
		}else if (fila == 0){
			for (int i = 0; i < 3; i++){
				posiciones_adyacentes[i] = false;
			}
		}
		if (columna == (MapMerger::width-1)){
			  for (int i = 2; i < 9; i = i+3){
					posiciones_adyacentes[i] = false;
				}
		}else if(columna == 0){
			for (int i = 0; i < 9; i = i+3){
				posiciones_adyacentes[i] = false;
			}
		}
		for (int i = 0; i < 9; i++){
			int pos = posicionRelativa(indice,i, MapMerger::width);
			if (posiciones_adyacentes[i] && (i != 4) ){
				ret = ret || (msg.data[pos] == -1);
			};
		}
	}

	return ret;
}

int MapMerger::updateFrontera(nav_msgs::OccupancyGrid map, std::string name){
	int x_ahora = positions[name].pose.position.x;
	int y_ahora = positions[name].pose.position.y;

	std::set<int> frontera_aux_nueva;
	int posicionActual = MapMerger::indice_origen + (((int)x_ahora + signo((int)x_ahora)*1) + ((int)y_ahora) * MapMerger::width);

	// p.data[posicionActual] = 100;

	std::set<int>::iterator f;
	for (f = MapMerger::frontera.begin(); f != MapMerger::frontera.end(); f++){
		if (MapMerger::esFrontera((*f), map)){
			frontera_aux_nueva.insert(*f);
		}
	}
	MapMerger::frontera.clear();
	for (int i = -8; i < 9; i++){
		for(int j = posicionActual - 8*MapMerger::width; j < posicionActual + 9*MapMerger::width; j +=MapMerger::width){
			int ind = i+j;
			if (MapMerger::esFrontera(ind, map)){
				frontera_aux_nueva.insert(ind);
			}else if (map.data[ind] == 100){
				MapMerger::obstaculos.insert(ind);
			}
		}
	}
	MapMerger::frontera = frontera_aux_nueva;
	return frontera_aux_nueva.size();
}

void MapMerger::updatePose(const geometry_msgs::PoseStampedConstPtr& newPose,std::string name){
  positions[name].header = newPose->header;
	positions[name].pose = newPose->pose;
}

cv::Point2f MapMerger::getPoint2f(int ind){
  return map_points[ind];
}

int MapMerger::getRange(){
	return range;

};

void MapMerger::setRange(int newRange){
	range = newRange;
};

std::vector<int> MapMerger::getFrontera(){
	std::vector<int> ret;
	std::set<int>::iterator f;
	for (f = MapMerger::frontera.begin(); f != MapMerger::frontera.end(); f++){
			ret.push_back(*f);
	}
	return ret;

};

void MapMerger::setFrontera(std::set<int> newFrontera){
	MapMerger::frontera = newFrontera;
};

std::vector<int> MapMerger::getObstaculos(){
	std::vector<int> ret;
	std::set<int>::iterator f;
	for (f = MapMerger::obstaculos.begin(); f != MapMerger::obstaculos.end(); f++){
			ret.push_back(*f);
	}
	return ret;

};

void MapMerger::setObstaculos(std::set<int> newObstaculos){
	MapMerger::obstaculos = newObstaculos;
};

nav_msgs::OccupancyGrid MapMerger::getMap(){
	return MapMerger::map_merged;
};
