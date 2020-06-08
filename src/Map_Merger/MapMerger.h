#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <tscf_exploration/mapMergedInfo.h>
#include <unistd.h>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include "../lib/utils.h"

class MapMerger {
 private:
  int range;
  bool init;
  int y_origin;
  int x_origin;
  uint width;
  uint height;
  int indice_origen;
  boost::unordered_set<int> frontera;
  boost::unordered_set<int> obstaculos;

  nav_msgs::OccupancyGrid map_merged;
  boost::unordered_map<int, cv::Point2f> map_points;
  boost::unordered_map<std::string, geometry_msgs::PoseStamped> positions;
  boost::unordered_map<std::string, bool> map_initialization;
  boost::unordered_map<std::string, nav_msgs::OccupancyGrid> maps_by_robots;

  /*Funcion que establece si todos los mapas de los distintos robots son
   * desconocidos o concuerdan en la misma celda.*/
  bool sonIgualesCeldas(int ind);
  /*obtengo el indice de la gridmap de la posiion actual*/
  int getIndicePosicionActual(float x_ahora, float y_ahora);
  /*Guardo la informacion del mapa particular para cada rovot]*/
  void saveRobotMap(const nav_msgs::OccupancyGridConstPtr& msg, std::string name);
  /*inicializa el mapa mergeado y toda la info*/
  void initMapMerger(const nav_msgs::OccupancyGridConstPtr& msg);
  /*Funcion que dado un punto me dice si existe un robot a una distancia menor a
   * la que se le pasa.*/
  bool isAnyRobotCloser(float dist, int ind, std::string name);
  /*Determina si una celda es frontera*/
  bool esFrontera(int indice, const nav_msgs::OccupancyGrid msg);

 public:
  MapMerger();
  int getRange();
  void setRange(int newRange);
  std::vector<int> getFrontera();
  void setFrontera(boost::unordered_set<int> newFrontera);
  std::vector<int> getObstaculos();
  void setObstaculos(boost::unordered_set<int> newObstaculos);
  nav_msgs::OccupancyGrid updateMap(const nav_msgs::OccupancyGridConstPtr& newMap,
                                    std::string name);
  void updatePose(const geometry_msgs::PoseStampedConstPtr& newPose, std::string name);
  int updateFrontera(nav_msgs::OccupancyGrid map, std::string name);
  nav_msgs::OccupancyGrid getMap();
  cv::Point2f getPoint2f(int ind);
};
