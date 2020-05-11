#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <tscf_exploration/asignacion.h>
#include <tscf_exploration/asignacionCelda.h>
#include <tscf_exploration/frontierReport.h>
#include <tscf_exploration/infoCentro.h>
#include <tscf_exploration/mapMergedInfo.h>
#include <tscf_exploration/takeobjetive.h>
#include <iostream>
#include <list>
#include <sstream>
#include <string>
#include <vector>
#include "../utils.cpp"

typedef std::map<int, std::list<int> > dict_clusters;

enum centralMouleState { WaitingAuction = 1, WaitingBids = 2 };

class CentralModule {
 private:
  centralMouleState estado;
  int indice;
  int number_robots;
  bool first;
  int sensor_range;
  float dist_info_gain_obst;
  nav_msgs::OccupancyGrid map_merged;
  std::map<std::string, bool> bids_arrivals;
  std::map<std::string, std::map<int, int> > cost_saved;
  std::map<std::string, std::map<int, std::set<int> > > info_gain_saved;
  std::map<int, std::set<int> > info_gain;
  std::map<std::string, bool> asignations;
  std::map<int, cv::Point2f> map_points;
  std::vector<int> centros_de_frontera;
  std::vector<int> obstaculos;
  std::set<int> frontera;

  void resetArray(std::map<std::string, bool> map);
  tscf_exploration::asignacionCelda getMaxUtility();
  float calcularUtilidad(int info_gain_celda, int cost_celda);
  bool checkMap(std::map<std::string, bool> mymap);
  void setDifference(std::set<int>& set1, std::set<int>& set2);
  void updateInfoGain(tscf_exploration::asignacionCelda info);
  int dividirFront(std::set<int> f, dict_clusters& clusters);
  std::pair<std::list<int>, std::vector<std::list<int> > > kmeans(int k,
                                                                  std::list<int> puntos,
                                                                  std::vector<cv::Point2f> centros,
                                                                  float dist_lim);
  std::vector<std::list<int> > asignacionKmean(int k,
                                               std::list<int> puntos,
                                               std::vector<cv::Point2f> centros);
  std::vector<cv::Point2f> actualizacionKmean(std::vector<std::list<int> > puntos_de_centros,
                                              int cant_centros);
  bool finalizarPorErrorKmean(std::vector<cv::Point2f> centros_viejos,
                              std::vector<cv::Point2f> centros_nuevos,
                              float dist_lim);
  std::list<int> nearestPoint(std::vector<cv::Point2f> centros_nuevos, std::list<int> puntos);
  bool esVecino(int celda, int vecino);
  bool esVecinoDeSet(int celda, std::set<int> lista_de_celdas);
  float distanciaArecta(int inicio, int fin, int punto);

 public:
  // getters and setters
  std::vector<int> getObstaculos();
  void setObstaculos(std::vector<int> newObstaculos);

  std::set<int> getFrontera();
  void setFrontera(std::set<int> newFrontera);

  std::vector<int> getCentrosF();
  void setCentrosF(std::vector<int> newCentrosf);

  centralMouleState getEstado();
  void setEstado(centralMouleState newEstado);

  int getNumRobots();
  void setNumRobots(int newNumRobots);

  // others
  CentralModule();

  void updateMap(const tscf_exploration::mapMergedInfoConstPtr&);

  void resetArrivals();

  void saveMap(const nav_msgs::OccupancyGrid map);
  nav_msgs::OccupancyGrid getMap();
  tscf_exploration::takeobjetive getObjetiveMap();

  void saveBid(const tscf_exploration::frontierReportConstPtr& msg, std::string name);
  tscf_exploration::asignacion assignTasks();

  std::vector<int> aplicarKmeans(std::set<int> frontera);

  std::set<int> getGainInfo(int celda);
};
