#include "../GVD/GVD.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <tscf_exploration/asignacion.h>
#include <tscf_exploration/asignacionCelda.h>
#include <tscf_exploration/frontierReport.h>
#include <tscf_exploration/goalList.h>
#include <tscf_exploration/infoCentro.h>
#include <tscf_exploration/resumenInstancia.h>
#include <tscf_exploration/takeobjetive.h>
#include <tscf_exploration/SegmentAuction.h>
#include <tscf_exploration/SegmentBid.h>
#include <tscf_exploration/SegmentAssignment.h>
#include <tscf_exploration/FrontierBid.h>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <list>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <sstream>
#include <string>
#include <vector>
#include "../utils.cpp"
#include "../conversion.cpp"

typedef std::map<int, std::list<int> > dict_clusters;

class Robot {
 private:
  geometry_msgs::Point position;

  nav_msgs::OccupancyGrid global_map;
  nav_msgs::OccupancyGrid control_map;
  std::map<int, cv::Point2f> map_points;
  std::list<int> centros_de_frontera;
  std::map<pos,std::list<VecGVD::Vertex>> paths;
  bool first;
  uint width;
  uint height;
  int y_origin;
  int x_origin;
  int indice_origen;
  float sensor_range;
  float lado;
  float error_average;
  int errorCont;
  int cant_errors;
  float dist_info_gain_obst;
  std::vector<int> last_info_gain;
  VecGVD gvd;
  pos my_pos;
  pos my_segment;

  std::map<int, std::vector<int> > crearOleadas(nav_msgs::OccupancyGrid msg,
                                                int fin,
                                                std::list<int> centros_def,
                                                nav_msgs::OccupancyGrid& p);
  tscf_exploration::frontierReport consultarCostosInfo(std::map<int, std::vector<int> > oleadas,
                                                       int posicionActual,
                                                       nav_msgs::OccupancyGrid& p);

 public:
  std::string nombreRobot;
  pos assigned_segment;
  Robot();
  void setPosition(int x, int y);
  geometry_msgs::Point getPosition();
  pos getGVDPos();
  void setNombre(std::string nom);
  std::string getNombre();
  void setCentrosF(std::vector<int> cdf);
  std::list<int> getCentrosF();
  void savePose(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void saveGlobalMap(nav_msgs::OccupancyGrid msg);
  nav_msgs::OccupancyGrid getGlobalMap();
  nav_msgs::OccupancyGrid getControlMap();
  void saveControlMap(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  tscf_exploration::frontierReport processMap();
  int getobjetive(const tscf_exploration::asignacionConstPtr& msg);

  tscf_exploration::SegmentBid getSegmentBid(tscf_exploration::SegmentAuction msg);

  tscf_exploration::goalList getPathToObjetive(int centro,
                                               std::vector<int> obstaculos,
                                               nav_msgs::OccupancyGrid& p);
  
  tscf_exploration::goalList getPathToSegment(tscf_exploration::Point2D segment);

  void ajustarParedes(int centro, nav_msgs::OccupancyGrid& p, std::vector<int> obstaculos);
  std::list<int> caminoAfrontera(std::vector<int> oleada,
                                 int obj,
                                 int start,
                                 nav_msgs::OccupancyGrid& p);
  tscf_exploration::goalList getGoalPath(std::list<int> list_camino, nav_msgs::OccupancyGrid& p);
  std::map<int, std::list<int> > obtenerCaminos(int& camino_mas_cercano,
                                                std::map<int, std::vector<int> > oleadas,
                                                int posicionActual,
                                                std::list<int> centros_def,
                                                nav_msgs::OccupancyGrid& p);
  void setLastInfoGain(std::vector<int> nom);
  std::vector<int> getLastInfoGain();
  std::string getRealInfoGain();
  float getErrorAverage();
  void setErrorAverage(float);
  float addErrorAverage(int);
  void resetCountError();
  void setErrorCont(int);
  bool isFinByError();
};
