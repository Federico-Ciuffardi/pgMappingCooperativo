#include "../lib/GVD/GVD.h"
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
#include "../lib/utils.h"
#include "../lib/conversion.h"
#include "../lib/auction.h"

typedef boost::unordered_map<int, std::list<int> > dict_clusters;

class Robot {
 private:
  // vars
  ///Properties
  float sensor_range;
  float lado;
  pos offset;

  // Segmentation relatied
  geometry_msgs::Point position;
  boost::unordered_map<pos,list<VecGVD::Vertex>> paths;
  boost::unordered_map<pos,float>> paths_costs;

  //boost::tuple<boost::unordered_map<pos,list<VecGVD::Vertex>> , boost::unordered_map<pos,float>> multi_paths_with_cost;
  VecGVD gvd;
  pos my_pos;
  pos my_segment;

 public:
  Robot();

  std::string nombreRobot;
  pos assigned_segment;
  int last_segment_assignment_id = -1;
  bids_priority_queue bids_pq;
  int auction_robots = 0;

  void setPosition(int x, int y);
  geometry_msgs::Point getPosition();
  void savePose(const geometry_msgs::PoseStamped::ConstPtr& msg);

  pos getGVDPos();
  geometry_msgs::Point pos_to_real_p3d(pos p);

  void setNombre(std::string nom);
  std::string getNombre();

  int getRobotId(){;

  void set_my_paths_to_frontieres(tscf_exploration::Point2D[] points);

  bool is_in_segment(pos my_segment, pos my_pos, pos assigned_segment, pos f_pos);
  void add_intermidiate_points(pos f_pos, pos current_pos,tscf_exploration::goalList & g_list, float min_dist);
  void add_to_gvd(pos f_pos);
  boost::tuple<int, VecGVD> getGVD(tscf_exploration::Graph g, pos r_pos);

  tscf_exploration::SegmentBid getSegmentBid(tscf_exploration::SegmentAuction msg);
  tscf_exploration::FrontierBid getFrontierBid(tscf_exploration::Point2D[] frontiers);
  bool saveFrontierBid(tscf_exploration::FrontierBid fb);
  pos assignFrontier();
  tscf_exploration::goalList getPathToSegment(pos p);
  void reset_bid();
};

  //nav_msgs::OccupancyGrid global_map;
  //nav_msgs::OccupancyGrid control_map;
  //std::boost::unordered_map<int, cv::Point2f> map_points;
  //std::list<int> centros_de_frontera;

  //uint width;
  //uint height;
  //int y_origin;
  //int x_origin;
  //int indice_origen;

  //float error_average;
  //int errorCont;
  //int cant_errors;
  //float dist_info_gain_obst;
  //std::vector<int> last_info_gain;
  //bool first;
  /*std::boost::unordered_map<int, std::vector<int> > crearOleadas(nav_msgs::OccupancyGrid msg,
                                              int fin,
                                              std::list<int> centros_def,
                                              nav_msgs::OccupancyGrid& p);
tscf_exploration::frontierReport consultarCostosInfo(std::boost::unordered_map<int, std::vector<int> > oleadas,
                                                      int posicionActual,
                                                      nav_msgs::OccupancyGrid& p);*/

  /*void ajustarParedes(int centro, nav_msgs::OccupancyGrid& p, std::vector<int> obstaculos);
  std::list<int> caminoAfrontera(std::vector<int> oleada,
                                 int obj,
                                 int start,
                                 nav_msgs::OccupancyGrid& p);
  tscf_exploration::goalList getGoalPath(std::list<int> list_camino, nav_msgs::OccupancyGrid& p);
  std::boost::unordered_map<int, std::list<int> > obtenerCaminos(int& camino_mas_cercano,
                                                std::boost::unordered_map<int, std::vector<int> > oleadas,
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
  bool isFinByError();*/


  /*tscf_exploration::goalList getPathToObjetive(int centro,
                                               std::vector<int> obstaculos,
                                               nav_msgs::OccupancyGrid& p);*/
    //void saveGlobalMap(nav_msgs::OccupancyGrid msg);
  //nav_msgs::OccupancyGrid getGlobalMap();
  //nav_msgs::OccupancyGrid getControlMap();
  //void saveControlMap(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  //tscf_exploration::frontierReport processMap();
  //int getobjetive(const tscf_exploration::asignacionConstPtr& msg);
   //void setCentrosF(std::vector<int> cdf);
  //std::list<int> getCentrosF();