#include "../lib/GVD/GVD.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <pgmappingcooperativo/asignacion.h>
#include <pgmappingcooperativo/asignacionCelda.h>
#include <pgmappingcooperativo/frontierReport.h>
#include <pgmappingcooperativo/goalList.h>
#include <pgmappingcooperativo/infoCentro.h>
#include <pgmappingcooperativo/resumenInstancia.h>
#include <pgmappingcooperativo/takeobjetive.h>
#include <pgmappingcooperativo/SegmentAuction.h>
#include <pgmappingcooperativo/SegmentBid.h>
#include <pgmappingcooperativo/SegmentAssignment.h>
#include <pgmappingcooperativo/FrontierBid.h>
#include <pgmappingcooperativo/mapMergedInfo.h>
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
#include "../lib/rviz.h"

typedef boost::unordered_map<int, std::list<int> > dict_clusters;

class Robot {
 private:
  // vars
  ///Properties
  float sensor_range;
  float lado;

  // Segmentation relatied
  geometry_msgs::Point position;
  boost::unordered_map<pos,list<VecGVD::Vertex>> paths;
  boost::unordered_map<pos,float> paths_costs;

  //boost::tuple<boost::unordered_map<pos,list<VecGVD::Vertex>> , boost::unordered_map<pos,float>> multi_paths_with_cost;
  VecGVD gvd;
  pos my_pos;
  pos my_segment;

 public:
  pos offset;
  Robot();

  std::string nombreRobot;
  pos assigned_segment;
  int last_segment_assignment_id = -1;
  int last_segment_auction_id = -1;
  int last_frontier_auction_id = -1;
  bids_priority_queue bids_pq;
  int auction_robots = 0;

  /// Map related
  pgmappingcooperativo::mapMergedInfo map_merged;
  grid_type grid;

  void setPosition(int x, int y);
  geometry_msgs::Point getPosition();
  void savePose(const geometry_msgs::Pose msg);

  pos getGVDPos();
  geometry_msgs::Point pos_to_real_p3d(pos p);

  void setNombre(std::string nom);
  std::string getNombre();

  int getRobotId();

  void set_my_paths_to_frontiers(vector<pgmappingcooperativo::Point2D> points);

  void set_grid();

  bool is_in_segment(pos my_segment, pos my_pos, pos assigned_segment, pos f_pos);
  void add_intermidiate_points(pos f_pos, pos current_pos,pgmappingcooperativo::goalList & g_list, float min_dist);
  void add_to_gvd(pos f_pos);

  void add_to_gvd(pos_set p_set);

  //boost::tuple<int, VecGVD> getGVD(pgmappingcooperativo::Graph g, pos r_pos);
  VecGVD getGVD(pgmappingcooperativo::Graph g, vector<pgmappingcooperativo::Point2D> vertex_segment);

  pgmappingcooperativo::SegmentBid getSegmentBid(pgmappingcooperativo::SegmentAuction msg);
  pgmappingcooperativo::FrontierBid getFrontierBid(vector<pgmappingcooperativo::Point2D> frontiers);
  bool saveFrontierBid(pgmappingcooperativo::FrontierBid fb);
  pos assignFrontier();
  pgmappingcooperativo::goalList getPathToSegment(pos p);
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
pgmappingcooperativo::frontierReport consultarCostosInfo(std::boost::unordered_map<int, std::vector<int> > oleadas,
                                                      int posicionActual,
                                                      nav_msgs::OccupancyGrid& p);*/

  /*void ajustarParedes(int centro, nav_msgs::OccupancyGrid& p, std::vector<int> obstaculos);
  std::list<int> caminoAfrontera(std::vector<int> oleada,
                                 int obj,
                                 int start,
                                 nav_msgs::OccupancyGrid& p);
  pgmappingcooperativo::goalList getGoalPath(std::list<int> list_camino, nav_msgs::OccupancyGrid& p);
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


  /*pgmappingcooperativo::goalList getPathToObjetive(int centro,
                                               std::vector<int> obstaculos,
                                               nav_msgs::OccupancyGrid& p);*/
    //void saveGlobalMap(nav_msgs::OccupancyGrid msg);
  //nav_msgs::OccupancyGrid getGlobalMap();
  //nav_msgs::OccupancyGrid getControlMap();
  //void saveControlMap(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  //pgmappingcooperativo::frontierReport processMap();
  //int getobjetive(const pgmappingcooperativo::asignacionConstPtr& msg);
   //void setCentrosF(std::vector<int> cdf);
  //std::list<int> getCentrosF();
