#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <pgmappingcooperativo/asignacion.h>
#include <pgmappingcooperativo/asignacionCelda.h>
#include <pgmappingcooperativo/frontierReport.h>
#include <pgmappingcooperativo/infoCentro.h>
#include <pgmappingcooperativo/mapMergedInfo.h>
#include <pgmappingcooperativo/takeobjetive.h>
#include <pgmappingcooperativo/SegmentAuction.h>
#include <pgmappingcooperativo/SegmentAssignment.h>

#include <iostream>
#include <list>
#include <sstream>
#include <string>
#include <vector>

#include "../lib/GVD/src/GVD.h"
#include "../lib/utils.h"
#include "../lib/conversion.h"
#include "../lib/rviz.h"
#include "../lib/auction.h"
#include "../lib/graph/gnuplot.h"
#include "../GlobalParameters.h"

typedef boost::unordered_map<int, list<int> > dict_clusters;

enum centralMouleState { WaitingAuction = 1, WaitingBids = 2,WaitingFirstBid=3,Resolving = 4 };

class CentralModule {
 private:
  // vars
  /// state
  centralMouleState estado;
  int segment_auction_id;
  int last_segment_assignment_id;
  bool first;

  /// Parameters
  int number_robots;
  int sensor_range;

  /// Map related
  nav_msgs::OccupancyGrid map_merged;
  boost::unordered_map<int, cv::Point2f> map_points;

  /// frontier related
  boost::unordered_set<int> frontera;
  vector<int> centros_de_frontera;

  /// Segment auction related
  criticals_info cis;
  bids_priority_queue bids_pq;
  boost::unordered_map<Pos,int> auction_segment_frontiers_num;
  boost::unordered_set<string> auction_robots;

  // Functions
  ///Kmeans
  int dividirFront(boost::unordered_set<int> f, dict_clusters& clusters);
  pair<list<int>, vector<list<int> > > kmeans(int k,
                                                                  list<int> puntos,
                                                                  vector<cv::Point2f> centros,
                                                                  float dist_lim);
  vector<list<int> > asignacionKmean(int k,
                                               list<int> puntos,
                                               vector<cv::Point2f> centros);
  vector<cv::Point2f> actualizacionKmean(vector<list<int> > puntos_de_centros,
                                              int cant_centros);
  bool finalizarPorErrorKmean(vector<cv::Point2f> centros_viejos,
                              vector<cv::Point2f> centros_nuevos,
                              float dist_lim);
  list<int> nearestPoint(vector<cv::Point2f> centros_nuevos, list<int> puntos);
  bool esVecino(int celda, int vecino);
  bool esVecinoDeSet(int celda, boost::unordered_set<int> lista_de_celdas);
  float distanciaArecta(int inicio, int fin, int punto);
  vector<int> aplicarKmeans(boost::unordered_set<int> frontera);

  /// Auction

 public:

  //log related
  int cell_count = 0;

  boost::unordered_map<string,boost::unordered_map<Pos,float>> segment_bids;
  // getters and setters
  boost::unordered_set<int> getFrontera();
  void setFrontera(boost::unordered_set<int> newFrontera);

  vector<int> getCentrosF();
  void setCentrosF(vector<int> newCentrosf);

  centralMouleState getEstado();
  void setEstado(centralMouleState newEstado);

  int getNumRobots();
  void setNumRobots(int newNumRobots);

  // others
  CentralModule();

  void updateMap(const pgmappingcooperativo::mapMergedInfoConstPtr&);
  void saveMap(const nav_msgs::OccupancyGrid map);
  nav_msgs::OccupancyGrid getMap();

  void reset_bid();
  pgmappingcooperativo::SegmentAuction getSegmentAuctionInfo();
  boost::unordered_map<string,pgmappingcooperativo::SegmentAssignment> assignSegment();
  bool saveSegmentBid( pgmappingcooperativo::SegmentBid msg, string name);
};


  //void resetArray(boost::unordered_map<string, bool> map);
  //pgmappingcooperativo::asignacionCelda getMaxUtility();
  //float calcularUtilidad(int info_gain_celda, int cost_celda);
  //bool checkMap(boost::unordered_map<string, bool> mymap);
  //void setDifference(boost::unordered_set<int>& set1, boost::unordered_set<int>& set2);
  //void updateInfoGain(pgmappingcooperativo::asignacionCelda info);

  //boost::unordered_set<int> getGainInfo(int celda);

  //boost::tuple<pgmappingcooperativo::takeobjetive, GVD> getObjetiveMap();

  //void saveBid(const pgmappingcooperativo::frontierReportConstPtr& msg, string name);
  //void resetArrivals();
  //vector<int> obstaculos
  
  //boost::unordered_map<string, bool> bids_arrivals;
  //boost::unordered_map<string, boost::unordered_map<int, int> > cost_saved;
  //boost::unordered_map<string, boost::unordered_map<int, boost::unordered_set<int> > > info_gain_saved;

  //boost::unordered_map<int, boost::unordered_set<int> > info_gain;
  //boost::unordered_map<string, bool> asignations;
    //float dist_info_gain_obst;

   //vector<int> getObstaculos();
  //void setObstaculos(vector<int> newObstaculos);

  //pgmappingcooperativo::asignacion assignTasks();
