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

#include "../lib/GVD/src/TopoMap.h"
#include "../lib/utils.h"
#include "../lib/conversion.h"
#include "../lib/rviz.h"
#include "../lib/auction.h"
#include "../lib/graph/gnuplot.h"
#include "../GlobalParameters.h"

typedef boost::unordered_map<int, list<int> > dict_clusters;

enum centralMouleState { WaitingAuction = 1, WaitingBids = 2,WaitingFirstBid=3,Resolving = 4 };

class CentralModule {
 public:
  ////////////////
  // Parameters //
  ////////////////

  // frontierSimplificationMethod:
  // * 0: No frontier simplification
  // * 1: Frontier is clustered into significant frontiers considering the robot sensor range
  int frontierSimplificationMethod = 1;

 private:
  //////////
  // vars //
  //////////

  // state
  centralMouleState state;
  int segmentAuctionId;
  int lastSegmentAssignmentId;
  bool first;

  // Problem parameters
  int robotNumber;
  int sensorRange;

  // Map related
  StateGrid gt;
  nav_msgs::OccupancyGrid mapMerged;
  boost::unordered_map<int, cv::Point2f> mapPoints;

  // frontier related
  boost::unordered_set<int> frontiers;
  vector<int> frontierCenters;

  // Segment auction related
  CriticalInfos cis;
  bids_priority_queue bidsPQ;
  boost::unordered_map<Pos,int> auctionSegmentFrontiersNum;
  boost::unordered_set<string> auctionRobots;

  ///////////////
  // Functions //
  ///////////////

  //k-means
  int dividirFront(boost::unordered_set<int> f, dict_clusters& clusters);
  pair<list<int>, vector<list<int> > > kmeans(int k, list<int> puntos, vector<cv::Point2f> centros, float dist_lim);
  vector<list<int> > asignacionKmean(int k, list<int> puntos, vector<cv::Point2f> centros);
  vector<cv::Point2f> actualizacionKmean(vector<list<int> > puntos_de_centros, int cant_centros);
  bool finalizarPorErrorKmean(vector<cv::Point2f> centros_viejos, vector<cv::Point2f> centros_nuevos, float dist_lim);
  list<int> nearestPoint(vector<cv::Point2f> centros_nuevos, list<int> puntos);
  bool esVecino(int celda, int vecino);
  bool esVecinoDeSet(int celda, boost::unordered_set<int> lista_de_celdas);
  float distanciaArecta(int inicio, int fin, int punto);
  vector<int> aplicarKmeans(boost::unordered_set<int> frontiers);

 public:

  //////////
  // vars //
  //////////

  // auction related
  TopoMap* topoMap = NULL;
  boost::unordered_map<string,boost::unordered_map<Pos,float>> segmentBids;

  //log related
  int cellCount = 0;

  ///////////////
  // Functions //
  ///////////////

  // Constructors
  CentralModule();

  // getters and setters
  void setFrontiers(boost::unordered_set<int> newFrontiers);

  vector<int> getFrontierCenters();
  void setFrontierCenters(vector<int> newFrontierCenters);

  centralMouleState getEstado();
  void setState(centralMouleState newState);

  int getNumRobots();
  void setNumRobots(int newNumRobots);

  nav_msgs::OccupancyGrid getMap();

  // Other
  void updateMap(const pgmappingcooperativo::mapMergedInfoConstPtr&);
  pgmappingcooperativo::SegmentAuction getSegmentAuctionInfo();
  boost::unordered_map<string,pgmappingcooperativo::SegmentAssignment> assignSegment();
  bool saveSegmentBid( pgmappingcooperativo::SegmentBid msg, string name);

  // Destructor
  ~CentralModule();

};
