#include "CentralModule.h"

using namespace std;

/*
 *  Variables
 */

// Estado 1 = Esperando solicitud, Estado 2 = Esperando ofertas

// Topics
/// Subscribers
map<string, ros::Subscriber> bids;
ros::Subscriber end_sub;
ros::Subscriber map_merged_sub;
ros::Subscriber request_objetive_sub;

/// Publishers
ros::Publisher take_obj_pub;
ros::Publisher objetive_pub;
ros::Publisher obj_pub2;

// others
ros::Timer auctionTimer;

CentralModule centralModule;

bool FIN = false;

int mapsHandled = 0;

string end_msg("END");

/*
 *  Functions
 */

// Auxiliar functions
void startAuction() {
  auctionTimer.stop();
  auctionTimer.setPeriod(ros::Duration(1.0));
  centralModule.resetArrivals();
  centralModule.setEstado(WaitingBids);
  tscf_exploration::takeobjetive ret = centralModule.getObjetiveMap();
  take_obj_pub.publish(ret);  // publica puntos a subastar
  ROS_INFO("CENTRAL MODULE :: auction start");
  auctionTimer.start();
}

// Handlers

/* cuando: auctionTimer timeout */
/* que: ejecucion subasta (asignacion de tareas) y publicacion de resultados */
void timerRoutine(const ros::TimerEvent&) {
  if (centralModule.getEstado() == WaitingBids) {
    centralModule.setEstado(WaitingAuction);
    objetive_pub.publish(centralModule.assignTasks());  // ejecucion subasta (asignacion de
                                                        // tareas) y publicacion de resultados
    ROS_INFO("CENTRAL MODULE :: auction end");
  } else {
    ROS_DEBUG("Wrong triggered");
  }
}

/* cuando: un robot te pide un objetivo */
/* que: inicia una subasta */
void handleRequest(const std_msgs::StringConstPtr& msg) {
  if (!FIN) {
    startAuction();
  }
}

/* cuando: llega un nuevo mapa */
/* que: actualizar mapa si no se estan esperando ofertas ni se termino
                y si es el decimo mapa recibido inicia una subasta*/
void handleNewMap(const tscf_exploration::mapMergedInfoConstPtr& msg) {
  if ((!FIN) && (centralModule.getEstado() != WaitingBids)) {
    /* update map */
    // ROS_INFO("CENTRAL MODULE :: update map");
    centralModule.updateMap(msg);
  }
  mapsHandled++;  // maps handled
  if (mapsHandled == 10) {
    startAuction();
  }
}

void handleEnd(const std_msgs::StringConstPtr& msg) {
  string str1(msg->data.c_str());
  FIN = (str1.compare(end_msg) == 0);
  if (FIN) {
    ROS_INFO("CENTRAL MODULE :: FIN");
    ros::Rate loop_rate(1);
    loop_rate.sleep();
    ros::shutdown();
  }
}

/* cuando: un robot tiene una oferta */
/* que: esta se gurda */
void handleReport(const tscf_exploration::frontierReportConstPtr& msg, string name) {
  if (!FIN) {
    centralModule.saveBid(msg, name);
    ROS_INFO("CENTRAL MODULE :: got bid from %s", name.c_str());
  }
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "map_merger");
  ros::NodeHandle n;

  auctionTimer = n.createTimer(ros::Duration(3.0), timerRoutine, true);

  // Publishers
  take_obj_pub = n.advertise<tscf_exploration::takeobjetive>("/take_obj", 1);
  objetive_pub = n.advertise<tscf_exploration::asignacion>("/objetive", 1);
  obj_pub2 = n.advertise<nav_msgs::OccupancyGrid>("/debbi", 1);

  // Subscribed to
  map_merged_sub = n.subscribe<tscf_exploration::mapMergedInfo>("/map_merged", 1, handleNewMap);
  end_sub = n.subscribe("/end", 1, handleEnd);
  request_objetive_sub = n.subscribe<std_msgs::String>("/request_objetive", 1, &handleRequest);

  ros::Rate loop_rate(1);

  centralModule = CentralModule();

  /// Wait for the robots to be ready
  int cont_atrv = 0;
  ros::master::V_TopicInfo topic_infos;

  while (cont_atrv < centralModule.getNumRobots()) {
    cont_atrv = 0;
    ros::master::getTopics(topic_infos);
    for (ros::master::V_TopicInfo::const_iterator it_topic = topic_infos.begin();
         it_topic != topic_infos.end(); ++it_topic) {
      const ros::master::TopicInfo& published_topic = *it_topic;
      if (published_topic.name.find("/pose") != string::npos) {
        cont_atrv++;
      }
      /* ROS_INFO("CENTRAL MODULE :: %d", cont_atrv); */
      loop_rate.sleep();
    }
    ROS_INFO(" CENTRAL MODULE :: esperando por %d robots ",
             centralModule.getNumRobots() - cont_atrv);
  }

  /// subscribe to robot specific topics
  ros::master::getTopics(topic_infos);

  for (ros::master::V_TopicInfo::const_iterator it_topic = topic_infos.begin();
       it_topic != topic_infos.end(); ++it_topic) {
    const ros::master::TopicInfo& published_topic = *it_topic;
    if (published_topic.name.find("/pose") != string::npos) {
      string nombre = published_topic.name;
      nombre.erase(0, 1);
      int pos = nombre.find('/');
      nombre = nombre.substr(0, pos);
      string rep_topic = "/" + nombre + "/bid";
      bids[nombre] = n.subscribe<tscf_exploration::frontierReport>(
          rep_topic, 1, boost::bind(&handleReport, _1, nombre));
    }
  }

  /* ROS_INFO("CENTRAL MODULE :: Numero %d", centralModule.getNumRobots()); */

  ros::spin();

  return 0;
}
