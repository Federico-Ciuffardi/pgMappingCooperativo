#include "CentralModule.h"

//Estado 1 = Esperando solicitud, Estado 2 = Esperando ofertas
int estado = 1;

//Topicos para la comunicacion con ROS
std::map< std::string, ros::Subscriber > bids;
ros::Subscriber end_sub;
ros::Subscriber map_merged;
ros::Subscriber request_suscriber;

ros::Publisher map_publisher;
ros::Publisher obj_pub;
ros::Publisher obj_pub2;
ros::Timer timer;

CentralModule centralModule;

bool FIN = false;
int cont = 0;
std::string end_msg("END");


void timerRoutine(const ros::TimerEvent&){
  if (centralModule.getEstado() == 2){
    centralModule.setEstado(1);
    obj_pub.publish(centralModule.assignTasks());
  }else{
    ROS_DEBUG("Wrong triggered");
  }
}

void handleRequest(const std_msgs::StringConstPtr& msg){
  if (!FIN){
    timer.stop();
    timer.setPeriod(ros::Duration(1.0));
    centralModule.resetArrivals();
    centralModule.setEstado(2);
    tscf_exploration::takeobjetive ret = centralModule.getObjetiveMap();
    map_publisher.publish(ret);
    timer.start();
  }
}

void handleNewMap(const tscf_exploration::mapMergedInfoConstPtr& msg){
  if ((!FIN) && (centralModule.getEstado() != 2)){
      centralModule.saveMap(msg->mapa);
      centralModule.setObstaculos(msg->obstaculos);
      std::set<int> set(msg->frontera.begin(), msg->frontera.end());
      centralModule.setFrontera(set);
  }
  cont++;
  if (cont == 10){
    timer.stop();
    timer.setPeriod(ros::Duration(1.0));
    centralModule.resetArrivals();
    centralModule.setEstado(2);
    map_publisher.publish(centralModule.getObjetiveMap());
    timer.start();
  }
}

void handleEnd(const std_msgs::StringConstPtr& msg){
	std::string str1 (msg->data.c_str());
	FIN = (str1.compare(end_msg)==0);
  if (FIN){
    ROS_INFO("CENTRAL MODULE :: FIN");
    ros::Rate loop_rate(1);
    loop_rate.sleep();
    ros::shutdown();
  }
}

void handleReport(const tscf_exploration::frontierReportConstPtr& msg, std::string name){
  if (!FIN){
    ROS_DEBUG("CENTRAL MODULE :: ENTRO handleReport");
    centralModule.saveBid(msg, name);
    ROS_DEBUG("CENTRAL MODULE :: SALGO handleReport");
  }
}

int main(int argc, char* argv[]){
	ros::init(argc, argv, "map_merger");
	ros::NodeHandle n;

  timer = n.createTimer(ros::Duration(3.0), timerRoutine, true);

  map_publisher = n.advertise<tscf_exploration::takeobjetive>("/take_obj", 1);
  //map_publisher2 = n.advertise<tscf_exploration::takeobjetive>("/info_effective", 1);
  obj_pub = n.advertise<tscf_exploration::asignacion>("/objetive", 1);
  obj_pub2 = n.advertise<nav_msgs::OccupancyGrid>("/debbi", 1);

  map_merged =  n.subscribe<tscf_exploration::mapMergedInfo>("/map_merged", 1, handleNewMap);
  end_sub = n.subscribe("/end", 1, handleEnd);
  request_suscriber = n.subscribe<std_msgs::String>("/request_objetive", 1, &handleRequest);

  ros::Rate loop_rate(1);
  centralModule = CentralModule();

  int cont_atrv = 0;
  ros::master::V_TopicInfo topic_infos;

  while(cont_atrv < centralModule.getNumRobots()){
    cont_atrv = 0;
    ros::master::getTopics(topic_infos);
    for(ros::master::V_TopicInfo::const_iterator it_topic = topic_infos.begin(); it_topic != topic_infos.end(); ++it_topic) {
      const ros::master::TopicInfo &published_topic = *it_topic;
      if(published_topic.name.find("/pose") != std::string::npos){
        cont_atrv++;
      }
    ROS_INFO("CENTRAL MODULE :: %d", cont_atrv);
    loop_rate.sleep();
    }
    //ROS_INFO(" CENTRAL MODULE esperando por %d robots ", centralModule.getNumRobots() - cont_atrv);
  }

  ros::master::getTopics(topic_infos);

  for(ros::master::V_TopicInfo::const_iterator it_topic = topic_infos.begin(); it_topic != topic_infos.end(); ++it_topic) {
    const ros::master::TopicInfo &published_topic = *it_topic;
    if(published_topic.name.find("/pose") != std::string::npos){
      std::string nombre = published_topic.name;
      nombre.erase(0,1);
      int pos = nombre.find('/');
      nombre = nombre.substr(0,pos);
      std::string rep_topic = "/" + nombre + "/bid";
      bids[nombre] = n.subscribe<tscf_exploration::frontierReport>(rep_topic, 1, boost::bind(&handleReport, _1, nombre));
    }
  }

  //ROS_INFO("CENTRAL MODULE :: Numero %d", centralModule.getNumRobots());

	ros::spin();

	return 0;
}
