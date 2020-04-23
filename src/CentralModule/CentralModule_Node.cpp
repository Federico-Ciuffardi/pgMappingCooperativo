#include "CentralModule.h"

//Estado 1 = Esperando solicitud, Estado 2 = Esperando ofertas
int estado = 1;

//Topicos para la comunicacion con ROS
std::map< std::string, ros::Subscriber > bids;
ros::Subscriber end_sub;
ros::Subscriber map_merged_sub;
ros::Subscriber request_objetive_sub;

ros::Publisher take_obj_pub;
ros::Publisher objetive_pub;
ros::Publisher obj_pub2;
ros::Timer timer;

CentralModule centralModule;

bool FIN = false;
int cont = 0;
std::string end_msg("END");

/* cuando: timer timeout */
/* que: ejecucion subasta (asignacion de tareas) y publicacion de resultados */
void timerRoutine(const ros::TimerEvent&){
	ROS_INFO("CENTRAL MODULE :: auction");
	if (centralModule.getEstado() == 2){
		centralModule.setEstado(1); 
		objetive_pub.publish(centralModule.assignTasks()); // ejecucion subasta (asignacion de tareas) y publicacion de resultados
	}else{
		ROS_DEBUG("Wrong triggered");
	}
}

/* cuando: un robot te pide un objetivo */
/* que: inicia una subasta */
void handleRequest(const std_msgs::StringConstPtr& msg){
	if (!FIN){
		timer.stop();
		timer.setPeriod(ros::Duration(1.0));
		centralModule.resetArrivals();
		centralModule.setEstado(2);
		tscf_exploration::takeobjetive ret = centralModule.getObjetiveMap();
		take_obj_pub.publish(ret); // publica puntos a subastar
		timer.start();
	}
}

/* cuando: llega un nuevo mapa */
/* que: actualizar mapa si no se estan esperando ofertas ni se termino
		y si es el decimo mapa recibido inicia una subasta*/
void handleNewMap(const tscf_exploration::mapMergedInfoConstPtr& msg){
	if ((!FIN) && (centralModule.getEstado() != 2)){
		/* update map */
		centralModule.saveMap(msg->mapa);
		centralModule.setObstaculos(msg->obstaculos);
		std::set<int> set(msg->frontera.begin(), msg->frontera.end());
		centralModule.setFrontera(set);
	}
	cont++;//maps handled
	if (cont == 10){
		timer.stop();
		timer.setPeriod(ros::Duration(1.0));
		centralModule.resetArrivals();
		centralModule.setEstado(2);
		take_obj_pub.publish(centralModule.getObjetiveMap());// publica puntos a subastar
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

/* cuando: un robot tiene una oferta */
/* que: esta se gurda */
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

	take_obj_pub = n.advertise<tscf_exploration::takeobjetive>("/take_obj", 1);
	//map_publisher2 = n.advertise<tscf_exploration::takeobjetive>("/info_effective", 1);
	objetive_pub = n.advertise<tscf_exploration::asignacion>("/objetive", 1);
	obj_pub2 = n.advertise<nav_msgs::OccupancyGrid>("/debbi", 1);

	map_merged_sub =  n.subscribe<tscf_exploration::mapMergedInfo>("/map_merged", 1, handleNewMap);
	end_sub = n.subscribe("/end", 1, handleEnd);
	request_objetive_sub = n.subscribe<std_msgs::String>("/request_objetive", 1, &handleRequest);

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
			/* ROS_INFO("CENTRAL MODULE :: %d", cont_atrv); */
			loop_rate.sleep();
		}
		ROS_INFO(" CENTRAL MODULE esperando por %d robots ", centralModule.getNumRobots() - cont_atrv);
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

	/* ROS_INFO("CENTRAL MODULE :: Numero %d", centralModule.getNumRobots()); */

	ros::spin();

	return 0;
}
