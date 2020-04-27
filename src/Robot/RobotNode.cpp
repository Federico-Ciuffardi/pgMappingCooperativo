#include "Robot.h"
#include <ros/ros.h>
#include <ctime>
#include <cstdlib>

Robot robot;

ros::Subscriber pose_sub;
ros::Subscriber take_obj_sub;
ros::Subscriber path_result_sub;
ros::Subscriber objetive_sub;
ros::Subscriber _map_sub;
ros::Subscriber coverage_sub;
ros::Subscriber end_sub;


ros::Publisher debug_pub;
ros::Publisher end_pub;
ros::Publisher bid_pub;
ros::Publisher request_objetive_pub;
ros::Publisher goalPath_pub;
ros::Publisher robot_debug_pub;
ros::Publisher coverage_report_pub;
ros::Publisher end_robots_pub;

bool FIN = false;
std::string end_msg("END");

void handlePose(const geometry_msgs::PoseStamped::ConstPtr& msg){
	robot.savePose(msg);
}

void handleControlMap(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	//ROS_INFO("guardo mapa");
	robot.saveControlMap(msg);
}

/* cuando: recibo puntos de interes (objetivos) */
/* que: Valorarlos y enviarlos a la central*/
void handleObjetiveSolicitation(const tscf_exploration::takeobjetiveConstPtr& msg){
	if (!FIN){
		int indice = msg->indice;
		robot.saveGlobalMap(msg->mapa);
		robot.setCentrosF(msg->centrosf);
		tscf_exploration::frontierReport report = robot.processMap();
		report.indice = indice;
		bid_pub.publish(report);
		ROS_INFO("%s :: make bid", robot.getNombre().c_str());
	}
}

void handlePathSucced(const std_msgs::String::ConstPtr& msg){
	ROS_INFO("Entro handlePathSucced");
	if (!FIN){
		std_msgs::String msg_request;
		std::stringstream ss;
		std::string ret = robot.getRealInfoGain();
		ss << robot.getNombre() << msg->data.c_str() << " " << ret;
		if (robot.isFinByError()){
			std_msgs::String msg_request2;
			std::stringstream ss7;
			ss7 << robot.getNombre();
			msg_request2.data = ss7.str();
			end_robots_pub.publish(msg_request2);
			FIN = true;
			ROS_INFO("%s MUERO", robot.getNombre().c_str() );
		}
		msg_request.data = ss.str();
		request_objetive_pub.publish(msg_request);
		//ROS_INFO("Salgo handlePathSucced");
	}
}

void handleEnd(const std_msgs::StringConstPtr& msg){
	std::string str1 (msg->data.c_str());
	FIN = (str1.compare(end_msg) == 0);
	if (FIN){
		std_msgs::String msg_request2;
		std::stringstream ss2;
		ss2 << "END";
		msg_request2.data = ss2.str();
		end_pub.publish(msg_request2);
	}
}

void handleCoverage(const std_msgs::StringConstPtr& msg){
	std::string str1 (msg->data.c_str());
	std_msgs::String msg_request2;
	std::stringstream ss2;
	ss2 << robot.getNombre() << " " << str1 << " " << robot.getErrorAverage();
	msg_request2.data = ss2.str();
	coverage_report_pub.publish(msg_request2);
}

void handleObjetive(const tscf_exploration::asignacionConstPtr& msg){
	if (!FIN){
		int centro = robot.getobjetive(msg);
		tscf_exploration::goalList path;
		if (centro != -1){
			nav_msgs::OccupancyGrid p;
			path = robot.getPathToObjetive(centro, msg->obstaculos, p);
			robot_debug_pub.publish(p);
			ROS_INFO("%s :: Publico Camino", robot.getNombre().c_str());
		}
		path.indice = msg->indice;//numero de subasta
		goalPath_pub.publish(path);
		ROS_INFO("%s :: Objective handled", robot.getNombre().c_str());
	}
}

int main(int argc, char* argv[]){
	ros::init(argc, argv, "fp_explorer");

	ros::NodeHandle n;
	ros::NodeHandle private_node_handle("~");

	std::string nom = ros::this_node::getNamespace();
	robot.setNombre(nom.erase(0,1));
	int x_ahora = -1;
	int y_ahora = -1;

	private_node_handle.getParam("init_pose_x",x_ahora);
	private_node_handle.getParam("init_pose_y",y_ahora);
	robot.setPosition(x_ahora, y_ahora);
	robot.setErrorAverage(0.0);
	robot.resetCountError();
	// Setear topicos para recibir informacion
	pose_sub = n.subscribe("pose", 1, handlePose);
	take_obj_sub = n.subscribe("/take_obj", 1, handleObjetiveSolicitation);
	path_result_sub = n.subscribe("path_result", 1, handlePathSucced);
	objetive_sub = n.subscribe("/objetive", 1, handleObjetive);
	_map_sub = n.subscribe("/"+ nom + "/map", 1, handleControlMap);
	coverage_sub = n.subscribe("/coverage", 1, handleCoverage);
	end_sub = n.subscribe("/end", 1, handleEnd);

	debug_pub = n.advertise<nav_msgs::OccupancyGrid>("/debug", 1);
	bid_pub = n.advertise<tscf_exploration::frontierReport>("bid", 1);
	request_objetive_pub = n.advertise<std_msgs::String>("/request_objetive", 1);
	goalPath_pub = n.advertise<tscf_exploration::goalList>("goalPath", 1, true);
	end_pub = n.advertise<std_msgs::String>("end", 1);
	end_robots_pub = n.advertise<std_msgs::String>("/end_robots", 1);
	robot_debug_pub = n.advertise<nav_msgs::OccupancyGrid>("debugggg", 1); 
	coverage_report_pub = n.advertise<std_msgs::String>("/coverage_report", 1);

	ros::spin();

	return 0;
}
