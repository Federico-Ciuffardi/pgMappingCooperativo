#include "Robot.h"
#include <ros/ros.h>
#include <ctime>
#include <cstdlib>

Robot robot;

ros::Subscriber pose_sub;
ros::Subscriber take_obj_sub;
ros::Subscriber path_sub;
ros::Subscriber obj_sub;
ros::Subscriber control_sub;
ros::Subscriber coverage_sub;
ros::Subscriber end_sub;


ros::Publisher debug;
ros::Publisher end_pub;
ros::Publisher frontRep_pub;
ros::Publisher need_obj_pub;
ros::Publisher path_publisher;
ros::Publisher robot_debug_publisher;
ros::Publisher coverage_pub;
ros::Publisher end_robots;

bool FIN = false;
std::string end_msg("END");

void handlePose(const geometry_msgs::PoseStamped::ConstPtr& msg){
			robot.savePose(msg);
}

void handleControlMap(const nav_msgs::OccupancyGrid::ConstPtr& msg){
			robot.saveControlMap(msg);
}

void handleObjetiveSolicitation(const tscf_exploration::takeobjetiveConstPtr& msg){
	if (!FIN){
		////ROS_INFO("Entro handleTakeObjetive");
		int indice = msg->indice;
	  robot.saveGlobalMap(msg->mapa);
		robot.setCentrosF(msg->centrosf);
		tscf_exploration::frontierReport
		report = robot.processMap();
		report.indice = indice;
		frontRep_pub.publish(report);
		////ROS_INFO("Salgo handleTakeObjetive");
	}
}

void handlePathSucced(const std_msgs::String::ConstPtr& msg){
	if (!FIN){
		//ROS_INFO("Entro handlePathSucced");
		std_msgs::String msg_request;
		std::stringstream ss;
		std::string ret = robot.getRealInfoGain();
		ss << robot.getNombre() << msg->data.c_str() << " " << ret;
		if (robot.isFinByError()){
			std_msgs::String msg_request2;
			std::stringstream ss7;
			ss7 << robot.getNombre();
			msg_request2.data = ss7.str();
			end_robots.publish(msg_request2);
			FIN = true;
		  ROS_INFO("MUERO %s", robot.getNombre().c_str() );
		}
		msg_request.data = ss.str();
		need_obj_pub.publish(msg_request);
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
	coverage_pub.publish(msg_request2);
}

void handleObjetive(const tscf_exploration::asignacionConstPtr& msg){
	if (!FIN){
		//ROS_INFO("Entro handleObjetive");
	  int centro = robot.getobjetive(msg);
		tscf_exploration::goalList path;
	  if (centro != -1){
			nav_msgs::OccupancyGrid p;
			path = robot.getPathToObjetive(centro, msg->obstaculos, p);
			robot_debug_publisher.publish(p);
			//ROS_INFO("Publico Camino");
	  }
		path.indice = msg->indice;
		path_publisher.publish(path);
		//ROS_INFO("Salgo handleObjetive");
	}
}

int main(int argc, char* argv[]){
	ros::init(argc, argv, "fp_explorer");

	ros::NodeHandle n;
	ros::NodeHandle private_node_handle("~");

	std::string nom = ros::this_node::getNamespace();
  robot.setNombre(nom.erase(0,2));

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
	path_sub = n.subscribe("path_result", 1, handlePathSucced);
  obj_sub = n.subscribe("/objetive", 1, handleObjetive);
	control_sub = n.subscribe("/"+ nom + "/map", 1, handleControlMap);
	coverage_sub = n.subscribe("/coverage", 1, handleCoverage);
	end_sub = n.subscribe("/end", 1, handleEnd);

  debug = n.advertise<nav_msgs::OccupancyGrid>("/debug", 1);
  frontRep_pub = n.advertise<tscf_exploration::frontierReport>("bid", 1);
  need_obj_pub = n.advertise<std_msgs::String>("/request_objetive", 1);
  path_publisher = n.advertise<tscf_exploration::goalList>("goalPath", 1, true);
  end_pub = n.advertise<std_msgs::String>("end", 1);
	end_robots = n.advertise<std_msgs::String>("/end_robots", 1);
	robot_debug_publisher = n.advertise<nav_msgs::OccupancyGrid>("debugggg", 1);
	coverage_pub = n.advertise<std_msgs::String>("/coverage_report", 1);

  ros::spin();

	return 0;
}
