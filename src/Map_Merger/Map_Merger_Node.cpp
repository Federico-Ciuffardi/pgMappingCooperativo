#include "MapMerger.h"

/*
 *  Variables
 */

// Topics
/// Subscribers
boost::unordered_map<std::string, ros::Subscriber> map_sub;
boost::unordered_map<std::string, ros::Subscriber> odom_sub;
ros::Subscriber end_sub;

/// Publishers
ros::Publisher map_merged_pub;
ros::Publisher map_controller_pub;

// others
int number_robots;

MapMerger map_merger;

boost::unordered_map<std::string, bool> map_initialization;
boost::unordered_map<std::string, nav_msgs::OccupancyGrid> maps;

std::string end_msg("END");

bool FIN = false;

/*
 *  Functions
 */

// Handlers
void handleNewMap(const nav_msgs::OccupancyGridConstPtr& msg, std::string name) {
  if (!FIN) {
    pgmappingcooperativo::mapMergedInfo info;
    info.mapa = map_merger.updateMap(msg, name);
    info.sizef = map_merger.updateFrontera(map_merger.getMap(), name);
    info.frontera = map_merger.getFrontera();
    info.obstaculos = map_merger.getObstaculos();
    info.sizeo = map_merger.getObstaculos().size();
    map_controller_pub.publish(info.mapa);
    map_merged_pub.publish(info);
  }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &odom, std::string name) {

  // ROS_INFO(" Obtengo nueva pose de  %s", name.c_str());
  if (!FIN) {
    map_merger.updatePose(toPoseStamped(*odom), name);
  }
  // ROS_INFO(" Nueva pose de %s agregada", name.c_str());
}

void handleEnd(const std_msgs::StringConstPtr& msg) {
  std::string str1(msg->data.c_str());
  FIN = (str1.compare(end_msg) == 0);
  if (FIN) {
    ROS_INFO("Stopping MAP MERGER");
  }
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "map_merger");
  ros::NodeHandle n;

  n.param<int>("/starting_robot_number", number_robots, 0);

  // Publishers
  map_merged_pub = n.advertise<pgmappingcooperativo::mapMergedInfo>("/map_merged", 1);
  map_controller_pub = n.advertise<nav_msgs::OccupancyGrid>("/map_controller", 1);
  end_sub = n.subscribe("/end", 1, handleEnd);

  ros::Rate loop_rate(1);
  map_merger = MapMerger();

  ros::master::V_TopicInfo topic_infos;

  while (true) {
    int cont_p3dx_pos = 0;
    int cont_p3dx_map = 0;
    ros::master::getTopics(topic_infos);
    for (ros::master::V_TopicInfo::const_iterator it_topic = topic_infos.begin();
         it_topic != topic_infos.end(); ++it_topic) {
      const ros::master::TopicInfo& published_topic = *it_topic;
      if (published_topic.name.find("/odom") != std::string::npos ) {
        cont_p3dx_pos++;
      }
      /* if (published_topic.name.find("/map") != std::string::npos ) { */
      if (published_topic.name.find("/move_base/global_costmap/costmap") != std::string::npos ) {
        cont_p3dx_map++;
      }
    }
    if (cont_p3dx_pos + cont_p3dx_map >= 2*number_robots){
      ROS_INFO(" MAP MERGER :: all %d ready! ", number_robots);
      break;
    }
    loop_rate.sleep();
    ROS_INFO(" MAP MERGER :: waiting for %d robots ", number_robots - max(cont_p3dx_pos,cont_p3dx_map));
  }

  ros::Duration(5.0).sleep();
  ros::master::getTopics(topic_infos);

  // Subscribed to
  for (ros::master::V_TopicInfo::const_iterator it_topic = topic_infos.begin();
       it_topic != topic_infos.end(); ++it_topic) {
    const ros::master::TopicInfo& published_topic = *it_topic;
    if (published_topic.name.find("/odom") != std::string::npos) {
      std::string nombre = published_topic.name;
      nombre.erase(0, 1);
      int pos = nombre.find('/');
      nombre = nombre.substr(0, pos);
      odom_sub[nombre] = n.subscribe<nav_msgs::Odometry>(
          published_topic.name, 1, boost::bind(&odomCallback, _1, nombre));
      /* std::string map_topic = "/" + nombre + "/map"; */
      std::string map_topic = "/" + nombre + "/move_base/global_costmap/costmap";
      map_initialization[nombre] = false;
      map_sub[nombre] = n.subscribe<nav_msgs::OccupancyGrid>(
          map_topic, 1, boost::bind(&handleNewMap, _1, nombre));
    }
  }

  ros::spin();

  return 0;
}
