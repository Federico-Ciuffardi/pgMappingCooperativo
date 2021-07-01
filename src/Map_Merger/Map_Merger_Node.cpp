#include "MapMerger.h"
/*
 *  Variables
 */

// Topics
/// Subscribers
boost::unordered_map<std::string, ros::Subscriber> map_sub;
boost::unordered_map<std::string, ros::Subscriber> pose_sub;
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

void handlePose(const geometry_msgs::PoseStamped::ConstPtr& msg, std::string name) {
  // ROS_INFO(" Obtengo nueva pose de  %s", name.c_str());
  if (!FIN) {
    map_merger.updatePose(msg, name);
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

  n.param<int>("/starting_robot_number", number_robots, STARTING_ROBOT_NUMBER);

  // Publishers
  map_merged_pub = n.advertise<pgmappingcooperativo::mapMergedInfo>("/map_merged", 1);
  map_controller_pub = n.advertise<nav_msgs::OccupancyGrid>("/map_controller", 1);
  end_sub = n.subscribe("/end", 1, handleEnd);

  ros::Rate loop_rate(5);
  map_merger = MapMerger();

  int cont_p3dx_ = 0;
  ros::master::V_TopicInfo topic_infos;

  while (cont_p3dx_ < number_robots) {
    cont_p3dx_ = 0;
    ros::master::getTopics(topic_infos);
    for (ros::master::V_TopicInfo::const_iterator it_topic = topic_infos.begin();
         it_topic != topic_infos.end(); ++it_topic) {
      const ros::master::TopicInfo& published_topic = *it_topic;
      if (published_topic.name.find("/pose") != std::string::npos) {
        cont_p3dx_++;
      }
      loop_rate.sleep();
    }
    ROS_INFO(" MAP MERGER esperando por %d robots ", number_robots - cont_p3dx_);
  }

  ros::master::getTopics(topic_infos);

  // Subscribed to
  for (ros::master::V_TopicInfo::const_iterator it_topic = topic_infos.begin();
       it_topic != topic_infos.end(); ++it_topic) {
    const ros::master::TopicInfo& published_topic = *it_topic;
    if (published_topic.name.find("/pose") != std::string::npos) {
      std::string nombre = published_topic.name;
      nombre.erase(0, 1);
      int pos = nombre.find('/');
      nombre = nombre.substr(0, pos);
      pose_sub[nombre] = n.subscribe<geometry_msgs::PoseStamped>(
          published_topic.name, 1, boost::bind(&handlePose, _1, nombre));
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
