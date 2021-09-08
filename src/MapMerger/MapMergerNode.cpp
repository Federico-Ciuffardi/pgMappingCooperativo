#include "MapMerger.h"

///////////////
// Variables //
///////////////

// Topics
/// Subscribers
boost::unordered_map<string, ros::Subscriber> mapSub;
boost::unordered_map<string, ros::Subscriber> odomSub;
ros::Subscriber endSub;

/// Publishers
ros::Publisher mapMergedPub;
ros::Publisher mapControllerPub;

// others
int numberRobots;

MapMerger mapMerger;

boost::unordered_map<string, OccupancyGrid> maps;

string end_msg("END");

bool FIN = false;

///////////////
// CallBacks //
///////////////

void mapCallBack(const OccupancyGridConstPtr& msg, string name) {
  if (FIN) return;

  mapMerger.updateMap(msg, name);

  mapControllerPub.publish(mapMerger.mapMerged);
}

void odomCallback(const Odometry::ConstPtr &odom, string name) {
  if (FIN) return;

  mapMerger.updatePose(toPoseStamped(*odom), name);
}

void endCallBack(const std_msgs::StringConstPtr& msg) {
  string str1(msg->data.c_str());
  FIN = (str1.compare(end_msg) == 0);
  if (FIN) {
    ROS_INFO("Stopping MAP MERGER");
  }
}

//////////
// main //
//////////

int main(int argc, char* argv[]) {
  // Init node
  ros::init(argc, argv, "mapMerger");
  ros::NodeHandle n;

  // Load params

  /// Global params
  FAIL_IFN(n.param<int>("/starting_robot_number", numberRobots, 0));
  FAIL_IFN(n.param<float>("/robot_sensor_range", mapMerger.sensorRange, 0));

  /// MapMerger params
  string robotMapTopic;
  FAIL_IFN(n.param<string>("/robots_map_topic", robotMapTopic, ""));

  // Initilize Publishers
  mapControllerPub = n.advertise<OccupancyGrid>("/map", 1);

  // Initilize Subscribers
  endSub           = n.subscribe("/end", 1, endCallBack);

  // Initilize Publishers/Subscribers for each robot

  /// Wait for the robots to be ready
  ros::Rate loop_rate(1);
  while (true) {
    int readyRobotPos = 0;
    int readyRobotMap = 0;

    ros::master::V_TopicInfo topicInfos; ros::master::getTopics(topicInfos);
    for (ros::master::TopicInfo& publishedTopic : topicInfos) {
      if (publishedTopic.name.find("/odom") != string::npos ) {
        readyRobotPos++;
      }
      if (publishedTopic.name.find(robotMapTopic) != string::npos ) {
        readyRobotMap++;
      }
    }

    if (readyRobotPos + readyRobotMap >= 2*numberRobots){
      ROS_INFO(" MAP MERGER :: all %d ready! ", numberRobots);
      break;
    }

    loop_rate.sleep();
    ROS_INFO(" MAP MERGER :: waiting for %d robots ", numberRobots - max(readyRobotPos,readyRobotMap));
  }

  ros::Duration(5.0).sleep();

  /// Subscribe to robot specific topics
  ros::master::V_TopicInfo topicInfos; ros::master::getTopics(topicInfos);
  for (ros::master::TopicInfo& publishedTopic : topicInfos) {
    if (publishedTopic.name.find("/odom") != string::npos) {
      string topicName = publishedTopic.name;                                  // name = "/robot_name/..."
      string robotName = topicName.erase(0, 1).substr(0, topicName.find('/')); // name = "robot_name"

      odomSub[robotName] = n.subscribe<Odometry>( publishedTopic.name, 1, boost::bind(&odomCallback, _1, robotName));

      string map_topic = "/" + robotName + robotMapTopic;
      mapSub[robotName] = n.subscribe<OccupancyGrid>( map_topic, 1, boost::bind(&mapCallBack, _1, robotName));
    }
  }

  ros::spin();

  return 0;
}
