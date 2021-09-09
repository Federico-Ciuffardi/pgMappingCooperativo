#include "MapMerger.h"
#include "nav_msgs/OccupancyGrid.h"

////////////////
// Parameters //
////////////////

// robotMapTopic:
/// The robots publish their maps on the topic: "[robot_ns]/[robotMapTopic]"
string robotMapTopic = "";

// robotMapUpdateTopic:
/// The robots publish their maps updates on the topic: "[robot_ns]/[robotMapUpdateTopic]"
/// if not specified then the updates are not used
string robotMapUpdateTopic = "";

// minRobotMapsArrived:
/// Each of the expected robots have to send at least [min_robot_maps_arrived] maps to the map merger
int minRobotMapsArrived = 1;

///////////////
// Variables //
///////////////

// Topics
/// Subscribers
boost::unordered_map<string, ros::Subscriber> mapSub;
boost::unordered_map<string, ros::Subscriber> mapUpdateSub;
boost::unordered_map<string, ros::Subscriber> odomSub;
ros::Subscriber endSub;

/// Publishers
ros::Publisher mapPub;
ros::Publisher mapUpdatePub;

// others
int numberRobots;

MapMerger mapMerger;

string end_msg("END");

bool endFlag = false;

bool readyToSend = false;
bool sentFirst = false;

/////////
// Aux //
/////////

bool isReadyToSend(){
  // if readyToSend if false check if it is true, once true it will be true until the end 
  if(!readyToSend && mapMerger.mapsArrived.size() >= numberRobots){
    readyToSend = true;
    for(auto it : mapMerger.mapsArrived){
      int robotMapsArrived = it.second;
      
      readyToSend = it.second >= minRobotMapsArrived;

      if(!readyToSend) break;
    }
  }
  return readyToSend;

}

///////////////
// CallBacks //
///////////////

void mapCallBack(const OccupancyGridConstPtr& msg, string name) {
  if (endFlag) return;

  mapMerger.mergeMap(msg, name);

  if(isReadyToSend()){
    mapPub.publish(mapMerger.mapMerged);
    sentFirst = true;
  }
}

void mapUpdateCallBack(const OccupancyGridUpdateConstPtr& msg, string name) {
  if (endFlag) return;

  OccupancyGridUpdate updateMerged = mapMerger.mergeMapUpdate(msg, name);

  if(isReadyToSend()){
    if(sentFirst){
      mapUpdatePub.publish(updateMerged);
    }else{
      mapPub.publish(mapMerger.mapMerged);
      sentFirst = true;
    }
  }
}

void odomCallback(const Odometry::ConstPtr &odom, string name) {
  if (endFlag) return;

  mapMerger.updatePose(toPoseStamped(*odom), name);
}

void endCallBack(const std_msgs::StringConstPtr& msg) {
  string str1(msg->data.c_str());
  endFlag = (str1.compare(end_msg) == 0);
  if (endFlag) {
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
  FAIL_IFN(n.param<string>("/robots_map_topic", robotMapTopic, robotMapTopic));
  n.param<string>("/robots_map_update_topic", robotMapUpdateTopic, robotMapUpdateTopic);
  n.param<int>("/min_robot_maps_arrived", minRobotMapsArrived, minRobotMapsArrived);
  n.param<float>("/decay", mapMerger.decay, mapMerger.decay);

  // Initilize Publishers
  mapPub = n.advertise<OccupancyGrid>("/map", 1);
  mapUpdatePub = n.advertise<OccupancyGridUpdate>("/map_update", 1);

  // Initilize Subscribers
  endSub           = n.subscribe("/endFlag", 1, endCallBack);

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

  /// Subscribe to robot specific topics
  ros::master::V_TopicInfo topicInfos; ros::master::getTopics(topicInfos);
  for (ros::master::TopicInfo& publishedTopic : topicInfos) {
    if (publishedTopic.name.find("/odom") != string::npos) {
      string topicName = publishedTopic.name;                                  // name = "/robot_name/..."
      string robotName = topicName.erase(0, 1).substr(0, topicName.find('/')); // name = "robot_name"

      odomSub[robotName] = n.subscribe<Odometry>( publishedTopic.name, 1, boost::bind(&odomCallback, _1, robotName));

      string mapTopic = "/" + robotName + "/" + robotMapTopic;
      mapSub[robotName] = n.subscribe<OccupancyGrid>( mapTopic, 1, boost::bind(&mapCallBack, _1, robotName));

      if( robotMapUpdateTopic != ""){
        string mapUpdateTopic = "/" + robotName + "/" + robotMapUpdateTopic;
        mapUpdateSub[robotName] = n.subscribe<OccupancyGridUpdate>( mapUpdateTopic, 1, boost::bind(&mapUpdateCallBack, _1, robotName));
      }
    }
  }

  ros::spin();

  return 0;
}
