#include "MapMerger.h"

/////////////////
// Constructor //
/////////////////

MapMerger::MapMerger() {}

/////////
// Aux //
/////////

bool isUnknown(int8_t data){
  return data == -1;
}

/////////
// API //
/////////

void MapMerger::updatePose(PoseStamped newPose, string name) {
  positions[name] = newPose;
}

void MapMerger::updateMap(const OccupancyGridConstPtr& msg, string name) {
  // if this is the first map then initialize the merged map with this map dimensions
  // note that this module assumes that all the robots share the same map dimensions 
  // so it uses the dimensions of the fist map for the merged map mantaind
  if (mapsArrived.empty()) {
    mapMerged.info = msg->info;
    mapMerged.header = msg->header;
    mapMerged.data = vector<int8_t>(msg->data.size(),-1);
  }

  int robotIntPos = toInt(positions[name].pose.position, mapMerged.info);

  int windowLenght = sensorRange; 
  for (int i = -windowLenght; i < (windowLenght + 1); i++) {
    for (int j = -windowLenght; j < (windowLenght + 1); j++) {
      int ind = robotIntPos + i + j * mapMerged.info.width;

      if (ind >= msg->data.size() || isUnknown(msg->data[ind])) continue; 

      if (isUnknown(mapMerged.data[ind])) {
        mapMerged.data[ind] = round(decay*msg->data[ind] + (1-decay)*50);
      } else {
        mapMerged.data[ind] = round(decay*msg->data[ind] + (1-decay)*mapMerged.data[ind]);
      }
    }
  }

  mapsArrived[name]++;
}
