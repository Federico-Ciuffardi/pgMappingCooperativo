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

bool unobstructedLine(Pos p1, Pos p2, const OccupancyGridConstPtr& occupancyGrid){
  for (Pos p : discretizeLine(p1,p2)){
    if(p == p2 || p == p1) continue;
    if(occupancyGrid->data[toInt(p,occupancyGrid->info.width)] == 100) return false;
  }
  return true;
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
  Pos robotPos = toPos(robotIntPos,mapMerged.info.width);

  int windowLenght = sensorRange; 
  for (int i = -windowLenght; i < (windowLenght + 1); i++) {
    for (int j = -windowLenght; j < (windowLenght + 1); j++) {
      int ind = robotIntPos + i + j * mapMerged.info.width;

      if (ind >= msg->data.size() || isUnknown(msg->data[ind])) continue; 

      Pos indPos = toPos(ind,mapMerged.info.width); 
      if(!unobstructedLine(robotPos, indPos, msg)) continue;

      if (isUnknown(mapMerged.data[ind])) {
        mapMerged.data[ind] = round(decay*msg->data[ind] + (1-decay)*50);
      } else {
        mapMerged.data[ind] = round(decay*msg->data[ind] + (1-decay)*mapMerged.data[ind]);
      }
    }
  }

  mapsArrived[name]++;
}
