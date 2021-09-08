#include "MapMerger.h"

/////////////////
// Constructor //
/////////////////

MapMerger::MapMerger() {
  init = false;
}

/////////
// Aux //
/////////

bool isUnknown(int8_t data){
  return data == -1;
}

void MapMerger::initMapMerger(const OccupancyGridConstPtr& msg) {
  mapMerged = *msg;

  init = true;
}

bool MapMerger::isAnyRobotCloser(float minDist, int ind, string name) {
  for( auto it : positions){
    string robotName = it.first;
    Point robotPosition = (it.second).pose.position;

    if (((robotName).compare(name) != 0) && !isUnknown(robotMaps[robotName].data[ind])) {

      int otherRobotPosition = toInt(robotPosition, mapMerged.info);

      float dist = toPos(ind, mapMerged.info.width).distanceTo(toPos(otherRobotPosition,mapMerged.info.width));

      if(minDist > dist) return true;
    }

  }
  return false;
}

/////////
// API //
/////////

void MapMerger::updatePose(PoseStamped newPose, string name) {
  positions[name] = newPose;
}

void MapMerger::saveRobotMap(const OccupancyGridConstPtr& msg, string name) {
  robotMaps[name] = *msg;
}

void MapMerger::updateMap(const OccupancyGridConstPtr& msg, string name) {
  saveRobotMap(msg, name);

  if (!init) {
    initMapMerger(msg);
  } else {
    int robotIntPos = toInt(positions[name].pose.position, mapMerged.info);

    for (int i = -sensorRange; i < (sensorRange + 1); i++) {
      for (int j = robotIntPos - sensorRange * mapMerged.info.width; j < robotIntPos + sensorRange * mapMerged.info.width; j += mapMerged.info.width) {
        int ind = i + j;

        if (isUnknown(msg->data[ind])) continue; 

        if (isUnknown(mapMerged.data[ind])) {
          mapMerged.data[ind] = robotMaps[name].data[ind];
        } else {
          float dist = toPos(ind,mapMerged.info.width).distanceTo(toPos(robotIntPos,mapMerged.info.width));
          if (!isAnyRobotCloser(dist, ind, name)) {
            mapMerged.data[ind] = robotMaps[name].data[ind];
          }
        }

      }
    }

  }
}

