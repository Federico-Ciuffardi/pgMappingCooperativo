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

bool MapMerger::isAnyRobotCloser(Pos pos, string name) {

  Pos robotPos = toPos(positions[name].pose.position, mapMerged.info);
  float minDistSqrt = pos.distanceToSquared(robotPos);

  for( auto it : positions){
    string robotName = it.first;
    Point robotPosition = (it.second).pose.position;

    if (robotName == name && isUnknown(robotMaps[robotName].data[toInt(pos,mapMerged.info.width)])) continue; 

    Pos otherRobotPos = toPos(robotPosition, mapMerged.info);
    float distSqrt = pos.distanceToSquared(otherRobotPos);

    if(minDistSqrt > distSqrt) return true;

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
      for (int j = -sensorRange; j < (sensorRange + 1); j++) {
        int ind = robotIntPos + i + j * mapMerged.info.width;

        if (ind >= msg->data.size() || isUnknown(msg->data[ind])) continue; 

        if (isUnknown(mapMerged.data[ind])) {
          mapMerged.data[ind] = msg->data[ind];
        } else if (!isAnyRobotCloser(toPos(ind, mapMerged.info.width), name)) {
            mapMerged.data[ind] = msg->data[ind];
        }
      }
    }

  }
}

