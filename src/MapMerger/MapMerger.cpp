#include "MapMerger.h"
#include <cstdint>
#include "map_msgs/OccupancyGridUpdate.h"

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

bool MapMerger::isInitialized(){
  return !mapsArrived.empty();
}

bool MapMerger::unobstructedLine(Pos p1, Pos p2, const vector<int8_t> &data, int width){
  int threshold = 50;
  for (Pos p : discretizeLine(p1,p2)){
    if(p == p2 || p == p1) continue;
    if(data[toInt(p,width)] >= threshold) return false;
  }
  return true;
}

/////////
// API //
/////////

void MapMerger::updatePose(PoseStamped newPose, string name) {
  positions[name] = newPose;
}

void MapMerger::mergeMap(const OccupancyGridConstPtr& msg, string name) {
  // if this is the first map then initialize the merged map with this map dimensions
  // note that this module assumes that all the robots share the same map dimensions 
  // so it uses the dimensions of the fist map for the merged map mantaind
  if (!isInitialized()) {
    mapMerged.info = msg->info;
    mapMerged.header = msg->header;
    mapMerged.data = vector<int8_t>(msg->data.size(),-1);
  }

  int robotGlobalInd = toInt(positions[name].pose.position, msg->info);
  Pos robotGlobalPos = toPos(positions[name].pose.position, msg->info);

  int windowLenght = sensorRange / msg->info.resolution; 
  for (int i = -windowLenght; i < (windowLenght + 1); i++) {
    for (int j = -windowLenght; j < (windowLenght + 1); j++) {
      int globalInd = robotGlobalInd + i + j * msg->info.width;

      // Skip if the update index is out bounds or the update index points to a value of unknown
      if (globalInd >= msg->data.size() || isUnknown(msg->data[globalInd])) continue; 


      // merge index
      if (isUnknown(mapMerged.data[globalInd])) {
        mapMerged.data[globalInd] = round(decay*msg->data[globalInd] + (1-decay)*50);
      } else {
        // Skip if there is no line of vision from the update index to the robot (on the update grid)
        Pos globalPos = toPos(globalInd, msg->info.width); 
        if(unobstructedLine(robotGlobalPos, globalPos, msg->data, msg->info.width)){ 
          mapMerged.data[globalInd] = round(decay*msg->data[globalInd] + (1-decay)*mapMerged.data[globalInd]);
        }
      }
    }
  }

  mapsArrived[name]++;
}

OccupancyGridUpdate MapMerger::mergeMapUpdate(const OccupancyGridUpdateConstPtr& update, string name) {
  if (!isInitialized()) return OccupancyGridUpdate(); // do not use the update if the complete map is not initialized

  OccupancyGridUpdate updateMerged = *update;

  Pos robotUpdatePos = toPos(positions[name].pose.position,mapMerged.info) - Pos(update->x,update->y);

  for (int updateInd = 0; updateInd < update->data.size(); updateInd++) {

    Pos updatePos = toPos(updateInd, update->width); 
    int globalInd = toInt(Pos(update->x,update->y) + updatePos, mapMerged.info.width);

    // only update if the update index points to a non unknown value 
    if (!isUnknown(update->data[updateInd])){

      // merge index
      if (isUnknown(mapMerged.data[globalInd])) {
        mapMerged.data[globalInd] = round(decay*update->data[updateInd] + (1-decay)*50);
      } else {
        // Skip if there is no line of vision from the update index to the robot (on the update grid)
        if(unobstructedLine(robotUpdatePos, updatePos, update->data, update->width)){
          mapMerged.data[globalInd] = round(decay*update->data[updateInd] + (1-decay)*mapMerged.data[globalInd]);
        }
      }

    }

    updateMerged.data[updateInd] = mapMerged.data[globalInd];
    
  }

  mapsArrived[name]++;

  return updateMerged;
}
