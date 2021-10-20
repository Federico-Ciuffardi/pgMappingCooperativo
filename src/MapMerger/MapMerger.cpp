#include "MapMerger.h"
#include "../lib/utils.h"
#include <cstdint>
#include "map_msgs/OccupancyGridUpdate.h"

/////////////////
// Constructor //
/////////////////

MapMerger::MapMerger() {}

/////////
// Aux //
/////////

bool MapMerger::isInitialized(){
  return !mapsArrived.empty();
}

void MapMerger::setOccupancy(int index, double occupancy){
  mapMerged.data[index]      = round(100*occupancy);
  mapMergedDoubleData[index] = occupancy;
}

#define P_PRIOR 0.5
#define P_OCC   0.55
#define P_FREE  0.45

void MapMerger::updateOccupancy(int index, int8_t newOccupancy){
  double newProb;
  if ( isUnknown(newOccupancy) ){
    newProb = P_PRIOR;
  }else if (isOccupied(newOccupancy)){
    newProb = P_OCC;
  }else{ // isFree(newOccupancy)
    newProb = P_FREE;
  }

  double currentProb = mapMergedDoubleData[index];
  currentProb = 1/(1 + ((1-newProb)/newProb) * /*(P_PRIOR/(1-P_PRIOR))* */ ((1-currentProb)/currentProb)); // optimized for P_PRIOR = 0.5

  // avoid certainty
  currentProb = min(currentProb, 0.99999);
  currentProb = max(currentProb, 0.00001);

  setOccupancy(index, currentProb);
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
    mapMergedDoubleData = vector<double>(msg->data.size(),-1);
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
        setOccupancy(globalInd, P_PRIOR);
        updateOccupancy(globalInd,msg->data[globalInd]);
        /* mapMerged.data[globalInd] = round(decay*msg->data[globalInd] + (1-decay)*50); */
      } else {
        // Skip if there is no line of vision from the update index to the robot (on the update grid)
        Pos globalPos = toPos(globalInd, msg->info.width); 
        if(isOccupied(msg->data[globalInd]) || unobstructedLine(robotGlobalPos, globalPos, msg->data, msg->info.width)){ 
          updateOccupancy(globalInd,msg->data[globalInd]);
          /* mapMerged.data[globalInd] = round(decay*msg->data[globalInd] + (1-decay)*mapMerged.data[globalInd]); */
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
        setOccupancy(globalInd, P_PRIOR);
        updateOccupancy(globalInd,update->data[updateInd]);
        /* mapMerged.data[globalInd] = round(decay*update->data[updateInd] + (1-decay)*50); */
      } else {
        // Skip if there is no line of vision from the update index to the robot (on the update grid)
        if(isOccupied(update->data[updateInd]) || unobstructedLine(robotUpdatePos, updatePos, update->data, update->width)){
          updateOccupancy(globalInd,update->data[updateInd]);
          /* mapMerged.data[globalInd] = round(decay*update->data[updateInd] + (1-decay)*mapMerged.data[globalInd]); */
        }
      }

    }

    updateMerged.data[updateInd] = mapMerged.data[globalInd];
    
  }

  mapsArrived[name]++;

  return updateMerged;
}
