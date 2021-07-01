#ifndef CONVERSION_H
#define CONVERSION_H

#include "../lib/GVD/GVD.h"
#include <pgmappingcooperativo/Point2D.h>
#include <pgmappingcooperativo/SegmentAuction.h>
#include <pgmappingcooperativo/SegmentBid.h>
#include <nav_msgs/OccupancyGrid.h>
#include "utils.h"

static pgmappingcooperativo::Point2D pos_to_p2d(pos p){
  pgmappingcooperativo::Point2D p2d;
  p2d.x = p.first;
  p2d.y = p.second; 
  return p2d;
}

static pos p2d_to_pos(pgmappingcooperativo::Point2D p2d){
  pos p;
  p.first = p2d.x; 
  p.second = p2d.y; 
  return p;
}

static vector<pgmappingcooperativo::Point2D> pos_to_p2d(vector<pos> p){
  vector<pgmappingcooperativo::Point2D> res;
  for(auto it = p.begin(); it != p.end(); ++it){
    res.push_back(pos_to_p2d(*it));
  }
  return res;  
} 

typedef nav_msgs::OccupancyGrid::_info_type map_info_type;

/* Converts from `pos` (pair<int,int>) in a grid map to a point (geometry_msgs::Point)
   adjusting the latter with the `map_info` so it lands on it's correspoing map position */
static geometry_msgs::Point p2d_to_p3d(pgmappingcooperativo::Point2D p2d, map_info_type map_info) {
  geometry_msgs::Point p3d;
  p3d.x = p2d.x * map_info.resolution + map_info.origin.position.x + 0.5;
  p3d.y = p2d.y * map_info.resolution + map_info.origin.position.y + 0.5;
  p3d.z = 0;
  return p3d;
}

static geometry_msgs::Point p2d_to_p3d(pgmappingcooperativo::Point2D p2d,float z, map_info_type map_info){
  geometry_msgs::Point p3d = p2d_to_p3d(p2d, map_info); 
  p3d.z = z;
  return p3d;
}

static pgmappingcooperativo::Point2D p3d_to_p2d(geometry_msgs::Point p3d) {
  pgmappingcooperativo::Point2D p2d;
  p2d.x = p3d.x;
  p2d.y = p3d.y;
  return p2d;
}


static geometry_msgs::Point pos_to_p3d(pos p, map_info_type map_info) {
  geometry_msgs::Point p3d;
  p3d.x = p.first * map_info.resolution + map_info.origin.position.x + 0.5;
  p3d.y = p.second * map_info.resolution + map_info.origin.position.y + 0.5;
  p3d.z = 0;
  return p3d;
}

static geometry_msgs::Point pos_to_p3d(pos p) {
  geometry_msgs::Point p3d;
  p3d.x = p.first;
  p3d.y = p.second;
  p3d.z = 0;
  return p3d;
}

static geometry_msgs::Point pos_to_p3d(pos p,float z) {
  geometry_msgs::Point p3d;
  p3d.x = p.first;
  p3d.y = p.second;
  p3d.z = z;
  return p3d;
}

//this should be fila * width + columna
static int pos_to_p1d(pos p,int width){
  return p.first + p.second*width;
}

static pos p1d_to_pos(int p1d,int width){
  return pos(p1d%width,p1d/width);
}

static int p3d_to_p1d(geometry_msgs::Point p, int indice_origen, int width){
 return indice_origen + (((int)p.x) + ((int)p.y) * width);
}

static pos p3d_to_pos(geometry_msgs::Point p, int indice_origen, int width){
  return p1d_to_pos(p3d_to_p1d(p,indice_origen,width), width); //no sure if is the same width
}

static pos p3d_to_pos(geometry_msgs::Point p3d){
  return pos(p3d.x,p3d.y);
}

static geometry_msgs::Point p2f_to_p3d(cv::Point2f ps){
    geometry_msgs::Point p3d;
    p3d.y = ps.y;
    p3d.x = ps.x;
    p3d.z = 0.0;
    return p3d;
}

/* De ocupancy grid a state grid*/
static grid_type og2gt(nav_msgs::OccupancyGrid og, vector<int> frontera = vector<int>(), int* count = NULL) {
  uint mapWidth = og.info.width;
  uint mapHeight = og.info.height;
  grid_type res;
  if(count) (*count) = 0;
  cout<<"og2gt general"<<endl; 
  for (int x = 0; x < mapWidth; x++) {
    res.push_back(row_type());
    for (int y = 0; y < mapHeight; y++) {
      cell_type ct = Unknown;
      switch (og.data[y * mapWidth + x]) {
        case 0:
          ct = Free;
          if(count) (*count)++;
          break;
        case 100:
          ct = Occupied;
          if(count) (*count)++;
          break;
        case -1:
          ct = Unknown;
          break;
        default:
          ct = (cell_type)-1;
      }
      res[x].push_back(ct);
    }
  }
  cout<<"og2gt Frontier"<<endl;
  for (auto it = frontera.begin(); it != frontera.end(); it++) {
    int p1d = *it;
    if(p1d>=mapWidth*mapHeight){
      cout<<"Warning: Frontier out of range"<<endl; //almost sure its a kmeans error happens some times in the office map
    }else{
      pos p = p1d_to_pos(p1d, mapWidth);
      res[p.first][p.second] = Frontier;
    }
  }
  return res;
}

#endif
