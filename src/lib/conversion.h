#pragma once

#include "../lib/GVD/src/Gvd.h"
#include "../lib/GVD/src/data/Vector2.h"
#include <pgmappingcooperativo/Point2D.h>
#include <pgmappingcooperativo/SegmentAuction.h>
#include <pgmappingcooperativo/SegmentBid.h>
#include <nav_msgs/OccupancyGrid.h>
#include "utils.h"

static pgmappingcooperativo::Point2D pos_to_p2d(Pos p){
  pgmappingcooperativo::Point2D p2d;
  p2d.x = p.x;
  p2d.y = p.y; 
  return p2d;
}

static Pos p2d_to_pos(pgmappingcooperativo::Point2D p2d){
  Pos p;
  p.x = p2d.x; 
  p.y = p2d.y; 
  return p;
}

static vector<pgmappingcooperativo::Point2D> pos_to_p2d(vector<Pos> p){
  vector<pgmappingcooperativo::Point2D> res;
  for(auto it = p.begin(); it != p.end(); ++it){
    res.push_back(pos_to_p2d(*it));
  }
  return res;  
} 

typedef nav_msgs::OccupancyGrid::_info_type map_info_type;

/* Converts from `Pos` (pair<int,int>) in a grid map to a point (geometry_msgs::Point)
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


static geometry_msgs::Point pos_to_p3d(Pos p, map_info_type map_info) {
  geometry_msgs::Point p3d;
  p3d.x = p.x * map_info.resolution + map_info.origin.position.x + 0.5;
  p3d.y = p.y * map_info.resolution + map_info.origin.position.y + 0.5;
  p3d.z = 0;
  return p3d;
}

static geometry_msgs::Point pos_to_p3d(Pos p) {
  geometry_msgs::Point p3d;
  p3d.x = p.x;
  p3d.y = p.y;
  p3d.z = 0;
  return p3d;
}

static geometry_msgs::Point pos_to_p3d(Pos p,float z) {
  geometry_msgs::Point p3d;
  p3d.x = p.x;
  p3d.y = p.y;
  p3d.z = z;
  return p3d;
}

//this should be fila * width + columna
static int pos_to_p1d(Pos p,int width){
  return p.x + p.y*width;
}

static Pos p1d_to_pos(int p1d,int width){
  return Pos(p1d%width,p1d/width);
}

static int p3d_to_p1d(geometry_msgs::Point p, int indice_origen, int width){
 return indice_origen + (((int)p.x) + ((int)p.y) * width);
}

static Pos p3d_to_pos(geometry_msgs::Point p, int indice_origen, int width){
  return p1d_to_pos(p3d_to_p1d(p,indice_origen,width), width); //no sure if is the same width
}

static Pos p3d_to_pos(geometry_msgs::Point p3d){
  return Pos(p3d.x,p3d.y);
}

static geometry_msgs::Point p2f_to_p3d(cv::Point2f ps){
    geometry_msgs::Point p3d;
    p3d.y = ps.y;
    p3d.x = ps.x;
    p3d.z = 0.0;
    return p3d;
}

/* De ocupancy grid a state grid*/
static StateGrid og2gt(nav_msgs::OccupancyGrid og, vector<int> frontera = vector<int>(), int* count = NULL) {
  uint mapWidth = og.info.width;
  uint mapHeight = og.info.height;
  StateGrid res;
  if(count) (*count) = 0;
  cout<<"og2gt general"<<endl; 
  for (int x = 0; x < mapWidth; x++) {
    res.grid.push_back(StateGrid::ColType());
    for (int y = 0; y < mapHeight; y++) {
      CellState ct = Unknown;
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
          ct = (CellState)-1;
      }
      res.grid[x].push_back(ct);
    }
  }
  cout<<"og2gt Frontier"<<endl;
  for (auto it = frontera.begin(); it != frontera.end(); it++) {
    int p1d = *it;
    if(p1d>=mapWidth*mapHeight){
      cout<<"Warning: Frontier out of range"<<endl; //almost sure its a kmeans error happens some times in the office map
    }else{
      Pos p = p1d_to_pos(p1d, mapWidth);
      res.grid[p.x][p.y] = Frontier;
    }
  }
  return res;
}
