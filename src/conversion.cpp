#ifndef CONVERSION_CPP
#define CONVERSION_CPP

#include "GVD/GVD.h"
#include <tscf_exploration/Point2D.h>
#include <tscf_exploration/SegmentAuction.h>
#include <tscf_exploration/SegmentBid.h>
#include <nav_msgs/OccupancyGrid.h>
#include "utils.cpp"

static tscf_exploration::Point2D pos_to_p2d(pos p){
  tscf_exploration::Point2D p2d;
  p2d.x = p.first;
  p2d.y = p.second; 
  return p2d;
}

static pos p2d_to_pos(tscf_exploration::Point2D p2d){
  pos p;
  p.first = p2d.x; 
  p.second = p2d.y; 
  return p;
}


typedef nav_msgs::OccupancyGrid::_info_type map_info_type;

/* Converts from `pos` (pair<int,int>) in a grid map to a point (geometry_msgs::Point)
   adjusting the latter with the `map_info` so it lands on it's correspoing world position */
static geometry_msgs::Point p2d_to_p3d(tscf_exploration::Point2D p2d, map_info_type map_info) {
  geometry_msgs::Point p3d;
  p3d.x = p2d.x * map_info.resolution + map_info.origin.position.x + 0.5;
  p3d.y = p2d.y * map_info.resolution + map_info.origin.position.y + 0.5;
  p3d.z = 0;
  return p3d;
}

static geometry_msgs::Point p2d_to_p3d(tscf_exploration::Point2D p2d,float z, map_info_type map_info){
  geometry_msgs::Point p3d = p2d_to_p3d(p2d, map_info); 
  p3d.z = z;
  return p3d;
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

//this should be fila * width + columna
static int pos_to_p1d(pos p,int width){
  return p.first + p.second*width;
}

static pos p1d_to_pos(int p1d,int width){
  return pos(p1d%width,p1d/width);
}

static int p3d_to_p1d(geometry_msgs::Point p, int indice_origen, int width){
 return indice_origen + (((int)p.x + signo((int)p.x) * 1) + ((int)p.y) * width);
}

static pos p3d_to_pos(geometry_msgs::Point p, int indice_origen, int width){
  return p1d_to_pos(p3d_to_p1d(p,indice_origen,width), width); //no sure if is the same width
}


#endif