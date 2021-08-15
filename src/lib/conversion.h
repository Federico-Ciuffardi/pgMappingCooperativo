#pragma once

#include "../lib/GVD/src/Gvd.h"
#include "../lib/GVD/src/data/Vector2.h"
#include <pgmappingcooperativo/Point2D.h>
#include <pgmappingcooperativo/SegmentAuction.h>
#include <pgmappingcooperativo/SegmentBid.h>
#include <nav_msgs/OccupancyGrid.h>
#include <utility>
#include "GVD/src/data/Pos.h"
#include "utils.h"

typedef pgmappingcooperativo::Point2D Point2D;
typedef nav_msgs::OccupancyGrid::_info_type mapInfoType;
typedef geometry_msgs::Point Point; 

/////////////
// Point2D //
/////////////
inline Point2D toPoint2D(Pos p){
  Point2D p2d;
  p2d.x = p.x;
  p2d.y = p.y; 
  return p2d;
}

inline Point2D toPoint2D(Point p3d) {
  Point2D p2d;
  p2d.x = p3d.x;
  p2d.y = p3d.y;
  return p2d;
}

inline vector<Point2D> toVecPoint2D(vector<Pos> ps){
  vector<Point2D> res;
  for( Pos p : ps ){
    res.push_back(toPoint2D(p));
  }
  return res;
}

/////////////
// Point3D //
/////////////

inline Point toPoint(Point2D p2d, mapInfoType map_info) {
  Point p3d;
  p3d.x = (p2d.x + 0.5) * map_info.resolution + map_info.origin.position.x;
  p3d.y = (p2d.y + 0.5) * map_info.resolution + map_info.origin.position.y;
  p3d.z = 0;
  return p3d;
}

inline Point toPoint(Point2D p2d, float z, mapInfoType map_info){
  Point p3d = toPoint(p2d, map_info); 
  p3d.z = z;
  return p3d;
}

inline Point toPoint(Pos p, mapInfoType map_info) {
  Point p3d;
  p3d.x = (p.x + 0.5) * map_info.resolution + map_info.origin.position.x;
  p3d.y = (p.y + 0.5) * map_info.resolution + map_info.origin.position.y;
  p3d.z = 0;
  return p3d;
}

inline Point toPoint(Pos p) {
  Point p3d;
  p3d.x = p.x;
  p3d.y = p.y;
  p3d.z = 0;
  return p3d;
}

inline Point toPoint(Pos p,float z) {
  Point p3d;
  p3d.x = p.x;
  p3d.y = p.y;
  p3d.z = z;
  return p3d;
}

inline Point toPoint(cv::Point2f ps){
    Point p3d;
    p3d.y = ps.y;
    p3d.x = ps.x;
    p3d.z = 0.0;
    return p3d;
}

inline vector<Point> toVecPoint3D(vector<pgmappingcooperativo::Point2D> p2ds, mapInfoType mapInfo){
  vector<geometry_msgs::Point> p3ds;
  for (Point2D c : p2ds) {
    p3ds.push_back(toPoint(c, mapInfo));
  }
  return p3ds;
}

/////////
// Int //
/////////

inline Int toInt(Pos p, Int width){
  return p.x + p.y*width;
}

inline Int toInt(Point p, int indice_origen, int width){
 return indice_origen + (((int)p.x) + ((int)p.y) * width);
}

/////////
// Pos //
/////////

inline Pos toPos(Point2D p2d){
  Pos p;
  p.x = p2d.x; 
  p.y = p2d.y; 
  return p;
}

inline Pos toPos(int p1d,int width){
  return Pos(p1d%width,p1d/width);
}

inline Pos toPos(Point p, int indice_origen, int width){
  return toPos(toInt(p,indice_origen,width), width); //no sure if is the same width
}

inline Pos toPos(Point p3d){
  return Pos(p3d.x,p3d.y);
}

///////////////
// StateGrid //
///////////////

inline StateGrid toStateGrid(nav_msgs::OccupancyGrid og, vector<int> fronteras = vector<int>(), int* count = NULL) {
  pair<Int,Int> mapSize = make_pair(og.info.width, og.info.height);
  StateGrid stateGrid(mapSize);

  if(count) (*count) = 0;

  for (Pos p : stateGrid) {
    CellState ct = Unknown;
    switch (og.data[toInt(p, mapSize.first)]) {
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
    stateGrid[p] = ct;
  }

  for (Int fInt : fronteras) {
    Pos fPos = toPos(fInt,mapSize.first);
    if(stateGrid.inside(fPos)){
      stateGrid[fPos] = Frontier;
    }else{
      cout<<"Warning: Frontier out of range"<<endl; //almost sure its a kmeans error happens some times in the office map
    }
  }

  return stateGrid;
}

