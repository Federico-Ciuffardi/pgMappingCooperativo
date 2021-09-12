#pragma once

#include "../lib/GVD/src/Gvd.h"
#include "../lib/GVD/src/data/Vector2.h"
#include <pgmappingcooperativo/Point2D.h>
#include <pgmappingcooperativo/Graph.h>
#include <nav_msgs/OccupancyGrid.h>
#include <boost/unordered/unordered_set_fwd.hpp>
#include <utility>
#include "GVD/src/Map.h"
#include "GVD/src/data/Pos.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
#include <map_msgs/OccupancyGridUpdate.h>

using namespace pgmappingcooperativo;
using namespace geometry_msgs;
using namespace nav_msgs;
using namespace map_msgs;

typedef nav_msgs::OccupancyGrid::_info_type mapInfoType;

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
  res.reserve(ps.size());
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

template <typename T>
inline Point toPoint(Vector2<T> p) {
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

inline vector<Point> toVecPoint3D(vector<Point2D> p2ds, mapInfoType mapInfo){
  vector<Point> p3ds;
  p3ds.reserve(p2ds.size());
  for (Point2D c : p2ds) {
    p3ds.push_back(toPoint(c, mapInfo));
  }
  return p3ds;
}

/////////////////
// PoseStamped //
/////////////////

inline PoseStamped toPoseStamped(nav_msgs::Odometry odom){
  PoseStamped ps;
  ps.header = odom.header;
  ps.pose = odom.pose.pose;
  return ps;
}

/////////////
// Vector3 //
/////////////

inline Vector3 toVector3(Point point){
  Vector3 vector3;
  vector3.x = point.x;
  vector3.y = point.y;
  vector3.z = point.z;
  return vector3;
}

/////////////////////////////
// Pos: GVD/src/data/Pos.h //
/////////////////////////////

inline Pos toPos(Point2D p2d){
  Pos p;
  p.x = p2d.x; 
  p.y = p2d.y; 
  return p;
}

inline Pos toPos(int p1d,int width){
  return Pos(p1d%width,p1d/width);
}

inline Pos toPos(Point p3d){
  return Pos(p3d.x,p3d.y);
}

inline Pos toPos(Point p3d, mapInfoType mapInfo) {

  Pos adjustment(-(p3d.x < 0),-(p3d.y < 0));

  return (toPos(p3d) - toPos(mapInfo.origin.position))/mapInfo.resolution  + adjustment;
}

inline PosSet toPosSet(vector<Point2D> &ps){
  PosSet res;
  for( Point2D p : ps ){
    res.insert(toPos(p));
  }
  return res;
}

inline PosSet toPosSet(boost::unordered_set<int> ps, int width){
  PosSet res;
  for( int p : ps ){
    res.insert(toPos(p,width));
  }
  return res;
}

inline PosSet toPosSet(vector<int> ps, int width){
  PosSet res;
  for( int p : ps ){
    res.insert(toPos(p,width));
  }
  return res;
}

template<typename T>
inline Pos toPos(Vector2<T> v){
  return Pos(v.x,v.y);
}

template<typename T>
inline Pos toPos(Vector2<T> v, mapInfoType mapInfo){

  Pos adjustment(-(v.x < 0),-(v.y < 0));

  return (toPos(v) - toPos(mapInfo.origin.position))/mapInfo.resolution  + adjustment;
}

///////////////
// toVector2 //
///////////////

template<typename T>
inline Vector2<T> toVector2(Point p3d){
  return Vector2<T>(p3d.x,p3d.y);
}

/////////
// Int //
/////////

inline Int toInt(Pos p, Int width){
  return p.x + p.y*width;
}

inline Int toInt(Point p, int width, int originIndex = 0){
 int x = p.x;
 int y = p.y;

 Pos adjustment(-(p.x < 0),-(p.y < 0));

 return originIndex + x + adjustment.x + (y + adjustment.y) * width;
}

inline Int toInt(Point p, mapInfoType mapInfo){
  /* int originX = mapInfo.origin.position.x; */
  /* int originY = mapInfo.origin.position.y; */
  /* return toInt(p,mapInfo.width,abs(originX) * mapInfo.width) + abs(originY); */
  return toInt(toPos(p,mapInfo),mapInfo.width);
}

/////////////////////////////
// StateGrid GVD/src/Map.h //
/////////////////////////////

// get stateGrid from occupancy grid, set the known cells count on the count attribute 
inline StateGrid toStateGrid(nav_msgs::OccupancyGrid &og, int* count = NULL) {
  int threshold = 50;

  pair<Int,Int> mapSize = make_pair(og.info.width, og.info.height);
  StateGrid stateGrid(mapSize);

  if(count) (*count) = 0;

  for (Pos p : stateGrid) {
    int occupancyCertantyPercentage = og.data[toInt(p, mapSize.first)];
    if(occupancyCertantyPercentage == -1){
      stateGrid[p] = Unknown;
    }else if(occupancyCertantyPercentage <= threshold){
      stateGrid[p] = Free;
      if(count) (*count)++;
    } else if(occupancyCertantyPercentage > threshold) {
      stateGrid[p] = Occupied;
      if(count) (*count)++;
    } else {
      FAIL("invalid occupancyCertantyPercentage: " << occupancyCertantyPercentage);
    }
  }

  return stateGrid;
}

/////////////////////////////////
// Graph: GVD/src/data/Graph.h //
/////////////////////////////////

template <typename Graph>
inline Graph toGraph(pgmappingcooperativo::Graph &gMsg) {
  Graph g;

  for (Point2D vPoint2D : gMsg.vertices) {
    Pos vPos = toPos(vPoint2D);
    g.addV(vPos);
  }

  for (int i = 0; i < gMsg.edges.size(); i++) {
    Pos from_p = toPos(gMsg.edges[i].from);
    Pos to_p = toPos(gMsg.edges[i].to);
    g.addE(g.idVertexMap[from_p], g.idVertexMap[to_p], from_p.distanceTo(to_p));
  }

  return g;
}
