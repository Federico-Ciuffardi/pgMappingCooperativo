#pragma once

#include "../lib/GVD/src/Gvd.h"
#include "conversion.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/ColorRGBA.h"

#include <visualization_msgs/Marker.h>
#include <pgmappingcooperativo/Point2D.h>
#include <pgmappingcooperativo/SegmentAuction.h>
#include <pgmappingcooperativo/SegmentBid.h>
#include <nav_msgs/OccupancyGrid.h>

//////////////////////////
// Construction Helpers //
//////////////////////////
inline std_msgs::ColorRGBA makeColorRGBA(float r, float g, float b, float a){
  std_msgs::ColorRGBA ret;
  ret.r = r;
  ret.g = g;
  ret.b = b;
  ret.a = a;
  return ret;
}
inline geometry_msgs::Vector3 makeVector3(float x, float y, float z){
  geometry_msgs::Vector3 ret;
  ret.x = x;
  ret.y = y;
  ret.z = z;
  return ret;
}

inline geometry_msgs::Vector3 makeVector3(float x){
  return makeVector3(x, x, x);
}


inline geometry_msgs::Quaternion makeQuaternion(float x, float y, float z, float w){
  geometry_msgs::Quaternion ret;
  ret.x = x;
  ret.y = y;
  ret.z = z;
  ret.w = w;
  return ret;
}

////////////////
// Conversion //
////////////////

inline geometry_msgs::Point toMarkerPoint(Pos p, mapInfoType mapInfo){
  geometry_msgs::Point p3d;
  p3d.x = (p.x + 0.5) * mapInfo.resolution + mapInfo.origin.position.x;
  p3d.y = (p.y + 0.5) * mapInfo.resolution + mapInfo.origin.position.y;
  return p3d;
}

inline visualization_msgs::Marker::_points_type toMarkerPoint(PosSet posSet, mapInfoType mapInfo){
  vector<geometry_msgs::Point> markerPoints;
  for (Pos p : posSet) {
    markerPoints.push_back(toMarkerPoint(p, mapInfo));
  }
  return markerPoints;
}

///////////////
// Constants //
///////////////

static const std_msgs::ColorRGBA RED     = makeColorRGBA(1,0,0,1);
static const std_msgs::ColorRGBA YELLOW  = makeColorRGBA(1,1,0,1);
static const std_msgs::ColorRGBA GREEN   = makeColorRGBA(0,1,0,1);
static const std_msgs::ColorRGBA CYAN    = makeColorRGBA(0,1,1,1);
static const std_msgs::ColorRGBA BLUE    = makeColorRGBA(0,0,1,1);
static const std_msgs::ColorRGBA MAGENTA = makeColorRGBA(1,0,1,1);

///////////////
// Functions //
///////////////

// get a distinct color, 0<i<1024 
// Visual representation of the colors here: https://i.stack.imgur.com/cuF3C.png
std_msgs::ColorRGBA getColor(int i);

////////////
// Helper //
////////////

struct RvizHelper{
  enum {
   // types:
    ARROW = 0u,
    CUBE = 1u,
    SPHERE = 2u,
    CYLINDER = 3u,
    LINE_STRIP = 4u,
    LINE_LIST = 5u,
    CUBE_LIST = 6u,
    SPHERE_LIST = 7u,
    POINTS = 8u,
    TEXT_VIEW_FACING = 9u,
    MESH_RESOURCE = 10u,
    TRIANGLE_LIST = 11u,
   // actions:
    ADD = 0u,
    MODIFY = 0u,
    DELETE = 2u,
    DELETEALL = 3u,
  };

  typedef visualization_msgs::Marker Marker;
  typedef visualization_msgs::Marker::_points_type MarkerPoints;
  typedef std_msgs::ColorRGBA ColorRGBA;

  ros::Publisher *topic = NULL;
  std_msgs::ColorRGBA color = BLUE;
  string ns = "default";
  int id = 0;
  string frame_id = "map";
  geometry_msgs::Vector3 scale = makeVector3(1,1,1);
  geometry_msgs::Quaternion orientation = makeQuaternion(0,0,0,1);
  geometry_msgs::Vector3 position = makeVector3(0,0,0);
  unsigned int type = POINTS;
  unsigned int action = ADD;

  Marker getMark(MarkerPoints markerPoints, string ns = "", int id = -1);
  void mark(MarkerPoints markerPoints, string ns = "", int id = -1 );
  void deleteMark(string ns = "", int id = -1 );

  // old
  Marker mark_points(string ns, MarkerPoints ps,ColorRGBA color);
  Marker mark_lines(string ns, MarkerPoints ls, ColorRGBA color,float z,int type);
  Marker mark_lines(string ns, MarkerPoints ls, ColorRGBA color);
  Marker delete_marks(string ns);
};
