#ifndef RVIZ_H
#define RVIZ_H

#include "../lib/GVD/GVD.h"
#include "conversion.h"

#include <visualization_msgs/Marker.h>
#include <pgmappingcooperativo/Point2D.h>
#include <pgmappingcooperativo/SegmentAuction.h>
#include <pgmappingcooperativo/SegmentBid.h>
#include <nav_msgs/OccupancyGrid.h>


/* Publishes mark points `ps` on the namespace `ns` with the color `color` to be visualized on rviz
 */
static visualization_msgs::Marker mark_points(string ns,
                 visualization_msgs::Marker::_points_type ps,
                 std_msgs::ColorRGBA color) {
  visualization_msgs::Marker points;
  // ns & id
  points.ns = ns;
  points.id = 0;
  // header
  points.header.frame_id = "map";
  points.header.stamp = ros::Time::now();
  // pose
  points.pose.orientation.w = 1.0;
  // others
  points.type = visualization_msgs::Marker::POINTS;
  points.action = visualization_msgs::Marker::ADD;
  points.scale.x = 1;
  points.scale.y = 1;
  points.color = color;
  points.points = ps;

  //marker_pub.publish(points);
  return points;
}

/* Publishes mark lines `ls` on the namespace `ns` with the color `color` to be visualized on rviz
 */

static visualization_msgs::Marker mark_lines(string ns, visualization_msgs::Marker::_points_type ls, std_msgs::ColorRGBA color,float z,int type) {
  visualization_msgs::Marker lines;
  // ns & id
  lines.ns = ns;
  lines.id = 0;
  // header
  lines.header.frame_id = "map";
  lines.header.stamp = ros::Time::now();
  // pose
  lines.pose.orientation.w = 1.0;
  lines.pose.position.z = z;
  // others
  lines.type = type;
  lines.action = visualization_msgs::Marker::ADD;
  lines.scale.x = 0.2;
  lines.color = color;
  lines.points = ls;

  //marker_pub.publish(lines);
  return lines;
}

static visualization_msgs::Marker mark_lines(string ns, visualization_msgs::Marker::_points_type ls, std_msgs::ColorRGBA color) {
  //marker_pub.publish(lines);
  return mark_lines(ns, ls, color,-0.1,visualization_msgs::Marker::LINE_LIST);
}

//delete

static visualization_msgs::Marker delete_marks(string ns) {
  visualization_msgs::Marker lines;
  // ns & id
  lines.ns = ns;
  lines.id = 0;
  // header
  lines.header.frame_id = "/map";
  lines.header.stamp = ros::Time::now();
  lines.action = visualization_msgs::Marker::DELETE;
  return lines;
}

#endif
