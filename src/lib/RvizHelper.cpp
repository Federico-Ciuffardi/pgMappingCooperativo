#include "RvizHelper.h"

RvizHelper::Marker RvizHelper::getMark(MarkerPoints markerPoints, string ns, int id) {
  if (id == -1) id = this->id;
  if (ns.empty()) ns = this->ns;

  Marker marker;
  // ns & id
  marker.ns = ns;
  marker.id = 0;
  // header
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  // pose/scale
  marker.pose.orientation = orientation;
  marker.scale = scale;
  // others
  marker.type = type;
  marker.action = action;
  marker.color = color;
  marker.points = markerPoints;

  return marker;
}

void RvizHelper::mark(MarkerPoints markerPoints, string ns, int id) {
  topic->publish(getMark(markerPoints, ns, id));
}

/* Publishes mark points `ps` on the namespace `ns` with the color `color` to be visualized on rviz */
RvizHelper::Marker RvizHelper::mark_points(string ns, MarkerPoints ps, ColorRGBA color) {
  Marker points;
  // ns & id
  points.ns = ns;
  points.id = 0;
  // header
  points.header.frame_id = frame_id;
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

/* Publishes mark lines `ls` on the namespace `ns` with the color `color` to be visualized on rviz */
RvizHelper::Marker RvizHelper::mark_lines(string ns, MarkerPoints ls, ColorRGBA color, float z, int type) {
  visualization_msgs::Marker lines;
  // ns & id
  lines.ns = ns;
  lines.id = 0;
  // header
  lines.header.frame_id = frame_id;
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

RvizHelper::Marker RvizHelper::mark_lines(string ns, MarkerPoints ls, ColorRGBA color) {
  //marker_pub.publish(lines);
  return mark_lines(ns, ls, color,-0.1,visualization_msgs::Marker::LINE_LIST);
}

//delete
RvizHelper::Marker RvizHelper::delete_marks(string ns) {
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

