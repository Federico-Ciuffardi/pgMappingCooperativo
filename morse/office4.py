# Two simulated Pioneer-3DX robots

# Copyright (C) 2014 Zhi Yan

from morse.builder import *
import os

##################### ROBOT 1 ####################
atrv0 = ATRV()
atrv0.add_default_interface('ros')
atrv0.translate(x=-36.5, y=36.5, z=0.0)

clock = Clock()
clock.add_interface('ros',topic='/clock')
atrv0.append(clock)

sick = Sick()
sick.translate(z=0.252)
sick.properties(Visible_arc = False)
sick.properties(laser_range = 6.0)
sick.properties(resolution = 3.0)
sick.properties(scan_window = 360.0)
sick.add_interface('ros', frame_id='/atrv0_tf/lms100', topic='/atrv0/scan')
atrv0.append(sick)

pose = Pose()
pose.add_interface('ros',frame_id='/atrv0_tf/odom', child_frame_id='/atrv0_tf/base_footprint')
pose.add_stream("ros", method="morse.middleware.ros.pose.TFPublisher", frame_id='/atrv0_tf/odom', child_frame_id='/atrv0_tf/base_footprint')
atrv0.append(pose)

waypoint = Waypoint()
waypoint.properties(ObstacleAvoidance=False)
waypoint.add_stream('socket')
waypoint.add_service('socket')
atrv0.append(waypoint)

keyboard = Keyboard()
atrv0.append(keyboard)

##################### ROBOT  2 ####################
atrv1 = ATRV()
atrv1.add_default_interface('ros')
atrv1.translate(x=-36.5, y=29.5, z=0.0)

clock = Clock()
clock.add_interface('ros',topic='/clock')
atrv1.append(clock)

sick = Sick()
sick.translate(z=0.252)
sick.properties(Visible_arc = False)
sick.properties(laser_range = 6.0)
sick.properties(resolution = 3.0)
sick.properties(scan_window = 360.0)
sick.add_interface('ros', frame_id='/atrv1_tf/lms100', topic='/atrv1/scan')
atrv1.append(sick)

pose = Pose()
pose.add_interface('ros', frame_id='/atrv1_tf/odom', child_frame_id='/atrv1_tf/base_footprint')
pose.add_stream("ros", method="morse.middleware.ros.pose.TFPublisher", frame_id='/atrv1_tf/odom', child_frame_id='/atrv1_tf/base_footprint')
pose.frequency(60)
atrv1.append(pose)

waypoint = Waypoint()
waypoint.properties(ObstacleAvoidance=False)
waypoint.add_stream('socket')
waypoint.add_service('socket')
#waypoint.properties(ControlType='Position')
atrv1.append(waypoint)

# # # ##################### ROBOT  3 ####################
atrv2 = ATRV()
atrv2.add_default_interface('ros')
atrv2.translate(x=-36.5, y=22.5, z=0.0)

clock = Clock()
clock.add_interface('ros',topic='/clock')
atrv2.append(clock)

sick = Sick()
sick.translate(z=0.252)
sick.properties(Visible_arc = False)
sick.properties(laser_range = 6.0)
sick.properties(resolution = 3.0)
sick.properties(scan_window = 360.0)
sick.add_interface('ros', frame_id='/atrv2_tf/lms100', topic='/atrv2/scan')
atrv2.append(sick)

pose = Pose()
pose.add_interface('ros', frame_id='/atrv2_tf/odom', child_frame_id='/atrv2_tf/base_footprint')
pose.add_stream("ros", method="morse.middleware.ros.pose.TFPublisher", frame_id='/atrv2_tf/odom', child_frame_id='/atrv2_tf/base_footprint')
atrv2.append(pose)

waypoint = Waypoint()
waypoint.properties(ObstacleAvoidance=False)
waypoint.add_stream('socket')
waypoint.add_service('socket')
atrv2.append(waypoint)

# # # ##################### ROBOT  4 ####################
atrv3 = ATRV()
atrv3.add_default_interface('ros')
atrv3.translate(x=-36.5, y=15.5, z=0.0)

clock = Clock()
clock.add_interface('ros',topic='/clock')
atrv3.append(clock)

sick = Sick()
sick.translate(z=0.252)
sick.properties(Visible_arc = False)
sick.properties(laser_range = 6.0)
sick.properties(resolution = 3.0)
sick.properties(scan_window = 360.0)
sick.add_interface('ros', frame_id='/atrv3_tf/lms100', topic='/atrv3/scan')
atrv3.append(sick)

pose = Pose()
pose.add_interface('ros', frame_id='/atrv3_tf/odom', child_frame_id='/atrv3_tf/base_footprint')
pose.add_stream("ros", method="morse.middleware.ros.pose.TFPublisher", frame_id='/atrv3_tf/odom', child_frame_id='/atrv3_tf/base_footprint')
atrv3.append(pose)

waypoint = Waypoint()
waypoint.properties(ObstacleAvoidance=False)
waypoint.add_stream('socket')
waypoint.add_service('socket')
atrv3.append(waypoint)
'''
# # # ##################### ROBOT  5 ####################
atrv4 = ATRV()
atrv4.add_default_interface('ros')
atrv4.translate(x=-36.5, y=8.5, z=0.0)

clock = Clock()
clock.add_interface('ros',topic='/clock')
atrv4.append(clock)

sick = Sick()
sick.translate(z=0.252)
sick.properties(Visible_arc = False)
sick.properties(laser_range = 6.0)
sick.properties(resolution = 3.0)
sick.properties(scan_window = 360.0)
sick.add_interface('ros', frame_id='/atrv4_tf/lms100', topic='/atrv4/scan')
atrv4.append(sick)

pose = Pose()
pose.add_interface('ros', frame_id='/atrv4_tf/odom', child_frame_id='/atrv4_tf/base_footprint')
pose.add_stream("ros", method="morse.middleware.ros.pose.TFPublisher", frame_id='/atrv4_tf/odom', child_frame_id='/atrv4_tf/base_footprint')
atrv4.append(pose)

waypoint = Waypoint()
waypoint.properties(ObstacleAvoidance=False)
waypoint.add_stream('socket')
waypoint.add_service('socket')
atrv4.append(waypoint)
'''

##################### ENVIRONMENT ####################
home = os.getenv("HOME")
env = Environment(home+'/catkin_ws/src/tscf_exploration/morse/scenarios/office.blend', fastmode=True)
#env.set_time_scale(accelerate_by=5.0)
env.use_vsync('OFF')
#env.place_camera([0, 0, 80])
#env.aim_camera([0, 0, 0])
