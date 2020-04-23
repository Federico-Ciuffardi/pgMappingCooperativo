# Two simulated Pioneer-3DX robots

# Copyright (C) 2014 Zhi Yan

from morse.builder import *
import os

##################### ROBOT 0 ####################
atrv0 = ATRV()
atrv0.add_default_interface('ros')
atrv0.translate(x=-36.5, y=36.5, z=0.0)

clock = Clock()
clock.add_interface('ros',topic='/clock')
atrv0.append(clock)

# odom = Odometry()
# odom.add_interface('ros', frame_id='/atrv0_tf/odom', child_frame_id='/atrv0_tf/base_footprint',  topic='/atrv0/odom')
# atrv0.append(odom)

sick = Sick()
sick.translate(z=0.252)
sick.properties(Visible_arc = True)
sick.properties(laser_range = 6.0)
sick.properties(resolution = 3.0)
sick.properties(scan_window = 360.0)
sick.add_interface('ros', frame_id='/atrv0_tf/lms100', topic='/atrv0/scan')
atrv0.append(sick)

pose = Pose()
pose.add_interface('ros', frame_id='/atrv0_tf/base_footprint')
pose.add_stream("ros", method="morse.middleware.ros.pose.TFPublisher", frame_id='/atrv0_tf/odom', child_frame_id='/atrv0_tf/base_footprint')
atrv0.append(pose)

# motion = MotionVWDiff()
# motion.add_interface('ros', topic='/atrv0/cmd_vel')
# atrv0.append(motion)

waypoint = Waypoint()
waypoint.add_stream('socket')
waypoint.add_service('socket')
atrv0.append(waypoint)

keyboard = Keyboard()
atrv0.append(keyboard)




##################### ROBOT  1 ####################
atrv1 = ATRV()
atrv1.add_default_interface('ros')
atrv1.translate(x=-36.5, y=29.5, z=0.0)

clock = Clock()
clock.add_interface('ros',topic='/clock')
atrv1.append(clock)

# odom = Odometry()
# odom.add_interface('ros', frame_id='/atrv1_tf/odom', child_frame_id='/atrv1_tf/base_footprint',  topic='/atrv1/odom')
# atrv1.append(odom)

sick = Sick()
sick.translate(z=0.252)
sick.properties(Visible_arc = True)
sick.properties(laser_range = 6.0)
sick.properties(resolution = 3.0)
sick.properties(scan_window = 360.0)
sick.add_interface('ros', frame_id='/atrv1_tf/lms100', topic='/atrv1/scan')
atrv1.append(sick)

pose = Pose()
pose.add_interface('ros', frame_id='/atrv1_tf/base_footprint')
pose.add_stream("ros", method="morse.middleware.ros.pose.TFPublisher", frame_id='/atrv1_tf/odom', child_frame_id='/atrv1_tf/base_footprint')
atrv1.append(pose)

# motion = MotionVWDiff()
# motion.add_interface('ros', topic='/atrv1/cmd_vel')
# atrv1.append(motion)

waypoint = Waypoint()
waypoint.add_stream('socket')
waypoint.add_service('socket')
atrv1.append(waypoint)

#
keyboard = Keyboard()
atrv1.append(keyboard)
#
# # # ##################### ROBOT  2 ####################
atrv2 = ATRV()
atrv2.add_default_interface('ros')
atrv2.translate(x=-36.5, y=22.5, z=0.0)

clock = Clock()
clock.add_interface('ros',topic='/clock')
atrv2.append(clock)

# odom = Odometry()
# odom.add_interface('ros', frame_id='/atrv2_tf/odom', child_frame_id='/atrv2_tf/base_footprint',  topic='/atrv2/odom')
# atrv2.append(odom)

sick = Sick()
sick.translate(z=0.252)
sick.properties(Visible_arc = True)
sick.properties(laser_range = 6.0)
sick.properties(resolution = 3.0)
sick.properties(scan_window = 360.0)
sick.add_interface('ros', frame_id='/atrv2_tf/lms100', topic='/atrv2/scan')
atrv2.append(sick)

pose = Pose()
pose.add_interface('ros', frame_id='/atrv2_tf/base_footprint')
pose.add_stream("ros", method="morse.middleware.ros.pose.TFPublisher", frame_id='/atrv2_tf/odom', child_frame_id='/atrv2_tf/base_footprint')
atrv2.append(pose)

# motion = MotionVWDiff()
# motion.add_interface('ros', topic='/atrv2/cmd_vel')
# atrv2.append(motion)

waypoint = Waypoint()
waypoint.add_stream('socket')
waypoint.add_service('socket')
atrv2.append(waypoint)


keyboard = Keyboard()
atrv2.append(keyboard)

# # # # # ##################### ROBOT 3 ####################
# atrv3 = ATRV()
# atrv3.add_default_interface('ros')
# atrv3.translate(x=-29.5, y=36.5, z=0.0)
#
# clock = Clock()
# clock.add_interface('ros',topic='/clock')
# atrv3.append(clock)
#
# # odom = Odometry()
# # odom.add_interface('ros', frame_id='/atrv3_tf/odom', child_frame_id='/atrv3_tf/base_footprint',  topic='/atrv3/odom')
# # atrv3.append(odom)
#
# sick = Sick()
# sick.translate(z=0.252)
# sick.properties(Visible_arc = True)
# sick.properties(laser_range = 6.0)
# sick.properties(resolution = 3.0)
# sick.properties(scan_window = 360.0)
# sick.add_interface('ros', frame_id='/atrv3_tf/lms100', topic='/atrv3/scan')
# atrv3.append(sick)
#
# pose = Pose()
# pose.add_interface('ros', frame_id='/atrv3_tf/base_footprint')
# pose.add_stream("ros", method="morse.middleware.ros.pose.TFPublisher", frame_id='/atrv3_tf/odom', child_frame_id='/atrv3_tf/base_footprint')
# atrv3.append(pose)
#
# # motion = MotionVWDiff()
# # motion.add_interface('ros', topic='/atrv3/cmd_vel')
# # atrv3.append(motion)
#
# waypoint = Waypoint()
# waypoint.add_stream('socket')
# waypoint.add_service('socket')
# atrv3.append(waypoint)
#
#
# keyboard = Keyboard()
# atrv3.append(keyboard)
# #
# # # # ##################### ROBOT 4 ####################
# atrv4 = ATRV()
# atrv4.add_default_interface('ros')
# atrv4.translate(x=-22.5, y=36.5, z=0.0)
#
# clock = Clock()
# clock.add_interface('ros',topic='/clock')
# atrv4.append(clock)
#
# # odom = Odometry()
# # odom.add_interface('ros', frame_id='/atrv4_tf/odom', child_frame_id='/atrv4_tf/base_footprint',  topic='/atrv4/odom')
# # atrv4.append(odom)
#
# sick = Sick()
# sick.translate(z=0.252)
# sick.properties(Visible_arc = True)
# sick.properties(laser_range = 6.0)
# sick.properties(resolution = 3.0)
# sick.properties(scan_window = 360.0)
# sick.add_interface('ros', frame_id='/atrv4_tf/lms100', topic='/atrv4/scan')
# atrv4.append(sick)
#
# pose = Pose()
# pose.add_interface('ros', frame_id='/atrv4_tf/base_footprint')
# pose.add_stream("ros", method="morse.middleware.ros.pose.TFPublisher", frame_id='/atrv4_tf/odom', child_frame_id='/atrv4_tf/base_footprint')
# atrv4.append(pose)
#
# # motion = MotionVWDiff()
# # motion.add_interface('ros', topic='/atrv4/cmd_vel')
# # atrv4.append(motion)
#
# waypoint = Waypoint()
# waypoint.add_stream('socket')
# waypoint.add_service('socket')
# atrv4.append(waypoint)
#
#
# keyboard = Keyboard()
# atrv4.append(keyboard)
#
# # # ##################### ROBOT 5 ####################
# atrv5 = ATRV()
# atrv5.add_default_interface('ros')
# atrv5.translate(x=-15.5, y=36.5, z=0.0)
#
# clock = Clock()
# clock.add_interface('ros',topic='/clock')
# atrv5.append(clock)
#
# # odom = Odometry()
# # odom.add_interface('ros', frame_id='/atrv5_tf/odom', child_frame_id='/atrv5_tf/base_footprint',  topic='/atrv5/odom')
# # atrv5.append(odom)
#
# sick = Sick()
# sick.translate(z=0.252)
# sick.properties(Visible_arc = True)
# sick.properties(laser_range = 6.0)
# sick.properties(resolution = 3.0)
# sick.properties(scan_window = 360.0)
# sick.add_interface('ros', frame_id='/atrv5_tf/lms100', topic='/atrv5/scan')
# atrv5.append(sick)
#
# pose = Pose()
# pose.add_interface('ros', frame_id='/atrv5_tf/base_footprint')
# pose.add_stream("ros", method="morse.middleware.ros.pose.TFPublisher", frame_id='/atrv5_tf/odom', child_frame_id='/atrv5_tf/base_footprint')
# atrv5.append(pose)
#
# # motion = MotionVWDiff()
# # motion.add_interface('ros', topic='/atrv5/cmd_vel')
# # atrv5.append(motion)
#
# waypoint = Waypoint()
# waypoint.add_stream('socket')
# waypoint.add_service('socket')
# atrv5.append(waypoint)
# #
# #
# keyboard = Keyboard()
# atrv5.append(keyboard)
##################### ENVIRONMENT ####################
home = os.getenv("HOME")
env = Environment(home+'/catkin_ws/src/tscf_exploration/morse/scenarios/maze.blend', fastmode=True)
env.place_camera([0, 0, 80])
env.set_time_scale(accelerate_by=3.0)
env.aim_camera([0, 0, 0])
