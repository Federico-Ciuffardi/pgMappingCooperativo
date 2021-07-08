# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "pgmappingcooperativo: 15 messages, 0 services")

set(MSG_I_FLAGS "-Ipgmappingcooperativo:/home/fede/catkin_ws/src/pgmappingcooperativo/msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Inav_msgs:/opt/ros/noetic/share/nav_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(pgmappingcooperativo_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/goalList.msg" NAME_WE)
add_custom_target(_pgmappingcooperativo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pgmappingcooperativo" "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/goalList.msg" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/frontierReport.msg" NAME_WE)
add_custom_target(_pgmappingcooperativo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pgmappingcooperativo" "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/frontierReport.msg" "pgmappingcooperativo/infoCentro"
)

get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/infoCentro.msg" NAME_WE)
add_custom_target(_pgmappingcooperativo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pgmappingcooperativo" "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/infoCentro.msg" ""
)

get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/asignacion.msg" NAME_WE)
add_custom_target(_pgmappingcooperativo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pgmappingcooperativo" "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/asignacion.msg" "pgmappingcooperativo/asignacionCelda"
)

get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/asignacionCelda.msg" NAME_WE)
add_custom_target(_pgmappingcooperativo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pgmappingcooperativo" "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/asignacionCelda.msg" ""
)

get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/resumenInstancia.msg" NAME_WE)
add_custom_target(_pgmappingcooperativo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pgmappingcooperativo" "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/resumenInstancia.msg" ""
)

get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/mapMergedInfo.msg" NAME_WE)
add_custom_target(_pgmappingcooperativo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pgmappingcooperativo" "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/mapMergedInfo.msg" "nav_msgs/OccupancyGrid:geometry_msgs/Pose:geometry_msgs/Quaternion:std_msgs/Header:geometry_msgs/Point:nav_msgs/MapMetaData"
)

get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/takeobjetive.msg" NAME_WE)
add_custom_target(_pgmappingcooperativo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pgmappingcooperativo" "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/takeobjetive.msg" "nav_msgs/OccupancyGrid:geometry_msgs/Pose:geometry_msgs/Quaternion:std_msgs/Header:geometry_msgs/Point:nav_msgs/MapMetaData"
)

get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg" NAME_WE)
add_custom_target(_pgmappingcooperativo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pgmappingcooperativo" "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg" ""
)

get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Edge.msg" NAME_WE)
add_custom_target(_pgmappingcooperativo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pgmappingcooperativo" "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Edge.msg" "pgmappingcooperativo/Point2D"
)

get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Graph.msg" NAME_WE)
add_custom_target(_pgmappingcooperativo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pgmappingcooperativo" "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Graph.msg" "pgmappingcooperativo/Edge:pgmappingcooperativo/Point2D"
)

get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/SegmentAuction.msg" NAME_WE)
add_custom_target(_pgmappingcooperativo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pgmappingcooperativo" "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/SegmentAuction.msg" "pgmappingcooperativo/Edge:pgmappingcooperativo/Graph:pgmappingcooperativo/Point2D"
)

get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/SegmentBid.msg" NAME_WE)
add_custom_target(_pgmappingcooperativo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pgmappingcooperativo" "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/SegmentBid.msg" "pgmappingcooperativo/Point2D"
)

get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/SegmentAssignment.msg" NAME_WE)
add_custom_target(_pgmappingcooperativo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pgmappingcooperativo" "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/SegmentAssignment.msg" "pgmappingcooperativo/Point2D"
)

get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/FrontierBid.msg" NAME_WE)
add_custom_target(_pgmappingcooperativo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pgmappingcooperativo" "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/FrontierBid.msg" "pgmappingcooperativo/Point2D"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/goalList.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_cpp(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/frontierReport.msg"
  "${MSG_I_FLAGS}"
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/infoCentro.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_cpp(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/infoCentro.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_cpp(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/asignacion.msg"
  "${MSG_I_FLAGS}"
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/asignacionCelda.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_cpp(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/asignacionCelda.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_cpp(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/resumenInstancia.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_cpp(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/mapMergedInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/nav_msgs/cmake/../msg/OccupancyGrid.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/MapMetaData.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_cpp(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/takeobjetive.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/nav_msgs/cmake/../msg/OccupancyGrid.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/MapMetaData.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_cpp(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_cpp(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Edge.msg"
  "${MSG_I_FLAGS}"
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_cpp(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Graph.msg"
  "${MSG_I_FLAGS}"
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Edge.msg;/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_cpp(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/SegmentAuction.msg"
  "${MSG_I_FLAGS}"
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Edge.msg;/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Graph.msg;/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_cpp(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/SegmentBid.msg"
  "${MSG_I_FLAGS}"
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_cpp(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/SegmentAssignment.msg"
  "${MSG_I_FLAGS}"
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_cpp(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/FrontierBid.msg"
  "${MSG_I_FLAGS}"
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pgmappingcooperativo
)

### Generating Services

### Generating Module File
_generate_module_cpp(pgmappingcooperativo
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pgmappingcooperativo
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(pgmappingcooperativo_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(pgmappingcooperativo_generate_messages pgmappingcooperativo_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/goalList.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_cpp _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/frontierReport.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_cpp _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/infoCentro.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_cpp _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/asignacion.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_cpp _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/asignacionCelda.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_cpp _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/resumenInstancia.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_cpp _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/mapMergedInfo.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_cpp _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/takeobjetive.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_cpp _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_cpp _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Edge.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_cpp _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Graph.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_cpp _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/SegmentAuction.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_cpp _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/SegmentBid.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_cpp _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/SegmentAssignment.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_cpp _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/FrontierBid.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_cpp _pgmappingcooperativo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pgmappingcooperativo_gencpp)
add_dependencies(pgmappingcooperativo_gencpp pgmappingcooperativo_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pgmappingcooperativo_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/goalList.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_eus(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/frontierReport.msg"
  "${MSG_I_FLAGS}"
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/infoCentro.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_eus(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/infoCentro.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_eus(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/asignacion.msg"
  "${MSG_I_FLAGS}"
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/asignacionCelda.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_eus(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/asignacionCelda.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_eus(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/resumenInstancia.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_eus(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/mapMergedInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/nav_msgs/cmake/../msg/OccupancyGrid.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/MapMetaData.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_eus(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/takeobjetive.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/nav_msgs/cmake/../msg/OccupancyGrid.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/MapMetaData.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_eus(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_eus(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Edge.msg"
  "${MSG_I_FLAGS}"
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_eus(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Graph.msg"
  "${MSG_I_FLAGS}"
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Edge.msg;/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_eus(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/SegmentAuction.msg"
  "${MSG_I_FLAGS}"
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Edge.msg;/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Graph.msg;/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_eus(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/SegmentBid.msg"
  "${MSG_I_FLAGS}"
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_eus(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/SegmentAssignment.msg"
  "${MSG_I_FLAGS}"
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_eus(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/FrontierBid.msg"
  "${MSG_I_FLAGS}"
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pgmappingcooperativo
)

### Generating Services

### Generating Module File
_generate_module_eus(pgmappingcooperativo
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pgmappingcooperativo
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(pgmappingcooperativo_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(pgmappingcooperativo_generate_messages pgmappingcooperativo_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/goalList.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_eus _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/frontierReport.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_eus _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/infoCentro.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_eus _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/asignacion.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_eus _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/asignacionCelda.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_eus _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/resumenInstancia.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_eus _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/mapMergedInfo.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_eus _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/takeobjetive.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_eus _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_eus _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Edge.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_eus _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Graph.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_eus _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/SegmentAuction.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_eus _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/SegmentBid.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_eus _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/SegmentAssignment.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_eus _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/FrontierBid.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_eus _pgmappingcooperativo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pgmappingcooperativo_geneus)
add_dependencies(pgmappingcooperativo_geneus pgmappingcooperativo_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pgmappingcooperativo_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/goalList.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_lisp(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/frontierReport.msg"
  "${MSG_I_FLAGS}"
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/infoCentro.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_lisp(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/infoCentro.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_lisp(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/asignacion.msg"
  "${MSG_I_FLAGS}"
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/asignacionCelda.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_lisp(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/asignacionCelda.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_lisp(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/resumenInstancia.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_lisp(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/mapMergedInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/nav_msgs/cmake/../msg/OccupancyGrid.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/MapMetaData.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_lisp(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/takeobjetive.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/nav_msgs/cmake/../msg/OccupancyGrid.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/MapMetaData.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_lisp(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_lisp(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Edge.msg"
  "${MSG_I_FLAGS}"
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_lisp(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Graph.msg"
  "${MSG_I_FLAGS}"
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Edge.msg;/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_lisp(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/SegmentAuction.msg"
  "${MSG_I_FLAGS}"
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Edge.msg;/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Graph.msg;/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_lisp(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/SegmentBid.msg"
  "${MSG_I_FLAGS}"
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_lisp(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/SegmentAssignment.msg"
  "${MSG_I_FLAGS}"
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_lisp(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/FrontierBid.msg"
  "${MSG_I_FLAGS}"
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pgmappingcooperativo
)

### Generating Services

### Generating Module File
_generate_module_lisp(pgmappingcooperativo
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pgmappingcooperativo
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(pgmappingcooperativo_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(pgmappingcooperativo_generate_messages pgmappingcooperativo_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/goalList.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_lisp _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/frontierReport.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_lisp _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/infoCentro.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_lisp _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/asignacion.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_lisp _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/asignacionCelda.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_lisp _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/resumenInstancia.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_lisp _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/mapMergedInfo.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_lisp _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/takeobjetive.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_lisp _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_lisp _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Edge.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_lisp _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Graph.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_lisp _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/SegmentAuction.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_lisp _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/SegmentBid.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_lisp _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/SegmentAssignment.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_lisp _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/FrontierBid.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_lisp _pgmappingcooperativo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pgmappingcooperativo_genlisp)
add_dependencies(pgmappingcooperativo_genlisp pgmappingcooperativo_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pgmappingcooperativo_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/goalList.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_nodejs(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/frontierReport.msg"
  "${MSG_I_FLAGS}"
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/infoCentro.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_nodejs(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/infoCentro.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_nodejs(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/asignacion.msg"
  "${MSG_I_FLAGS}"
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/asignacionCelda.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_nodejs(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/asignacionCelda.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_nodejs(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/resumenInstancia.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_nodejs(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/mapMergedInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/nav_msgs/cmake/../msg/OccupancyGrid.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/MapMetaData.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_nodejs(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/takeobjetive.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/nav_msgs/cmake/../msg/OccupancyGrid.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/MapMetaData.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_nodejs(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_nodejs(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Edge.msg"
  "${MSG_I_FLAGS}"
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_nodejs(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Graph.msg"
  "${MSG_I_FLAGS}"
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Edge.msg;/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_nodejs(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/SegmentAuction.msg"
  "${MSG_I_FLAGS}"
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Edge.msg;/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Graph.msg;/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_nodejs(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/SegmentBid.msg"
  "${MSG_I_FLAGS}"
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_nodejs(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/SegmentAssignment.msg"
  "${MSG_I_FLAGS}"
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_nodejs(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/FrontierBid.msg"
  "${MSG_I_FLAGS}"
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pgmappingcooperativo
)

### Generating Services

### Generating Module File
_generate_module_nodejs(pgmappingcooperativo
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pgmappingcooperativo
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(pgmappingcooperativo_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(pgmappingcooperativo_generate_messages pgmappingcooperativo_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/goalList.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_nodejs _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/frontierReport.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_nodejs _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/infoCentro.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_nodejs _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/asignacion.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_nodejs _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/asignacionCelda.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_nodejs _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/resumenInstancia.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_nodejs _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/mapMergedInfo.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_nodejs _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/takeobjetive.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_nodejs _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_nodejs _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Edge.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_nodejs _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Graph.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_nodejs _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/SegmentAuction.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_nodejs _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/SegmentBid.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_nodejs _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/SegmentAssignment.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_nodejs _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/FrontierBid.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_nodejs _pgmappingcooperativo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pgmappingcooperativo_gennodejs)
add_dependencies(pgmappingcooperativo_gennodejs pgmappingcooperativo_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pgmappingcooperativo_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/goalList.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_py(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/frontierReport.msg"
  "${MSG_I_FLAGS}"
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/infoCentro.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_py(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/infoCentro.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_py(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/asignacion.msg"
  "${MSG_I_FLAGS}"
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/asignacionCelda.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_py(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/asignacionCelda.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_py(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/resumenInstancia.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_py(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/mapMergedInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/nav_msgs/cmake/../msg/OccupancyGrid.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/MapMetaData.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_py(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/takeobjetive.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/nav_msgs/cmake/../msg/OccupancyGrid.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/MapMetaData.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_py(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_py(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Edge.msg"
  "${MSG_I_FLAGS}"
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_py(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Graph.msg"
  "${MSG_I_FLAGS}"
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Edge.msg;/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_py(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/SegmentAuction.msg"
  "${MSG_I_FLAGS}"
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Edge.msg;/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Graph.msg;/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_py(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/SegmentBid.msg"
  "${MSG_I_FLAGS}"
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_py(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/SegmentAssignment.msg"
  "${MSG_I_FLAGS}"
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pgmappingcooperativo
)
_generate_msg_py(pgmappingcooperativo
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/FrontierBid.msg"
  "${MSG_I_FLAGS}"
  "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pgmappingcooperativo
)

### Generating Services

### Generating Module File
_generate_module_py(pgmappingcooperativo
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pgmappingcooperativo
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(pgmappingcooperativo_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(pgmappingcooperativo_generate_messages pgmappingcooperativo_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/goalList.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_py _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/frontierReport.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_py _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/infoCentro.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_py _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/asignacion.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_py _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/asignacionCelda.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_py _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/resumenInstancia.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_py _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/mapMergedInfo.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_py _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/takeobjetive.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_py _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Point2D.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_py _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Edge.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_py _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/Graph.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_py _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/SegmentAuction.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_py _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/SegmentBid.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_py _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/SegmentAssignment.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_py _pgmappingcooperativo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fede/catkin_ws/src/pgmappingcooperativo/msg/FrontierBid.msg" NAME_WE)
add_dependencies(pgmappingcooperativo_generate_messages_py _pgmappingcooperativo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pgmappingcooperativo_genpy)
add_dependencies(pgmappingcooperativo_genpy pgmappingcooperativo_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pgmappingcooperativo_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pgmappingcooperativo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pgmappingcooperativo
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(pgmappingcooperativo_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(pgmappingcooperativo_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(pgmappingcooperativo_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pgmappingcooperativo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pgmappingcooperativo
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(pgmappingcooperativo_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(pgmappingcooperativo_generate_messages_eus nav_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(pgmappingcooperativo_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pgmappingcooperativo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pgmappingcooperativo
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(pgmappingcooperativo_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(pgmappingcooperativo_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(pgmappingcooperativo_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pgmappingcooperativo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pgmappingcooperativo
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(pgmappingcooperativo_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(pgmappingcooperativo_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(pgmappingcooperativo_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pgmappingcooperativo)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pgmappingcooperativo\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pgmappingcooperativo
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(pgmappingcooperativo_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(pgmappingcooperativo_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(pgmappingcooperativo_generate_messages_py std_msgs_generate_messages_py)
endif()
