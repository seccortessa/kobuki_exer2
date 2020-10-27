cmake_minimum_required(VERSION 3.0.2)
project(kobuki_exer2)


find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  kobuki_description
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/param_test.cfg
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

