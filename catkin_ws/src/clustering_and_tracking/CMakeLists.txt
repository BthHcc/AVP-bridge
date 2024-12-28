cmake_minimum_required(VERSION 3.0.2)
project(clustering_and_tracking)

# 找到catkin构建工具和所需组件
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

# 声明该包为catkin包
catkin_package()

# 安装Python脚本
# catkin_install_python(PROGRAMS
#   scripts/ros_perception_info.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# 包含目录（如果有头文件，可添加 include/ 目录路径）
include_directories(
  ${catkin_INCLUDE_DIRS}
)
