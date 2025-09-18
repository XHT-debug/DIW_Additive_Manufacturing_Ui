cmake_minimum_required(VERSION 3.8)
project(motion_ui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set(CMAKE_PREFIX_PATH "/home/xht/Qt/6.9.0/gcc_64/lib")
# set(Qt6_DIR "/home/xht/Qt/6.9.0/gcc_64/lib/cmake/Qt6")

# # Set Qt version explicitly
# set(QT_MAJOR_VERSION 6)
# set(QT_MINOR_VERSION 9)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(my_custom_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core PrintSupport)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  rviz_common
  rviz_default_plugins
  rviz_rendering
  rviz_ogre_vendor
  my_custom_msgs
)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
set_source_files_properties(Resources.qrc PROPERTIES AUTORCC ON)

add_executable(motion_ui
src/main.cpp
src/ros_qtgui.cpp
src/ros_qtgui.h
src/ros_qtgui.ui
src/qcustomplot.cpp
Resources.qrc
)


ament_target_dependencies(motion_ui ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

# target_link_directories(motion_ui
# PUBLIC include)

# ament_target_dependenicies(motion_ui
#   "moveit_ros_planning_interface"
#   "moveit_visual_tools"
#   "rclcpp"
# )

target_link_libraries(motion_ui Qt5::Widgets Qt5::Core Qt5::PrintSupport)

install(TARGETS motion_ui
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_include_directories(motion_ui
  PUBLIC include
  PUBLIC src
  PUBLIC /opt/ros/humble/include
  PUBLIC /opt/ros/humble/include/rviz_common
  PUBLIC /opt/ros/humble/include/rviz_rendering)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
