cmake_minimum_required(VERSION 3.5)
project(final_student)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-dev -Wunused-result)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(Python COMPONENTS NumPy REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(xacro REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${Python_NumPy_INCLUDE_DIRS})
include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${xacro_INCLUDE_DIRS})
link_directories(${xacro_LIBRARY_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

macro(macro1 arg)
  add_executable(${arg} src/${arg}.cpp)
  target_link_libraries(${arg}${catkin_LIBRARIES} ${PCL_LIBRARIES})
  ament_target_dependencies(${arg} rclcpp std_msgs sensor_msgs geometry_msgs gazebo_msgs xacro Eigen3)
  install(TARGETS
          ${arg}
          DESTINATION lib/${PROJECT_NAME})
endmacro()

macro1(navigate_20200726)

add_executable(teleop_cpp src/teleop_cpp_ros2.cpp)
ament_target_dependencies(teleop_cpp rclcpp std_msgs sensor_msgs)
target_link_libraries(teleop_cpp ${catkin_LIBRARIES})

add_executable(evaluate src/evaluate.cpp)
target_link_libraries(evaluate ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} ${xacro_LIBRARY_DIRS})
ament_target_dependencies(evaluate rclcpp std_msgs sensor_msgs std_srvs geometry_msgs gazebo_msgs xacro Eigen3)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

add_compile_options(-Wno-dev -Wunused-result)

install(DIRECTORY
  launch
  rsc
  worlds
  DESTINATION share/${PROJECT_NAME})


install(TARGETS
        evaluate
        teleop_cpp
        DESTINATION lib/${PROJECT_NAME})


# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
