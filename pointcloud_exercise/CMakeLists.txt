cmake_minimum_required(VERSION 3.8)
project(pointcloud_exercise)

# Enable strict compilation warnings for GCC/Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(PCL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_py REQUIRED)
find_package(tf2_eigen REQUIRED)  
# find_package(cnpy REQUIRED)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

# Add executable and link libraries
add_executable(transform_pointcloud src/transform_pointcloud.cpp)
target_include_directories(transform_pointcloud PUBLIC
  ${PCL_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_eigen_INCLUDE_DIRS}
  include
)
target_link_libraries(transform_pointcloud
  ${PCL_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${tf2_eigen_LIBRARIES}
)

target_link_libraries(transform_pointcloud
  ${rclcpp_LIBRARIES}
  ${PCL_LIBRARIES}
  cnpy  # Link the cnpy library
)

target_compile_definitions(transform_pointcloud PUBLIC ${PCL_DEFINITIONS})

# Specify ament dependencies
ament_target_dependencies(
  transform_pointcloud
  rclcpp
  std_msgs
  sensor_msgs
  pcl_conversions
  tf2_ros
  tf2_eigen
)

# Install executables and additional files
install(TARGETS
  transform_pointcloud
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} OPTIONAL)
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME} OPTIONAL)
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME} OPTIONAL)

# Install Python modules if needed
ament_python_install_package(${PROJECT_NAME})

# Enable testing and linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export package
ament_package()

