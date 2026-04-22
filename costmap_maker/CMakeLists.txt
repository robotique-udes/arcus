set(PROJECT_NAME "costmap_maker")
set(EXECUTABLE_NAME "costmap_node")
set(CPP_FILE_PATH "src/costmap_maker.cpp")
set(IS_PACKAGE_EXECUTABLE TRUE)
set(IS_PACKAGE_INTERFACE FALSE)

set(DEPENDENCY_LIST 
  "ament_cmake"
  "rclcpp"
  "rclpy"
  "geometry_msgs" 
  "nav_msgs" 
  "sensor_msgs" 
  "std_msgs"
  "tf2"
  "tf2_ros"
  )

cmake_minimum_required(VERSION 3.8)
project(${PROJECT_NAME})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
foreach(PACKAGE ${DEPENDENCY_LIST})
  find_package(${PACKAGE} REQUIRED)
endforeach()

# Include Cpp "include" directory
include_directories(include)

IF (IS_PACKAGE_EXECUTABLE)
  # Create Cpp executable
  add_executable(${EXECUTABLE_NAME} ${CPP_FILE_PATH})
  ament_target_dependencies(${EXECUTABLE_NAME} 
    ${DEPENDENCY_LIST}
  )

  # Install Cpp executables
  install(TARGETS
    ${EXECUTABLE_NAME}
    DESTINATION lib/${PROJECT_NAME})


  # Install config dir
  install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
  )

  # Install launch dir
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )
ENDIF()

ament_package()
