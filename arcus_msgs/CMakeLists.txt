# ============== BEGIN USER CONFIGURATION ==============
set(PROJECT_NAME "arcus_msgs")
set(EXECUTABLE_NAME "")
set(CPP_FILE_PATH "")
set(IS_PACKAGE_EXECUTABLE FALSE)
set(IS_PACKAGE_INTERFACE TRUE)

# If dependencies need to be built before this package, add them in the package.xml file in the <build_depend> tags.
set(DEPENDENCY_LIST 
  "ament_cmake"
  "std_msgs"
  )

  set(MSGS
    "msg/ControllerType.msg"
    "msg/ErrorCode.msg"
    )
  set(SRVS
    "srv/DemoSrv.srv"
    )

# =============== END USER CONFIGURATION ===============
cmake_minimum_required(VERSION 3.8)
project(${PROJECT_NAME})

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
foreach(PACKAGE ${DEPENDENCY_LIST})
  find_package(${PACKAGE} REQUIRED)
endforeach()

IF (IS_PACKAGE_INTERFACE)
  find_package(rosidl_default_generators REQUIRED)
  # Generate interfaces here
  rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSGS}
  ${SRVS}
  DEPENDENCIES ${DEPENDENCY_LIST}
 )
ENDIF()

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
ENDIF()

ament_package()