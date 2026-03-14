cmake_minimum_required(VERSION 3.5)
project(range_libc)  # renamed to match package.xml

# ament_cmake must be found first for colcon compatibility
find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# ───────────────────────────────────────────────
# Options
# ───────────────────────────────────────────────
option(WITH_CUDA "Compile CUDA ray cast variants" ON)

set(CHUNK_SIZE 262144)
set(CHUNK_THREADS 256)

# ───────────────────────────────────────────────
# Output directories
# ───────────────────────────────────────────────
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# ───────────────────────────────────────────────
# Base path definitions
# ───────────────────────────────────────────────
set(BASEPATH "${CMAKE_SOURCE_DIR}")
add_definitions(-DBASEPATH=${BASEPATH})

add_subdirectory(${BASEPATH}/vendor/gflags)

include_directories(${BASEPATH})
include_directories(${PROJECT_SOURCE_DIR}/vendor)

# ───────────────────────────────────────────────
# C++14 (upgraded from C++11 for ROS2 compatibility)
# ───────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-march=native -ffast-math -fno-math-errno ${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_BUILD_TYPE Release)

# ───────────────────────────────────────────────
# Build the C++ executable and shared library (with or without CUDA)
# ───────────────────────────────────────────────
if(WITH_CUDA)
  add_definitions(-DUSE_CUDA=1)
  add_definitions(-DCHUNK_SIZE=${CHUNK_SIZE})
  add_definitions(-DNUM_THREADS=${CHUNK_THREADS})

  find_package(CUDA REQUIRED)

  # Shared library for downstream packages (e.g. particle_filter)
  file(GLOB LIB_CUDA_SRCS
    "includes/kernels.cu"
    "vendor/lodepng/lodepng.cpp"
  )
  cuda_add_library(range_libc_lib SHARED ${LIB_CUDA_SRCS})
  set_target_properties(range_libc_lib PROPERTIES OUTPUT_NAME range_libc)
  target_link_libraries(range_libc_lib pthread)

  # Executable – links against the shared library instead of recompiling CUDA sources
  cuda_add_executable(range_libc_exe main.cpp)
  set_target_properties(range_libc_exe PROPERTIES OUTPUT_NAME range_libc)
  target_link_libraries(range_libc_exe range_libc_lib gflags pthread)
else()
  add_definitions(-DUSE_CUDA=0)

  # Shared library for downstream packages
  add_library(range_libc_lib SHARED "vendor/lodepng/lodepng.cpp")
  set_target_properties(range_libc_lib PROPERTIES OUTPUT_NAME range_libc)
  target_link_libraries(range_libc_lib pthread)

  # Executable
  file(GLOB SRCS
    main.cpp
    "vendor/lodepng/lodepng.cpp"
  )
  add_executable(range_libc_exe ${SRCS})
  set_target_properties(range_libc_exe PROPERTIES OUTPUT_NAME range_libc)
  target_link_libraries(range_libc_exe gflags pthread)
endif()

# ───────────────────────────────────────────────
# Build the Python3 Cython extension
# ───────────────────────────────────────────────
if(WITH_CUDA)
  set(RANGE_LIBC_CUDA_FLAG "WITH_CUDA=True")
else()
  set(RANGE_LIBC_CUDA_FLAG "WITH_CUDA=False")
endif()

#execute_process(
#  COMMAND ${Python3_EXECUTABLE} setup.py build_ext --inplace
#  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/pywrappers
#  RESULT_VARIABLE PY_BUILD_RESULT
#)

#if(NOT PY_BUILD_RESULT EQUAL 0)
#  message(FATAL_ERROR
#    "range_libc Python extension build failed.\n"
#    "Make sure cython<3.0 and numpy are installed:\n"
#    "  pip3 install 'cython<3.0' numpy"
#  )
#endif()

# ───────────────────────────────────────────────
# Install C++ targets
# ───────────────────────────────────────────────
install(TARGETS range_libc_exe
  RUNTIME DESTINATION bin
)
install(TARGETS range_libc_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# ───────────────────────────────────────────────
# Install headers
# ───────────────────────────────────────────────
install(DIRECTORY ${CMAKE_SOURCE_DIR}/vendor/lodepng DESTINATION include/vendor)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/vendor         DESTINATION include)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/includes       DESTINATION include)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/includes/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
)

# ───────────────────────────────────────────────
# Install Python extension .so into ROS2 overlay
# ───────────────────────────────────────────────
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/pywrappers/
  DESTINATION lib/python3/dist-packages
  FILES_MATCHING PATTERN "*.so"
)


# ───────────────────────────────────────────────
# THIS IS THE KEY LINE — generates range_libcConfig.cmake
# so that find_package(range_libc) works in particle_filter
# ───────────────────────────────────────────────
ament_export_include_directories(include)
ament_export_libraries(range_libc_lib)
ament_package()