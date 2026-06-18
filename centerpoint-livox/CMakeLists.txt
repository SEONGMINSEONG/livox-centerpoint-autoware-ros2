cmake_minimum_required(VERSION 3.5)
project(centerpoint)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(CUDA REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs  REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ws_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

if(${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL  "aarch64")
  set(CMAKE_C_COMPILER /usr/bin/aarch64-linux-gnu-gcc)
  set(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++)
  set(CUDA_INSTALL_TARGET_DIR targets/aarch64-linux)
elseif(${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL "x86_64")
  set(CMAKE_C_COMPILER /usr/bin/gcc)
  set(CMAKE_CXX_COMPILER /usr/bin/g++)
  set(CUDA_INSTALL_TARGET_DIR targets/x86_64-linux)
endif()

set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda)
set(CUDA_INCLUDE_DIRS ${CUDA_TOOLKIT_ROOT_DIR}/${CUDA_INSTALL_TARGET_DIR}/include)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-Wno-deprecated-declarations -O2")
add_compile_options(-W)

set( SMS 75 86 87)
foreach(sm ${SMS})
	set(GENCODE ${GENCODE} -gencode arch=compute_${sm},code=sm_${sm})
endforeach()
set(HIGHEST_SM 87)
set(GENCODE ${GENCODE} -gencode arch=compute_${HIGHEST_SM},code=compute_${HIGHEST_SM})

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}
  -ccbin ${CMAKE_CXX_COMPILER}
  -Xcompiler -DWIN_INTERFACE_CUSTOM
)

if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  message("Using Debug Mode")
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -g -G --ptxas-options=-v)
endif()

if(${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL "x86_64")
  set(TENSORRT_INCLUDE_DIRS /usr/include/x86_64-linux-gnu CACHE PATH "TensorRT include path")
  set(TENSORRT_LIBRARY_DIRS /lib/x86_64-linux-gnu CACHE PATH "TensorRT library path")
elseif(${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL "aarch64")
  set(TENSORRT_INCLUDE_DIRS /usr/include/aarch64-linux-gnu CACHE PATH "TensorRT include path")
  set(TENSORRT_LIBRARY_DIRS /usr/lib/aarch64-linux-gnu CACHE PATH "TensorRT library path")
endif()

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${ws_msgs_INCLUDE_DIRS}
)

link_directories(
	${TENSORRT_LIBRARY_DIRS}
)

file(GLOB_RECURSE SOURCE_FILES
  src/*.cu
  src/plugin/*.cu
  src/*.cpp
  src/plugin/*.cpp
)
list(FILTER SOURCE_FILES EXCLUDE REGEX ".*/src/app/.*")

cuda_add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} nvinfer nvonnxparser yaml-cpp)

add_executable(${PROJECT_NAME}_node src/app/detection.cpp src/app/main.cpp)

 target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
    ${PCL_LIBRARIES}
    yaml-cpp
  )

ament_target_dependencies(${PROJECT_NAME}_node
    rclcpp 
    ament_index_cpp
    sensor_msgs  
    std_msgs
    geometry_msgs
    tf2_ros
    tf2_geometry_msgs
    tf2_eigen
    pcl_conversions
    ws_msgs 
    visualization_msgs
    CUDA
    yaml-cpp
  ) 

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  rviz
  cfgs
  model
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
