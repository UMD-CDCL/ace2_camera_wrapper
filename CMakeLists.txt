cmake_minimum_required(VERSION 3.8)
project(ace2_camera_wrapper)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)

# The pylon SDK is not an ament package, so it is found the way Basler's own
# samples do it. The SDK ships lib/cmake/Findpylon.cmake, which asks pylon-config
# for the include and link flags of the installed version (including the rpath to
# /opt/pylon/lib, so the node finds libpylonbase.so without LD_LIBRARY_PATH).
# Setting PYLON_ROOT in the environment overrides the default prefix.
if(DEFINED ENV{PYLON_ROOT})
  set(pylon_ROOT_DIR "$ENV{PYLON_ROOT}" CACHE PATH "pylon SDK install prefix")
else()
  set(pylon_ROOT_DIR "/opt/pylon" CACHE PATH "pylon SDK install prefix")
endif()
list(APPEND CMAKE_MODULE_PATH "${pylon_ROOT_DIR}/lib/cmake")
find_package(pylon 8.0.0 REQUIRED)
message(STATUS "Found pylon ${pylon_VERSION} at ${pylon_ROOT_DIR}")

# pylon_LIBRARIES arrives as one pylon-config string. Splitting it into real list
# items keeps the raw flags out of the exported target, which is what lets a
# downstream package link this component without having to find pylon itself.
separate_arguments(pylon_LINK_FLAGS UNIX_COMMAND "${pylon_LIBRARIES}")

set(dependencies
  rclcpp
  rclcpp_components
  sensor_msgs
  std_srvs
  image_transport
  camera_info_manager
)

# ---------------------------------------------------------------- component ---
# Built as a shared library so rclcpp_components can load it into a container.
# Sharing a process with its subscribers is what makes intra-process transfer of
# a 15 MB frame possible at all.
add_library(${PROJECT_NAME} SHARED
  src/ace2_camera_wrapper_node.cpp
)
target_compile_definitions(${PROJECT_NAME} PRIVATE "ACE2_CAMERA_WRAPPER_BUILDING_LIBRARY")
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${pylon_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} ${pylon_LINK_FLAGS})

rclcpp_components_register_nodes(${PROJECT_NAME}
  "ace2_camera_wrapper::Ace2CameraWrapperNode")

# --------------------------------------------------------------- executable ---
# Same class, run in its own process, so `ros2 run` and a plain launch Node
# action keep working exactly as before.
add_executable(ace2_camera_wrapper_node src/ace2_camera_wrapper_main.cpp)
target_link_libraries(ace2_camera_wrapper_node ${PROJECT_NAME})
ament_target_dependencies(ace2_camera_wrapper_node ${dependencies})

# ------------------------------------------------------------------ install ---
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ace2_camera_wrapper_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Install launch files within "launch/" directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Exported so another package can `find_package(ace2_camera_wrapper)` and either
# link the library directly or list the component in a container.
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

ament_package()
