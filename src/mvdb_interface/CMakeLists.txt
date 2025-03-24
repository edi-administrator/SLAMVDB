cmake_minimum_required(VERSION 3.8)
project(mvdb_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VectorImage.msg"
  "msg/EmbeddingMessage.msg"
  "msg/LoopMessage.msg"
  "srv/Segmentation.srv"
  "srv/Embedding.srv"
  "srv/MapQuerySemantic.srv"
  "srv/MapQuerySpatial.srv"
  "srv/MapQueryDiscrete.srv"
  "srv/MapQueryDiscretePlanar.srv"
  DEPENDENCIES sensor_msgs std_msgs geometry_msgs nav_msgs builtin_interfaces
)

ament_package()