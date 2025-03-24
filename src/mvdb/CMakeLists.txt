cmake_minimum_required(VERSION 3.8)
project(mvdb)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic -Wno-reorder -Wno-sign-compare)
  # add_compile_options(-Wall -Wextra -Wno-reorder -Wno-sign-compare)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(mvdb_interface REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP)

if(OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(${PROJECT_NAME}_include_dirs
  include/mvdb
  ${EIGEN3_INCLUDE_DIRS}
)

include_directories(${${PROJECT_NAME}_include_dirs})

set(${PROJECT_NAME}_test_targets
  test_octree
  test_utils
  test_record
  test_quantizer
  test_projection
  test_config
)

set(${PROJECT_NAME}_exec_targets
  main
)

set(${PROJECT_NAME}_lib_targets
  math_utils
  quantizer
  octree
  record
  utils
  projection
  ros_utils
  buffer_nodes
  mapper_nodes
  config
)

set(${PROJECT_NAME}_rosdeps 
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  mvdb_interface
)

set(${PROJECT_NAME}_rostargets 
  ros_utils
  buffer_nodes
  mapper_nodes
  config
  main
)

foreach(exe ${${PROJECT_NAME}_exec_targets})
  add_executable(${exe} src/${exe}.cpp)
endforeach()

foreach(lib ${${PROJECT_NAME}_lib_targets})
  add_library(${lib} lib/${PROJECT_NAME}/${lib}.cpp)
  target_link_libraries(${lib} ${${PROJECT_NAME}_libraries})
  set(${PROJECT_NAME}_libraries ${${PROJECT_NAME}_libraries} ${lib})
endforeach()

foreach(exe ${${PROJECT_NAME}_exec_targets})
  target_link_libraries(${exe} ${${PROJECT_NAME}_libraries})
endforeach()

foreach(rostarget ${${PROJECT_NAME}_rostargets})
  ament_target_dependencies(${rostarget} ${${PROJECT_NAME}_rosdeps})
endforeach()


if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)

  ament_find_gtest()
  message(STATUS "libraries: " ${${PROJECT_NAME}_libraries})
  foreach( test ${${PROJECT_NAME}_test_targets} )
    ament_add_gtest(${test} test/${test}.cpp)
    target_include_directories(${test} PUBLIC
      <BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    target_link_libraries(${test} ${${PROJECT_NAME}_libraries})
  endforeach()

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS ${${PROJECT_NAME}_lib_targets} ${${PROJECT_NAME}_exec_targets} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
