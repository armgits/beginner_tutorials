cmake_minimum_required(VERSION 3.5)
project(beginner_tutorials)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Package dependencies
set(PACKAGE_DEPENDENCIES
  ament_cmake
  rclcpp
  tf2
  tf2_ros
  std_msgs
  std_srvs
)

foreach(dependency IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(std_srvs REQUIRED)

# Add gtest here
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  # un-comment the following lines skip linters (TRUE = skip that test)
  # set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  # Run all lint tests in package.xml except those listed above
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}_test test/test.cpp)
  ament_target_dependencies(${PROJECT_NAME}_test ${PACKAGE_DEPENDENCIES})
  target_include_directories(${PROJECT_NAME}_test
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
  )
endif()

add_executable(first_astronaut src/first_astronaut.cpp)
ament_target_dependencies(first_astronaut ${PACKAGE_DEPENDENCIES})
target_include_directories(first_astronaut
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(second_astronaut src/second_astronaut.cpp)
ament_target_dependencies(second_astronaut ${PACKAGE_DEPENDENCIES})
target_include_directories(second_astronaut
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS
  first_astronaut
  second_astronaut
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
