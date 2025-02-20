cmake_minimum_required(VERSION 3.8)
project(transform_helper)

# Default to C++17.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Require that dependencies from package.xml be available.
find_package(ament_cmake_auto REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(${PROJECT_NAME}_SRC
  src/transform_helper.cpp
)

set(${PROJECT_NAME}_HEADER
  include/transform_helper/transform_helper.hpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_SRC}
  ${${PROJECT_NAME}_HEADER}
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
#   set(TEST_SOURCES test/test_rvc.cpp)
#   set(TEST_RVC_EXE test_rvc)
#   ament_add_gtest(${TEST_RVC_EXE} ${TEST_SOURCES})
#   target_link_libraries(${TEST_RVC_EXE} ${PROJECT_NAME})
# endif()

# Create & install ament package.
ament_auto_package()
