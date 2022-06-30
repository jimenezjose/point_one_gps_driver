cmake_minimum_required(VERSION 3.8)
project(point_one_gps_driver)

include(ExternalProject)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
# Require that dependencies from package.xml be available.
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(${PROJECT_NAME}_SRC
  src/point_one_gps_node.cpp
)

# Create this project as an executable.
ament_auto_add_executable(${PROJECT_NAME}_node_exe
  ${${PROJECT_NAME}_SRC}
)

# Set location to store external project.
set(EXTERNAL_INSTALL_LOCATION 
  ${CMAKE_BINARY_DIR}/p1_fusion_engine_client
)

# Specify external project as a target.
ExternalProject_Add(project_p1_fusion_engine_client
  GIT_REPOSITORY https://github.com/PointOneNav/fusion-engine-client.git
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${EXTERNAL_INSTALL_LOCATION}
  PREFIX ${EXTERNAL_INSTALL_LOCATION}
)

# Include header files required for external project.
include_directories(${EXTERNAL_INSTALL_LOCATION}/include)

# Append shared library path to dynamic linker in rpath (runtime search path).
SET(CMAKE_EXE_LINKER_FLAGS 
  "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath -Wl,${EXTERNAL_INSTALL_LOCATION}/lib"
)

# Library is not built until there is a dependency.
add_dependencies(${PROJECT_NAME}_node_exe project_p1_fusion_engine_client)

# Link point_one_gps node to fusion_engine_client library.
target_link_libraries(${PROJECT_NAME}_node_exe fusion_engine_client)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  param
)
