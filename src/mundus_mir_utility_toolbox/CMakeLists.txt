cmake_minimum_required(VERSION 3.8)
project(utility_toolbox)

# default to c++ 20
if(NOT CMAKE_CXX_STANDARD) 
  set(CMAKE_CXX_STANDARD 20)
endif()

# set compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# add directories
include_directories(include)

# declare library  
add_library(${PROJECT_NAME} SHARED
  src/network.cc
  src/distributions.cc
  src/string_methods.cc
  src/orientations.cc
)

# add dependencies
ament_target_dependencies(${PROJECT_NAME}
)

# install library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install directories
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies()
ament_package()
