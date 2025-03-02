cmake_minimum_required(VERSION 3.8)
project(pid_siso)

# set compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set cmake standard
set(CMAKE_CXX_STANDARD 20)

# find dependenceis
find_package(ament_cmake)

# add directories
include_directories(include)


# declare library
add_library(${PROJECT_NAME} SHARED
    src/pid_siso.cpp
)

# add dependencies
ament_target_dependencies(${PROJECT_NAME}
)

#install library
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