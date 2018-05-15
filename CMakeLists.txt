project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/main.cpp src/map/map.cpp src/vehicle/vehicle.cpp src/prediction/predictor.cpp src/planning/planner.cpp src/planning/trajectory_generator.cpp src/planning/driving_strategy.cpp)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
include_directories(/usr/local/opt/openssl/include)
include_directories(/Eigen-3.3/Eigen)

link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

add_executable(path_planning ${sources})

target_include_directories(path_planning PRIVATE)
target_link_libraries(path_planning pthread z ssl uv uWS)

