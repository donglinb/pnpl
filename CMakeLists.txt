cmake_minimum_required(VERSION 2.8)
project(pnpl)

if(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native")

find_package(Eigen3 REQUIRED)
message("-- Found Eigen3 ${Eigen3_VERSION} at ${EIGEN3_INCLUDE_DIR}")

include_directories(
${PROJECT_SOURCE_DIR}
${EIGEN3_INCLUDE_DIR}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

aux_source_directory(./PoseLib/ POSELIB_SRCS)
add_library(pose_lib SHARED ${POSELIB_SRCS})

aux_source_directory(./PoseComputer/ POSE_COMPUTER_SRCS)
add_library(pose_computer SHARED ${POSE_COMPUTER_SRCS})
target_link_libraries(pose_computer pose_lib)

aux_source_directory(./RANSAC/ RANSAC_SRCS)
add_library(ransac SHARED ${RANSAC_SRCS})
target_link_libraries(ransac pose_computer)

aux_source_directory(./PnPL/ PNPL_SRCS)
add_library(pnpl SHARED ${PNPL_SRCS})
target_link_libraries(pnpl pose_computer ransac)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build)

add_executable(benchmark test/benchmark.cc test/problem_generator.cc)
target_link_libraries(benchmark pose_lib)

add_executable(test_pnpl test/test_pnpl.cpp test/problem_generator.cc)
target_link_libraries(test_pnpl pnpl)

add_executable(test_pnpl2 test/test_pnpl2.cpp)
target_link_libraries(test_pnpl2 pnpl)

