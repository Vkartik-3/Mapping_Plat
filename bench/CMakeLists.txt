# Google Benchmark harness for the HD map pipeline extensions (Phases 1-3).
# Self-contained: compiles the pipeline sources plus the bundled benchmark
# submodule, so it builds without the heavy Boost/Osmium stack.
#
# Standalone use:  cmake -S bench -B build/bench -DCMAKE_BUILD_TYPE=Release
#                  cmake --build build/bench && ./build/bench/hdmap_bench

cmake_minimum_required(VERSION 3.16)
project(hdmap_pipeline_bench LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

get_filename_component(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
set(LIB_SRC "${REPO_ROOT}/src/library")
set(NANOFLANN_INC "${REPO_ROOT}/third_party/nanoflann/include")

set(PIPELINE_SOURCES
        ${LIB_SRC}/src/types/io/track/kitti_oxts_importer.cpp
        ${LIB_SRC}/src/types/io/lidar/kitti_lidar_reader.cpp
        ${LIB_SRC}/src/geometry/centerline/centerline_extractor.cpp
        ${LIB_SRC}/src/geometry/road_graph/osm_importer.cpp
        ${LIB_SRC}/src/geometry/road_graph/map_matcher.cpp
        ${LIB_SRC}/src/geometry/lidar/ground_extractor.cpp)

add_library(hdmap_pipeline_bench_lib STATIC ${PIPELINE_SOURCES})
target_include_directories(hdmap_pipeline_bench_lib PUBLIC "${LIB_SRC}/include")
target_include_directories(hdmap_pipeline_bench_lib SYSTEM PUBLIC "${NANOFLANN_INC}")

# Google Benchmark from the submodule (skip its own tests).
set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
set(BENCHMARK_ENABLE_GTEST_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory("${REPO_ROOT}/third_party/benchmark" benchmark-build EXCLUDE_FROM_ALL)

add_executable(hdmap_bench
        bench_main.cpp
        bench_kitti.cpp
        bench_graph.cpp
        bench_kdtree.cpp)

target_link_libraries(hdmap_bench PRIVATE hdmap_pipeline_bench_lib benchmark::benchmark)
