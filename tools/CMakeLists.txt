# Real KITTI GPS -> real OSM road-graph map-matching workflow executable.
# Self-contained: compiles the pipeline sources plus nanoflann, no Boost/Osmium.
#
# Standalone use:  cmake -S tools -B build/tools -DCMAKE_BUILD_TYPE=Release
#                  cmake --build build/tools
#                  KITTI_PATH=... OSM_PATH=... OUTPUT_PATH=out ./build/tools/kitti_osm_workflow

cmake_minimum_required(VERSION 3.16)
project(hdmap_pipeline_tools LANGUAGES CXX)

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
        ${LIB_SRC}/src/geometry/coordinates/wgs84.cpp
        ${LIB_SRC}/src/types/io/track/kitti_oxts_importer.cpp
        ${LIB_SRC}/src/types/io/lidar/kitti_lidar_reader.cpp
        ${LIB_SRC}/src/geometry/road_graph/osm_importer.cpp
        ${LIB_SRC}/src/geometry/road_graph/map_matcher.cpp)

add_library(hdmap_pipeline_tools_lib STATIC ${PIPELINE_SOURCES})
target_include_directories(hdmap_pipeline_tools_lib PUBLIC "${LIB_SRC}/include")
target_include_directories(hdmap_pipeline_tools_lib SYSTEM PUBLIC "${NANOFLANN_INC}")

# Best-effort git commit hash (falls back to "unknown").
execute_process(
        COMMAND git -C "${REPO_ROOT}" rev-parse --short HEAD
        OUTPUT_VARIABLE GIT_COMMIT
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET)
if (NOT GIT_COMMIT)
    set(GIT_COMMIT "unknown")
endif ()

add_executable(kitti_osm_workflow kitti_osm_workflow.cpp)
target_link_libraries(kitti_osm_workflow PRIVATE hdmap_pipeline_tools_lib)
target_compile_definitions(kitti_osm_workflow PRIVATE
        WORKFLOW_BUILD_TYPE="${CMAKE_BUILD_TYPE}"
        WORKFLOW_GIT_COMMIT="${GIT_COMMIT}")
