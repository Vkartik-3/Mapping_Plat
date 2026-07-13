# GoogleTest suite for the HD map pipeline extensions (Phases 1-3).
# Self-contained: compiles only the pipeline's own sources plus the bundled
# googletest submodule, so it builds without the heavy Boost/Osmium stack.
#
# Standalone use:  cmake -S test -B build/test && cmake --build build/test && ctest --test-dir build/test

cmake_minimum_required(VERSION 3.16)
project(hdmap_pipeline_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

get_filename_component(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
set(LIB_SRC "${REPO_ROOT}/src/library")
set(NANOFLANN_INC "${REPO_ROOT}/third_party/nanoflann/include")

# Pipeline sources under test.
set(PIPELINE_SOURCES
        ${LIB_SRC}/src/types/io/track/kitti_oxts_importer.cpp
        ${LIB_SRC}/src/types/io/lidar/kitti_lidar_reader.cpp
        ${LIB_SRC}/src/geometry/centerline/centerline_extractor.cpp
        ${LIB_SRC}/src/geometry/road_graph/osm_importer.cpp
        ${LIB_SRC}/src/geometry/road_graph/map_matcher.cpp
        ${LIB_SRC}/src/geometry/lidar/ground_extractor.cpp)

add_library(hdmap_pipeline STATIC ${PIPELINE_SOURCES})
target_include_directories(hdmap_pipeline PUBLIC "${LIB_SRC}/include")
target_include_directories(hdmap_pipeline SYSTEM PUBLIC "${NANOFLANN_INC}")

# googletest from the submodule.
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
add_subdirectory("${REPO_ROOT}/third_party/googletest" googletest-build EXCLUDE_FROM_ALL)

enable_testing()

add_executable(hdmap_tests
        test_crc32.cpp
        test_kitti_reader.cpp
        test_kdtree.cpp
        test_ground_extraction.cpp
        test_frame_validator.cpp
        test_centerline.cpp
        test_map_matching.cpp)

target_link_libraries(hdmap_tests PRIVATE hdmap_pipeline GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(hdmap_tests)
