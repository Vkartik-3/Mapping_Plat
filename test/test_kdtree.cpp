// Copyright (C) 2025 Kartik Vadhawana
#include <gtest/gtest.h>

#include <cmath>

#include "geometry/index/kdtree.hpp"
#include "util/synthetic_data.hpp"

using namespace map_matching_2;

TEST(KDTree, BuildAndSize) {
    auto frame = util::synthetic::lidar_frame(1000, 200);
    geometry::index::KDTree3D tree(frame);
    EXPECT_EQ(tree.size(), 1200u);
    EXPECT_FALSE(tree.empty());
}

TEST(KDTree, NearestNeighborCorrectness) {
    io::lidar::LidarFrame frame{};
    frame.points = {{0, 0, 0, 0.1f}, {10, 0, 0, 0.1f}, {0, 10, 0, 0.1f}, {5, 5, 5, 0.1f}};
    frame.point_count = frame.points.size();
    geometry::index::KDTree3D tree(frame);
    bool found = false;
    auto hit = tree.nearest_neighbor(9.5f, 0.2f, 0.0f, found);
    ASSERT_TRUE(found);
    EXPECT_EQ(hit.index, 1u);            // (10,0,0) is closest
    EXPECT_NEAR(hit.distance, 0.5385f, 1e-3);
}

TEST(KDTree, NearestOfExistingPointIsZero) {
    auto frame = util::synthetic::lidar_frame(500, 100);
    geometry::index::KDTree3D tree(frame);
    const auto &p = frame.points[123];
    bool found = false;
    auto hit = tree.nearest_neighbor(p.x, p.y, p.z, found);
    ASSERT_TRUE(found);
    EXPECT_NEAR(hit.distance, 0.0, 1e-4);
}

TEST(KDTree, KnnReturnsSortedK) {
    auto frame = util::synthetic::lidar_frame(2000, 0);
    geometry::index::KDTree3D tree(frame);
    auto hits = tree.knn_search(0, 0, -1.7f, 10);
    ASSERT_EQ(hits.size(), 10u);
    for (std::size_t i = 1; i < hits.size(); ++i) {
        EXPECT_LE(hits[i - 1].distance, hits[i].distance);
    }
}

TEST(KDTree, RadiusSearchFindsWithinRadius) {
    io::lidar::LidarFrame frame{};
    for (int i = 0; i < 10; ++i) frame.points.push_back({static_cast<float>(i), 0, 0, 0.1f});
    frame.point_count = frame.points.size();
    geometry::index::KDTree3D tree(frame);
    auto hits = tree.radius_search(0, 0, 0, 3.5);
    // points at x=0,1,2,3 are within 3.5 m
    EXPECT_EQ(hits.size(), 4u);
    for (const auto &h : hits) EXPECT_LE(h.distance, 3.5);
}

TEST(KDTree, EmptyCloudEdgeCase) {
    geometry::index::KDTree3D tree;
    EXPECT_TRUE(tree.empty());
    bool found = true;
    auto hit = tree.nearest_neighbor(1, 2, 3, found);
    EXPECT_FALSE(found);
    (void) hit;
    EXPECT_TRUE(tree.knn_search(0, 0, 0, 5).empty());
    EXPECT_TRUE(tree.radius_search(0, 0, 0, 10).empty());
}
