// Copyright (C) 2025 Kartik Vadhawana
#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "geometry/lidar/ground_extractor.hpp"
#include "util/synthetic_data.hpp"

using namespace map_matching_2;

TEST(GroundExtraction, PlanarCloudIsAllGround) {
    io::lidar::LidarFrame flat{};
    std::mt19937 rng(1);
    std::uniform_real_distribution<float> xy(-20.0f, 20.0f);
    for (int i = 0; i < 3000; ++i) flat.points.push_back({xy(rng), xy(rng), 0.0f, 0.5f});
    flat.point_count = flat.points.size();

    geometry::lidar::ground_extractor ge;
    auto g = ge.extract(flat);
    ASSERT_TRUE(g.success);
    EXPECT_NEAR(g.ground_ratio, 1.0, 1e-6);
    // normal should be near vertical (|c| ~ 1)
    EXPECT_NEAR(std::fabs(g.c), 1.0, 1e-3);
}

TEST(GroundExtraction, MixedCloudRecoversGroundFraction) {
    auto frame = util::synthetic::lidar_frame(8000, 2000); // 80% ground
    geometry::lidar::ground_extractor ge;
    auto g = ge.extract(frame);
    ASSERT_TRUE(g.success);
    EXPECT_GT(g.ground_ratio, 0.7);
    EXPECT_LT(g.ground_ratio, 0.9);
    EXPECT_EQ(g.inlier_count + g.outlier_count, frame.point_count);
}

TEST(GroundExtraction, NonPlanarNoiseHasLowGroundRatio) {
    io::lidar::LidarFrame noise{};
    std::mt19937 rng(3);
    std::uniform_real_distribution<float> box(-30.0f, 30.0f);
    for (int i = 0; i < 3000; ++i) noise.points.push_back({box(rng), box(rng), box(rng), 0.5f});
    noise.point_count = noise.points.size();
    geometry::lidar::ground_extractor ge;
    auto g = ge.extract(noise);
    // a uniformly-filled cube has no dominant plane -> small inlier band
    EXPECT_LT(g.ground_ratio, 0.2);
}

TEST(GroundExtraction, TooFewPointsFails) {
    io::lidar::LidarFrame tiny{};
    tiny.points = {{0, 0, 0, 0.1f}, {1, 0, 0, 0.1f}};
    tiny.point_count = 2;
    geometry::lidar::ground_extractor ge;
    auto g = ge.extract(tiny);
    EXPECT_FALSE(g.success);
}
