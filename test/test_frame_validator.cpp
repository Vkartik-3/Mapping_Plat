// Copyright (C) 2025 Kartik Vadhawana
#include <gtest/gtest.h>

#include <cmath>

#include "geometry/lidar/frame_validator.hpp"
#include "util/synthetic_data.hpp"

using namespace map_matching_2;

TEST(FrameValidator, GoodFramePasses) {
    auto frame = util::synthetic::lidar_frame(100000, 25000);
    geometry::lidar::frame_validator fv;
    auto r = fv.validate(frame, /*expect_crc=*/true);
    EXPECT_TRUE(r.crc_valid);
    EXPECT_TRUE(r.point_count_valid);
    EXPECT_TRUE(r.range_valid);
    EXPECT_TRUE(r.density_valid);
    EXPECT_TRUE(r.intensity_valid);
    EXPECT_TRUE(r.nan_free);
    EXPECT_GT(r.ground_ratio, 0.10);
    EXPECT_TRUE(r.validation_pass);
}

TEST(FrameValidator, CrcFailFrameFails) {
    auto frame = util::synthetic::lidar_frame(100000, 25000);
    frame.crc_valid = false;
    geometry::lidar::frame_validator fv;
    auto r = fv.validate(frame, /*expect_crc=*/true);
    EXPECT_FALSE(r.crc_valid);
    EXPECT_FALSE(r.validation_pass);
}

TEST(FrameValidator, EmptyFrameFails) {
    io::lidar::LidarFrame empty{};
    empty.crc_valid = true;
    geometry::lidar::frame_validator fv;
    auto r = fv.validate(empty, true);
    EXPECT_FALSE(r.point_count_valid);
    EXPECT_FALSE(r.validation_pass);
}

TEST(FrameValidator, NaNFrameFails) {
    auto frame = util::synthetic::lidar_frame(100000, 25000);
    frame.points[42].y = std::nanf("");
    geometry::lidar::frame_validator fv;
    auto r = fv.validate(frame, true);
    EXPECT_FALSE(r.nan_free);
    EXPECT_FALSE(r.validation_pass);
}

TEST(FrameValidator, AllZeroIntensityFails) {
    auto frame = util::synthetic::lidar_frame(100000, 25000);
    for (auto &p : frame.points) p.intensity = 0.0f;
    geometry::lidar::frame_validator fv;
    auto r = fv.validate(frame, true);
    EXPECT_FALSE(r.intensity_valid);
    EXPECT_FALSE(r.validation_pass);
}
