// Copyright (C) 2025 Kartik Vadhawana
//
// RANSAC ground-plane extraction for LiDAR frames. Fits a dominant plane
// (ax + by + cz + d = 0) by random 3-point sampling and inlier counting,
// then classifies each point as ground or obstacle. Used both for map-layer
// separation and as an input to frame integrity validation.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.

#ifndef MAP_MATCHING_2_GEOMETRY_LIDAR_GROUND_EXTRACTOR_HPP
#define MAP_MATCHING_2_GEOMETRY_LIDAR_GROUND_EXTRACTOR_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

#include "types/io/lidar/kitti_lidar_reader.hpp"

namespace map_matching_2::geometry::lidar {

    struct GroundParams {
        double distance_threshold_m = 0.2; // inlier band around the plane
        int iterations = 100;              // RANSAC iterations
        std::uint64_t seed = 42;           // deterministic sampling for tests
    };

    // Plain-old-data result: all scalar members, so the type is standard-layout
    // and trivially copyable. This guarantees a single, fixed ABI/offset layout
    // across every translation unit, compiler and standard library — a
    // std::vector<bool> (or any stdlib container) member would make the struct
    // non-standard-layout and its return-by-value ABI stdlib-dependent, which
    // can corrupt scalar reads across a TU/ABI boundary.
    struct GroundResult {
        double a = 0.0, b = 0.0, c = 0.0, d = 0.0; // plane coefficients (normal a,b,c)
        double plane_height = 0.0;                 // signed distance of origin to plane
        double ground_ratio = 0.0;                 // inliers / total
        std::size_t inlier_count = 0;
        std::size_t outlier_count = 0;
        bool success = false;                      // a plane was found
    };

    class ground_extractor {

    public:
        explicit ground_extractor(GroundParams params = {}) : _params(params) {}

        [[nodiscard]] GroundResult extract(const io::lidar::LidarFrame &frame) const;
        [[nodiscard]] GroundResult extract(const std::vector<io::lidar::LidarPoint> &points) const;

    private:
        GroundParams _params;
    };

}

#endif //MAP_MATCHING_2_GEOMETRY_LIDAR_GROUND_EXTRACTOR_HPP
