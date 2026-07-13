// Copyright (C) 2025 Kartik Vadhawana
//
// Structured integrity validation for a LiDAR frame: CRC32, point-count
// bounds, NaN/Inf screening, range validity, angular coverage (density),
// intensity distribution, and ground-extraction success. Produces a per-frame
// report used by the ingestion pipeline and the benchmark/test suites.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.

#ifndef MAP_MATCHING_2_GEOMETRY_LIDAR_FRAME_VALIDATOR_HPP
#define MAP_MATCHING_2_GEOMETRY_LIDAR_FRAME_VALIDATOR_HPP

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_set>

#include "types/io/lidar/kitti_lidar_reader.hpp"
#include "geometry/lidar/ground_extractor.hpp"

namespace map_matching_2::geometry::lidar {

    struct ValidationParams {
        std::size_t min_points = 10000;
        std::size_t max_points = 150000;
        double min_range_m = 0.5;
        double max_range_m = 200.0;
        double min_coverage = 0.30;      // fraction of azimuth sectors hit
        double min_ground_ratio = 0.10;
        std::size_t azimuth_sectors = 360; // 1-degree bins for coverage
    };

    struct FrameValidation {
        std::uint64_t frame_index = 0;
        bool crc_valid = false;
        bool point_count_valid = false;
        bool range_valid = false;
        bool density_valid = false;
        bool intensity_valid = false;
        bool nan_free = false;
        double ground_ratio = 0.0;
        double coverage = 0.0;
        bool validation_pass = false;
    };

    class frame_validator {

    public:
        explicit frame_validator(ValidationParams params = {}) : _params(params) {}

        // `expect_crc` toggles whether a false frame.crc_valid should fail the
        // frame (a sidecar was expected). When false, CRC is treated as pass.
        [[nodiscard]] FrameValidation validate(
                const io::lidar::LidarFrame &frame, bool expect_crc = true) const {
            FrameValidation r{};
            r.frame_index = frame.frame_index;

            r.crc_valid = expect_crc ? frame.crc_valid : true;

            r.point_count_valid = frame.point_count >= _params.min_points &&
                    frame.point_count <= _params.max_points;

            // Single pass: NaN/Inf, range band, azimuth coverage, intensity spread.
            std::unordered_set<int> sectors;
            bool nan_free = true;
            bool range_ok = !frame.points.empty();
            bool all_zero_i = true, all_one_i = true;
            const double d2sector = static_cast<double>(_params.azimuth_sectors) / 360.0;

            for (const auto &p : frame.points) {
                if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
                        !std::isfinite(p.z) || !std::isfinite(p.intensity)) {
                    nan_free = false;
                    continue;
                }
                const double range = std::sqrt(
                        static_cast<double>(p.x) * p.x +
                        static_cast<double>(p.y) * p.y +
                        static_cast<double>(p.z) * p.z);
                if (range < _params.min_range_m || range > _params.max_range_m) {
                    range_ok = false;
                }
                double az = std::atan2(p.y, p.x) * 180.0 / M_PI; // [-180,180)
                if (az < 0.0) az += 360.0;
                sectors.insert(static_cast<int>(az * d2sector));

                if (p.intensity != 0.0f) all_zero_i = false;
                if (p.intensity != 1.0f) all_one_i = false;
            }

            r.nan_free = nan_free;
            r.range_valid = range_ok && nan_free;
            r.coverage = _params.azimuth_sectors == 0
                    ? 0.0
                    : static_cast<double>(sectors.size()) / _params.azimuth_sectors;
            r.density_valid = r.coverage >= _params.min_coverage;
            r.intensity_valid = !frame.points.empty() && !all_zero_i && !all_one_i;

            ground_extractor ge;
            const auto ground = ge.extract(frame);
            r.ground_ratio = ground.ground_ratio;

            r.validation_pass = r.crc_valid && r.point_count_valid && r.range_valid &&
                    r.density_valid && r.intensity_valid && r.nan_free &&
                    r.ground_ratio > _params.min_ground_ratio;
            return r;
        }

    private:
        ValidationParams _params;
    };

}

#endif //MAP_MATCHING_2_GEOMETRY_LIDAR_FRAME_VALIDATOR_HPP
