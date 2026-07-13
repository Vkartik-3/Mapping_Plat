// Copyright (C) 2025 Kartik Vadhawana
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.

#include "geometry/lidar/ground_extractor.hpp"

#include <cmath>
#include <random>

namespace map_matching_2::geometry::lidar {

    GroundResult ground_extractor::extract(const io::lidar::LidarFrame &frame) const {
        return extract(frame.points);
    }

    GroundResult ground_extractor::extract(
            const std::vector<io::lidar::LidarPoint> &points) const {
        GroundResult best{};
        if (points.size() < 3) {
            return best;
        }

        std::mt19937_64 rng(_params.seed);
        std::uniform_int_distribution<std::size_t> pick(0, points.size() - 1);

        const double thr = _params.distance_threshold_m;
        std::size_t best_inliers = 0;

        for (int iter = 0; iter < _params.iterations; ++iter) {
            // Sample 3 distinct points.
            std::size_t i0 = pick(rng), i1 = pick(rng), i2 = pick(rng);
            if (i0 == i1 || i1 == i2 || i0 == i2) continue;

            const auto &p0 = points[i0];
            const auto &p1 = points[i1];
            const auto &p2 = points[i2];

            // Plane normal via cross product of two edge vectors.
            const double ux = p1.x - p0.x, uy = p1.y - p0.y, uz = p1.z - p0.z;
            const double vx = p2.x - p0.x, vy = p2.y - p0.y, vz = p2.z - p0.z;
            double nx = uy * vz - uz * vy;
            double ny = uz * vx - ux * vz;
            double nz = ux * vy - uy * vx;
            const double norm = std::sqrt(nx * nx + ny * ny + nz * nz);
            if (norm < 1e-9) continue; // degenerate / collinear sample
            nx /= norm; ny /= norm; nz /= norm;
            const double d = -(nx * p0.x + ny * p0.y + nz * p0.z);

            // Count inliers.
            std::size_t inliers = 0;
            for (const auto &p : points) {
                const double dist = std::fabs(nx * p.x + ny * p.y + nz * p.z + d);
                if (dist <= thr) ++inliers;
            }

            if (inliers > best_inliers) {
                best_inliers = inliers;
                best.a = nx; best.b = ny; best.c = nz; best.d = d;
                best.success = true;
            }
        }

        if (!best.success) {
            return best;
        }

        // Final classification against the best plane.
        best.classification.assign(points.size(), false);
        std::size_t inliers = 0;
        for (std::size_t i = 0; i < points.size(); ++i) {
            const auto &p = points[i];
            const double dist = std::fabs(best.a * p.x + best.b * p.y + best.c * p.z + best.d);
            const bool ground = dist <= thr;
            best.classification[i] = ground;
            if (ground) ++inliers;
        }
        best.inlier_count = inliers;
        best.outlier_count = points.size() - inliers;
        best.ground_ratio = static_cast<double>(inliers) / static_cast<double>(points.size());
        // Signed distance from sensor origin (0,0,0) to the plane is just d
        // (normal is unit length).
        best.plane_height = best.d;
        return best;
    }

}
