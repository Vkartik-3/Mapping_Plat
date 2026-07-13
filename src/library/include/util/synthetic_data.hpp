// Copyright (C) 2025 Kartik Vadhawana
//
// Synthetic sensor-data generators shared by the benchmark and test suites,
// so both can exercise the pipeline deterministically without the real KITTI
// dataset. Header-only; not compiled into production builds.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.

#ifndef MAP_MATCHING_2_UTIL_SYNTHETIC_DATA_HPP
#define MAP_MATCHING_2_UTIL_SYNTHETIC_DATA_HPP

#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

#include "types/io/track/kitti_oxts_importer.hpp"
#include "types/io/lidar/kitti_lidar_reader.hpp"
#include "geometry/road_graph/road_graph.hpp"

namespace map_matching_2::util::synthetic {

    // L-shaped GPS trajectory near Karlsruhe: `east_steps` frames heading east
    // then `north_steps` frames heading north, `step_m` metres apart. ENU
    // fields are filled anchored to the first frame.
    inline std::vector<io::track::KittiOxtsFrame> trajectory(
            int east_steps = 40, int north_steps = 40, double step_m = 5.0) {
        std::vector<io::track::KittiOxtsFrame> f;
        const double lat0 = 49.0, lon0 = 8.4, R = 6371000.0, d2r = M_PI / 180.0;
        const double m_per_lon = R * std::cos(lat0 * d2r) * d2r;
        const double m_per_lat = R * d2r;
        for (int i = 0; i < east_steps; ++i) {
            io::track::KittiOxtsFrame k{};
            k.lat = lat0;
            k.lon = lon0 + (i * step_m) / m_per_lon;
            k.alt = 116.0; k.navstat = 0; k.numsats = 8;
            f.push_back(k);
        }
        const double east_fixed = (east_steps - 1) * step_m;
        for (int i = 1; i <= north_steps; ++i) {
            io::track::KittiOxtsFrame k{};
            k.lat = lat0 + (i * step_m) / m_per_lat;
            k.lon = lon0 + east_fixed / m_per_lon;
            k.alt = 116.0; k.navstat = 0; k.numsats = 8;
            f.push_back(k);
        }
        if (!f.empty()) {
            const double alat = f[0].lat, alon = f[0].lon, aalt = f[0].alt;
            for (auto &k : f) {
                io::track::kitti_oxts_importer::to_enu(alat, alon, aalt,
                        k.lat, k.lon, k.alt, k.easting, k.northing, k.up);
            }
        }
        return f;
    }

    // A LiDAR frame: `ground` points on a plane at z=`ground_z`, plus
    // `obstacle` points at random heights, full azimuth coverage.
    inline io::lidar::LidarFrame lidar_frame(
            std::size_t ground = 100000, std::size_t obstacle = 25000,
            float ground_z = -1.7f, std::uint64_t idx = 0, std::uint64_t seed = 7) {
        io::lidar::LidarFrame frame{};
        frame.frame_index = idx;
        std::mt19937 rng(static_cast<unsigned>(seed));
        std::uniform_real_distribution<float> rad(3.0f, 60.0f);
        std::uniform_real_distribution<float> ang(0.0f, 2.0f * static_cast<float>(M_PI));
        std::uniform_real_distribution<float> inten(0.05f, 0.95f);
        std::uniform_real_distribution<float> zjit(-0.03f, 0.03f);
        std::uniform_real_distribution<float> zh(-1.0f, 3.0f);
        frame.points.reserve(ground + obstacle);
        for (std::size_t i = 0; i < ground; ++i) {
            float r = rad(rng), a = ang(rng);
            frame.points.push_back({r * std::cos(a), r * std::sin(a), ground_z + zjit(rng), inten(rng)});
        }
        for (std::size_t i = 0; i < obstacle; ++i) {
            float r = rad(rng), a = ang(rng);
            frame.points.push_back({r * std::cos(a), r * std::sin(a), zh(rng), inten(rng)});
        }
        frame.point_count = frame.points.size();
        frame.crc_valid = true;
        return frame;
    }

    // The L-shaped road graph matching trajectory(): two connected ways.
    inline geometry::road_graph::RoadGraph road_graph(double step_m = 5.0,
            int east_steps = 40, int north_steps = 40) {
        using namespace geometry::road_graph;
        RoadGraph g;
        const double east_len = (east_steps - 1) * step_m;
        const double north_len = north_steps * step_m;
        // 3 nodes: origin, corner, top.
        g.nodes.push_back({1, {0.0, 0.0}, 49.0, 8.4});
        g.nodes.push_back({2, {east_len, 0.0}, 49.0, 8.4});
        g.nodes.push_back({3, {east_len, north_len}, 49.0, 8.4});
        RoadEdge e0{}; e0.id = 0; e0.from = 0; e0.to = 1; e0.road_class = "residential";
        e0.speed_limit = 50; e0.polyline = {g.nodes[0].pos, g.nodes[1].pos};
        e0.length_meters = distance(g.nodes[0].pos, g.nodes[1].pos);
        RoadEdge e1{}; e1.id = 1; e1.from = 1; e1.to = 2; e1.road_class = "secondary";
        e1.speed_limit = 60; e1.polyline = {g.nodes[1].pos, g.nodes[2].pos};
        e1.length_meters = distance(g.nodes[1].pos, g.nodes[2].pos);
        g.edges.push_back(e0);
        g.edges.push_back(e1);
        g.build_adjacency();
        return g;
    }

    // A larger grid road graph (rows x cols nodes) for routing benchmarks.
    inline geometry::road_graph::RoadGraph grid_graph(int rows, int cols, double spacing = 20.0) {
        using namespace geometry::road_graph;
        RoadGraph g;
        auto nid = [cols](int r, int c) { return static_cast<std::size_t>(r * cols + c); };
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                RoadNode n{};
                n.id = nid(r, c);
                n.pos = {c * spacing, r * spacing};
                g.nodes.push_back(n);
            }
        }
        std::uint64_t eid = 0;
        auto add_edge = [&](std::size_t a, std::size_t b) {
            RoadEdge e{}; e.id = eid++; e.from = a; e.to = b; e.road_class = "residential";
            e.speed_limit = 50; e.polyline = {g.nodes[a].pos, g.nodes[b].pos};
            e.length_meters = distance(g.nodes[a].pos, g.nodes[b].pos);
            g.edges.push_back(e);
        };
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                if (c + 1 < cols) { add_edge(nid(r, c), nid(r, c + 1)); add_edge(nid(r, c + 1), nid(r, c)); }
                if (r + 1 < rows) { add_edge(nid(r, c), nid(r + 1, c)); add_edge(nid(r + 1, c), nid(r, c)); }
            }
        }
        g.build_adjacency();
        return g;
    }

}

#endif //MAP_MATCHING_2_UTIL_SYNTHETIC_DATA_HPP
