// Copyright (C) 2025 Kartik Vadhawana
//
// nanoflann-backed 3D k-d tree for LiDAR point-cloud nearest-neighbour,
// radius and kNN queries. Complements the Boost.Geometry R*-tree (which the
// upstream project uses for the 2D road network) with a cache-friendly
// static index tuned for dense 3D point clouds.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.

#ifndef MAP_MATCHING_2_GEOMETRY_INDEX_KDTREE_HPP
#define MAP_MATCHING_2_GEOMETRY_INDEX_KDTREE_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <memory>
#include <vector>

#include "nanoflann.hpp"

#include "types/io/lidar/kitti_lidar_reader.hpp"

namespace map_matching_2::geometry::index {

    struct NeighborHit {
        std::size_t index = 0;
        double distance = 0.0; // Euclidean metres (not squared)
    };

    // KDTree3D indexes the x/y/z of a LidarFrame's points. It owns a copy of
    // the coordinates so it stays valid independently of the source frame.
    class KDTree3D {

        // nanoflann dataset adaptor over the internal coordinate array.
        struct Cloud {
            std::vector<std::array<float, 3>> pts;

            [[nodiscard]] std::size_t kdtree_get_point_count() const { return pts.size(); }

            [[nodiscard]] float kdtree_get_pt(std::size_t idx, std::size_t dim) const {
                return pts[idx][dim];
            }

            template<class BBOX>
            bool kdtree_get_bbox(BBOX &) const { return false; }
        };

        using tree_t = nanoflann::KDTreeSingleIndexAdaptor<
                nanoflann::L2_Simple_Adaptor<float, Cloud>, Cloud, 3, std::size_t>;

    public:
        KDTree3D() = default;

        explicit KDTree3D(const io::lidar::LidarFrame &frame) { build(frame); }

        // (Re)build the index from a LiDAR frame's points.
        void build(const io::lidar::LidarFrame &frame) {
            _cloud.pts.clear();
            _cloud.pts.reserve(frame.points.size());
            for (const auto &p : frame.points) {
                _cloud.pts.push_back({p.x, p.y, p.z});
            }
            rebuild_index();
        }

        // Build directly from raw points.
        void build(const std::vector<io::lidar::LidarPoint> &points) {
            _cloud.pts.clear();
            _cloud.pts.reserve(points.size());
            for (const auto &p : points) _cloud.pts.push_back({p.x, p.y, p.z});
            rebuild_index();
        }

        [[nodiscard]] std::size_t size() const noexcept { return _cloud.pts.size(); }
        [[nodiscard]] bool empty() const noexcept { return _cloud.pts.empty(); }

        // Single nearest neighbour. Returns {index=0, distance=inf-ish} sentinel
        // via `found=false` when the tree is empty.
        NeighborHit nearest_neighbor(float x, float y, float z, bool &found) const {
            found = false;
            NeighborHit hit{};
            if (!_tree || empty()) return hit;
            const float q[3] = {x, y, z};
            std::size_t idx = 0;
            float dist2 = 0.0f;
            nanoflann::KNNResultSet<float, std::size_t> rs(1);
            rs.init(&idx, &dist2);
            _tree->findNeighbors(rs, q);
            hit.index = idx;
            hit.distance = std::sqrt(static_cast<double>(dist2));
            found = true;
            return hit;
        }

        // k nearest neighbours, sorted by ascending distance.
        std::vector<NeighborHit> knn_search(float x, float y, float z, std::size_t k) const {
            std::vector<NeighborHit> out;
            if (!_tree || empty() || k == 0) return out;
            k = std::min(k, size());
            const float q[3] = {x, y, z};
            std::vector<std::size_t> idx(k);
            std::vector<float> dist2(k);
            nanoflann::KNNResultSet<float, std::size_t> rs(k);
            rs.init(idx.data(), dist2.data());
            _tree->findNeighbors(rs, q);
            out.reserve(rs.size());
            for (std::size_t i = 0; i < rs.size(); ++i) {
                out.push_back({idx[i], std::sqrt(static_cast<double>(dist2[i]))});
            }
            return out;
        }

        // All neighbours within `radius` metres, sorted by ascending distance.
        std::vector<NeighborHit> radius_search(float x, float y, float z, double radius) const {
            std::vector<NeighborHit> out;
            if (!_tree || empty() || radius <= 0.0) return out;
            const float q[3] = {x, y, z};
            const float radius2 = static_cast<float>(radius * radius); // L2 uses squared
            std::vector<nanoflann::ResultItem<std::size_t, float>> matches;
            nanoflann::SearchParameters params;
            params.sorted = true;
            (void) _tree->radiusSearch(q, radius2, matches, params);
            out.reserve(matches.size());
            for (const auto &m : matches) {
                out.push_back({m.first, std::sqrt(static_cast<double>(m.second))});
            }
            return out;
        }

    private:
        void rebuild_index() {
            _tree = std::make_unique<tree_t>(3, _cloud,
                    nanoflann::KDTreeSingleIndexAdaptorParams(10));
            if (!_cloud.pts.empty()) _tree->buildIndex();
        }

        Cloud _cloud{};
        std::unique_ptr<tree_t> _tree;
    };

}

#endif //MAP_MATCHING_2_GEOMETRY_INDEX_KDTREE_HPP
