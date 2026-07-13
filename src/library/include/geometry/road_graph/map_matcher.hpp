// Copyright (C) 2025 Kartik Vadhawana
//
// HMM map matching (Newson & Krumm, 2009) that snaps a GPS trajectory onto a
// RoadGraph. Emission probabilities come from perpendicular GPS-to-edge
// distances; transition probabilities from the agreement between measured GPS
// step length and on-road travel distance. A Viterbi pass recovers the most
// likely edge sequence.
//
// This is a self-contained matcher used by the HD map pipeline; the upstream
// map-matching-2 MDP/HMM matcher remains available for large-scale runs.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.

#ifndef MAP_MATCHING_2_GEOMETRY_ROAD_GRAPH_MAP_MATCHER_HPP
#define MAP_MATCHING_2_GEOMETRY_ROAD_GRAPH_MAP_MATCHER_HPP

#include <cstddef>
#include <limits>
#include <vector>

#include "geometry/road_graph/road_graph.hpp"
#include "types/io/track/kitti_oxts_importer.hpp"

namespace map_matching_2::geometry::road_graph {

    struct MatchParams {
        double search_radius_m = 50.0; // candidate edges within this distance
        double sigma_z_m = 4.07;       // GPS noise std (Newson-Krumm default)
        double beta_m = 3.0;           // transition decay scale
        std::size_t max_candidates = 8;
    };

    struct MatchedPoint {
        bool matched = false;
        std::size_t edge_index = 0;      // index into RoadGraph::edges
        Point2D snapped{};               // closest point on the matched edge
        double gps_distance_m = 0.0;     // perpendicular distance to that edge
        double confidence = 0.0;         // normalized emission likelihood [0,1]
    };

    struct MatchResult {
        std::vector<MatchedPoint> points;
        std::size_t total_points = 0;
        std::size_t matched_points = 0;
        std::size_t unmatched_points = 0;
        double matching_rate = 0.0;      // matched / total
        double mean_confidence = 0.0;
    };

    class map_matcher {

    public:
        explicit map_matcher(MatchParams params = {}) : _params(params) {}

        // Match a GPS trajectory (ENU easting/northing of OXTS frames) to the
        // graph. The graph must be in the same ENU frame as the frames.
        [[nodiscard]] MatchResult match(const RoadGraph &graph,
                const std::vector<io::track::KittiOxtsFrame> &frames) const;

        // Closest point on segment [a,b] to p, plus the distance to it.
        static Point2D project_point_to_segment(
                const Point2D &p, const Point2D &a, const Point2D &b, double &out_dist);

    private:
        MatchParams _params;
    };

}

#endif //MAP_MATCHING_2_GEOMETRY_ROAD_GRAPH_MAP_MATCHER_HPP
