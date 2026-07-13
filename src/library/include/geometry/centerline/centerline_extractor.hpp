// Copyright (C) 2025 Kartik Vadhawana
//
// Road centerline extraction from a GPS trajectory, following the trace-
// clustering idea of Schroedl et al. / Biagioni & Eriksson: bin trajectory
// points by heading, cluster spatially within each heading bin, connect the
// cluster centroids into ordered centerline segments, and mark intersections
// where differently-headed clusters meet.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.

#ifndef MAP_MATCHING_2_GEOMETRY_CENTERLINE_EXTRACTOR_HPP
#define MAP_MATCHING_2_GEOMETRY_CENTERLINE_EXTRACTOR_HPP

#include <cstdint>
#include <vector>

#include "geometry/road_graph/road_graph.hpp"
#include "types/io/track/kitti_oxts_importer.hpp"

namespace map_matching_2::geometry::centerline {

    struct CenterlineParams {
        double heading_bin_deg = 10.0;     // heading bucket width
        double cluster_radius_m = 5.0;     // spatial merge radius within a bin
        double intersection_dist_m = 10.0; // cross-heading proximity => intersection
    };

    // A clustered centerline point (one cluster centroid).
    struct CenterlinePoint {
        road_graph::Point2D pos{};
        double heading_deg = 0.0;
        int heading_bin = 0;
        bool is_intersection = false;
        std::size_t support = 0; // number of GPS points merged
    };

    struct CenterlineNetwork {
        road_graph::RoadGraph graph;       // nodes = centroids/intersections, edges = segments
        std::vector<CenterlinePoint> points;
        std::size_t intersection_count = 0;
    };

    class centerline_extractor {

    public:
        explicit centerline_extractor(CenterlineParams params = {}) : _params(params) {}

        // Build a centerline network from parsed OXTS frames (uses their ENU
        // easting/northing and derives heading from successive motion).
        [[nodiscard]] CenterlineNetwork extract(
                const std::vector<io::track::KittiOxtsFrame> &frames) const;

        // Convenience: heading of the vector (de, dn) in [0, 360).
        static double heading_of(double de, double dn);

    private:
        CenterlineParams _params;
    };

}

#endif //MAP_MATCHING_2_GEOMETRY_CENTERLINE_EXTRACTOR_HPP
