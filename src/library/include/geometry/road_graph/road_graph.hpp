// Copyright (C) 2025 Kartik Vadhawana
//
// Lightweight road-network graph model shared by the OSM importer and the
// map matcher. Coordinates are stored in a local ENU frame (metres) so GPS
// traces, OSM ways and LiDAR all live in one comparable space.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.

#ifndef MAP_MATCHING_2_GEOMETRY_ROAD_GRAPH_HPP
#define MAP_MATCHING_2_GEOMETRY_ROAD_GRAPH_HPP

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace map_matching_2::geometry::road_graph {

    struct Point2D {
        double e = 0.0; // easting  (metres, ENU)
        double n = 0.0; // northing (metres, ENU)
    };

    inline double distance(const Point2D &a, const Point2D &b) {
        const double de = a.e - b.e;
        const double dn = a.n - b.n;
        return std::sqrt(de * de + dn * dn);
    }

    struct RoadNode {
        std::uint64_t id = 0;
        Point2D pos{};
        double lat = 0.0, lon = 0.0;
    };

    struct RoadEdge {
        std::uint64_t id = 0;
        std::size_t from = 0;               // index into RoadGraph::nodes
        std::size_t to = 0;
        double length_meters = 0.0;
        double speed_limit = 0.0;           // km/h
        std::string road_class;             // highway=* value
        std::vector<Point2D> polyline;      // ordered geometry, ENU
    };

    class RoadGraph {

    public:
        std::vector<RoadNode> nodes;
        std::vector<RoadEdge> edges;
        // adjacency[i] = indices of edges whose `from` == node i.
        std::vector<std::vector<std::size_t>> adjacency;

        void build_adjacency() {
            adjacency.assign(nodes.size(), {});
            for (std::size_t i = 0; i < edges.size(); ++i) {
                if (edges[i].from < adjacency.size()) {
                    adjacency[edges[i].from].push_back(i);
                }
            }
        }

        [[nodiscard]] std::size_t node_count() const noexcept { return nodes.size(); }
        [[nodiscard]] std::size_t edge_count() const noexcept { return edges.size(); }

        [[nodiscard]] double total_length_meters() const noexcept {
            double total = 0.0;
            for (const auto &e : edges) total += e.length_meters;
            return total;
        }
    };

}

#endif //MAP_MATCHING_2_GEOMETRY_ROAD_GRAPH_HPP
