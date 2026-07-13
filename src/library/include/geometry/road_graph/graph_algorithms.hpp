// Copyright (C) 2025 Kartik Vadhawana
//
// Basic graph algorithms over the lightweight RoadGraph: Dijkstra shortest
// path (edge weight = length_meters) and a brute-force nearest-edge spatial
// query. These support routing/benchmarking on top of the GPS+OSM road graph.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.

#ifndef MAP_MATCHING_2_GEOMETRY_ROAD_GRAPH_GRAPH_ALGORITHMS_HPP
#define MAP_MATCHING_2_GEOMETRY_ROAD_GRAPH_GRAPH_ALGORITHMS_HPP

#include <algorithm>
#include <cstddef>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

#include "geometry/road_graph/road_graph.hpp"

namespace map_matching_2::geometry::road_graph {

    struct ShortestPath {
        double cost = std::numeric_limits<double>::infinity();
        std::vector<std::size_t> nodes; // node indices from source to target
        bool reachable = false;
    };

    // Dijkstra over node indices; edge cost is length_meters. Requires
    // graph.build_adjacency() to have been called.
    inline ShortestPath dijkstra(const RoadGraph &graph, std::size_t source, std::size_t target) {
        ShortestPath result{};
        const std::size_t n = graph.nodes.size();
        if (source >= n || target >= n) return result;

        std::vector<double> dist(n, std::numeric_limits<double>::infinity());
        std::vector<std::size_t> prev(n, std::numeric_limits<std::size_t>::max());
        using QItem = std::pair<double, std::size_t>; // (dist, node)
        std::priority_queue<QItem, std::vector<QItem>, std::greater<>> pq;

        dist[source] = 0.0;
        pq.emplace(0.0, source);
        while (!pq.empty()) {
            auto [d, u] = pq.top();
            pq.pop();
            if (d > dist[u]) continue;
            if (u == target) break;
            if (u >= graph.adjacency.size()) continue;
            for (std::size_t ei : graph.adjacency[u]) {
                const auto &e = graph.edges[ei];
                const double nd = d + e.length_meters;
                if (nd < dist[e.to]) {
                    dist[e.to] = nd;
                    prev[e.to] = u;
                    pq.emplace(nd, e.to);
                }
            }
        }

        if (dist[target] == std::numeric_limits<double>::infinity()) return result;
        result.cost = dist[target];
        result.reachable = true;
        for (std::size_t at = target; at != std::numeric_limits<std::size_t>::max(); at = prev[at]) {
            result.nodes.push_back(at);
            if (at == source) break;
        }
        std::reverse(result.nodes.begin(), result.nodes.end());
        return result;
    }

    // Nearest edge (by perpendicular distance) to a query point, brute force.
    inline std::size_t nearest_edge(const RoadGraph &graph, const Point2D &q, double &out_dist) {
        std::size_t best = 0;
        double best_d = std::numeric_limits<double>::infinity();
        for (std::size_t e = 0; e < graph.edges.size(); ++e) {
            const auto &edge = graph.edges[e];
            for (std::size_t k = 0; k + 1 < edge.polyline.size(); ++k) {
                const auto &a = edge.polyline[k];
                const auto &b = edge.polyline[k + 1];
                const double dx = b.e - a.e, dy = b.n - a.n;
                const double len2 = dx * dx + dy * dy;
                double t = 0.0;
                if (len2 > 1e-12) {
                    t = ((q.e - a.e) * dx + (q.n - a.n) * dy) / len2;
                    t = t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
                }
                const Point2D proj{a.e + t * dx, a.n + t * dy};
                const double d = distance(q, proj);
                if (d < best_d) { best_d = d; best = e; }
            }
        }
        out_dist = best_d;
        return best;
    }

}

#endif //MAP_MATCHING_2_GEOMETRY_ROAD_GRAPH_GRAPH_ALGORITHMS_HPP
