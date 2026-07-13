// Copyright (C) 2025 Kartik Vadhawana
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.

#include "geometry/centerline/centerline_extractor.hpp"

#include <algorithm>
#include <cmath>

namespace map_matching_2::geometry::centerline {

    double centerline_extractor::heading_of(double de, double dn) {
        double deg = std::atan2(de, dn) * 180.0 / M_PI; // 0 = north, clockwise
        if (deg < 0.0) deg += 360.0;
        return deg;
    }

    CenterlineNetwork centerline_extractor::extract(
            const std::vector<io::track::KittiOxtsFrame> &frames) const {
        CenterlineNetwork net{};
        if (frames.size() < 2) {
            return net;
        }

        // Step 1-2: per trajectory point, take ENU position and heading from
        // the direction of travel to the next point.
        struct Sample {
            road_graph::Point2D pos;
            double heading;
            int bin;
        };
        std::vector<Sample> samples;
        samples.reserve(frames.size());
        const int bin_count = static_cast<int>(std::lround(360.0 / _params.heading_bin_deg));
        for (std::size_t i = 0; i + 1 < frames.size(); ++i) {
            const double de = frames[i + 1].easting - frames[i].easting;
            const double dn = frames[i + 1].northing - frames[i].northing;
            if (std::fabs(de) < 1e-9 && std::fabs(dn) < 1e-9) {
                continue; // stationary, no reliable heading
            }
            const double h = heading_of(de, dn);
            int bin = static_cast<int>(h / _params.heading_bin_deg) % bin_count;
            samples.push_back({{frames[i].easting, frames[i].northing}, h, bin});
        }

        // Step 3: within each heading bin, greedily cluster by proximity.
        struct Cluster {
            road_graph::Point2D sum{};
            double heading_sum = 0.0;
            std::size_t support = 0;
            int bin = 0;
            road_graph::Point2D centroid() const {
                return {sum.e / support, sum.n / support};
            }
        };
        std::vector<Cluster> clusters;
        for (const auto &s : samples) {
            Cluster *best = nullptr;
            double best_d = _params.cluster_radius_m;
            for (auto &c : clusters) {
                if (c.bin != s.bin) continue;
                const double d = road_graph::distance(c.centroid(), s.pos);
                if (d <= best_d) {
                    best_d = d;
                    best = &c;
                }
            }
            if (best) {
                best->sum.e += s.pos.e;
                best->sum.n += s.pos.n;
                best->heading_sum += s.heading;
                ++best->support;
            } else {
                Cluster c{};
                c.sum = s.pos;
                c.heading_sum = s.heading;
                c.support = 1;
                c.bin = s.bin;
                clusters.push_back(c);
            }
        }

        // Step 4 output: cluster centroids become centerline points / nodes.
        net.points.reserve(clusters.size());
        net.graph.nodes.reserve(clusters.size());
        for (std::size_t i = 0; i < clusters.size(); ++i) {
            const auto &c = clusters[i];
            CenterlinePoint cp{};
            cp.pos = c.centroid();
            cp.heading_deg = c.heading_sum / c.support;
            cp.heading_bin = c.bin;
            cp.support = c.support;
            net.points.push_back(cp);

            road_graph::RoadNode node{};
            node.id = i;
            node.pos = cp.pos;
            net.graph.nodes.push_back(node);
        }

        // Step 6: mark intersections where clusters from different heading
        // bins fall within intersection_dist_m of each other.
        for (std::size_t i = 0; i < net.points.size(); ++i) {
            for (std::size_t j = i + 1; j < net.points.size(); ++j) {
                if (net.points[i].heading_bin == net.points[j].heading_bin) continue;
                if (road_graph::distance(net.points[i].pos, net.points[j].pos)
                        <= _params.intersection_dist_m) {
                    net.points[i].is_intersection = true;
                    net.points[j].is_intersection = true;
                }
            }
        }

        // Step 5: connect centroids within the same heading bin into ordered
        // segments. Order along the bin's heading direction, then chain.
        std::uint64_t edge_id = 0;
        for (int bin = 0; bin < bin_count; ++bin) {
            std::vector<std::size_t> members;
            for (std::size_t i = 0; i < net.points.size(); ++i) {
                if (net.points[i].heading_bin == bin) members.push_back(i);
            }
            if (members.size() < 2) continue;

            const double hd = bin * _params.heading_bin_deg * M_PI / 180.0;
            const double ux = std::sin(hd); // east component of heading unit vec
            const double uy = std::cos(hd); // north component
            std::sort(members.begin(), members.end(), [&](std::size_t a, std::size_t b) {
                const auto &pa = net.points[a].pos;
                const auto &pb = net.points[b].pos;
                return (pa.e * ux + pa.n * uy) < (pb.e * ux + pb.n * uy);
            });

            for (std::size_t k = 0; k + 1 < members.size(); ++k) {
                const std::size_t a = members[k];
                const std::size_t b = members[k + 1];
                road_graph::RoadEdge edge{};
                edge.id = edge_id++;
                edge.from = a;
                edge.to = b;
                edge.polyline = {net.points[a].pos, net.points[b].pos};
                edge.length_meters = road_graph::distance(net.points[a].pos, net.points[b].pos);
                edge.speed_limit = 0.0;
                edge.road_class = "centerline";
                net.graph.edges.push_back(edge);
            }
        }

        net.graph.build_adjacency();
        net.intersection_count = static_cast<std::size_t>(
                std::count_if(net.points.begin(), net.points.end(),
                        [](const CenterlinePoint &p) { return p.is_intersection; }));
        return net;
    }

}
