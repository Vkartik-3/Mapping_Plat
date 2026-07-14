// Copyright (C) 2025 Kartik Vadhawana
//
// Quality/performance metrics and JSON report for the KITTI-GPS -> OSM-graph
// map-matching workflow. Header-only so both the workflow executable and its
// unit tests share the exact same metric and serialization code.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.

#ifndef MAP_MATCHING_2_WORKFLOW_PIPELINE_REPORT_HPP
#define MAP_MATCHING_2_WORKFLOW_PIPELINE_REPORT_HPP

#include <algorithm>
#include <cstddef>
#include <sstream>
#include <string>
#include <vector>

#include "geometry/road_graph/road_graph.hpp"
#include "geometry/road_graph/map_matcher.hpp"

namespace map_matching_2::workflow {

    // Linear-interpolated percentile (p in [0, 100]) over a distance sample.
    // Returns 0 for an empty sample. Does not mutate the input.
    inline double percentile(std::vector<double> values, double p) {
        if (values.empty()) return 0.0;
        std::sort(values.begin(), values.end());
        if (values.size() == 1) return values.front();
        const double rank = (p / 100.0) * static_cast<double>(values.size() - 1);
        const std::size_t lo = static_cast<std::size_t>(rank);
        const double frac = rank - static_cast<double>(lo);
        if (lo + 1 >= values.size()) return values.back();
        return values[lo] + frac * (values[lo + 1] - values[lo]);
    }

    struct SnapStats {
        std::size_t count = 0;
        double mean = 0.0;
        double p50 = 0.0;
        double p95 = 0.0;
        double max_v = 0.0;
    };

    inline SnapStats snap_stats(const std::vector<double> &dists) {
        SnapStats s{};
        s.count = dists.size();
        if (dists.empty()) return s;
        double sum = 0.0, mx = 0.0;
        for (double d : dists) { sum += d; mx = std::max(mx, d); }
        s.mean = sum / static_cast<double>(dists.size());
        s.p50 = percentile(dists, 50.0);
        s.p95 = percentile(dists, 95.0);
        s.max_v = mx;
        return s;
    }

    // Do two edges share a graph node (are they topologically adjacent)?
    inline bool edges_share_node(const geometry::road_graph::RoadGraph &g,
            std::size_t ei, std::size_t ej) {
        if (ei >= g.edges.size() || ej >= g.edges.size()) return false;
        const auto &a = g.edges[ei];
        const auto &b = g.edges[ej];
        return a.from == b.from || a.from == b.to || a.to == b.from || a.to == b.to;
    }

    // Count transitions where consecutive matched observations jump to a
    // different edge that is not topologically connected to the previous one.
    inline std::size_t count_disconnected(const geometry::road_graph::RoadGraph &g,
            const geometry::road_graph::MatchResult &m) {
        std::size_t disconnected = 0;
        bool have_prev = false;
        std::size_t prev_edge = 0;
        for (const auto &p : m.points) {
            if (!p.matched) { have_prev = false; continue; }
            if (have_prev && p.edge_index != prev_edge &&
                    !edges_share_node(g, prev_edge, p.edge_index)) {
                ++disconnected;
            }
            prev_edge = p.edge_index;
            have_prev = true;
        }
        return disconnected;
    }

    inline std::size_t count_low_confidence(
            const geometry::road_graph::MatchResult &m, double threshold) {
        std::size_t n = 0;
        for (const auto &p : m.points) {
            if (p.matched && p.confidence < threshold) ++n;
        }
        return n;
    }

    // Snap distances for the matched points only.
    inline std::vector<double> matched_snap_distances(
            const geometry::road_graph::MatchResult &m) {
        std::vector<double> d;
        d.reserve(m.matched_points);
        for (const auto &p : m.points) if (p.matched) d.push_back(p.gps_distance_m);
        return d;
    }

    // Full machine-readable report. All timings are milliseconds.
    struct WorkflowReport {
        std::string kitti_sequence;
        std::size_t gps_observations = 0;
        std::size_t matched_observations = 0;
        double matched_percentage = 0.0;

        std::size_t osm_node_count = 0;
        std::size_t osm_edge_count = 0;

        // HMM snap-distance metrics (metres).
        double hmm_mean_snap_m = 0.0;
        double hmm_p50_snap_m = 0.0;
        double hmm_p95_snap_m = 0.0;
        double hmm_max_snap_m = 0.0;
        std::size_t low_confidence_count = 0;
        std::size_t disconnected_transitions = 0;

        // Nearest-edge baseline snap-distance metrics (metres), kept separate.
        double baseline_mean_snap_m = 0.0;
        double baseline_p50_snap_m = 0.0;
        double baseline_p95_snap_m = 0.0;
        double baseline_max_snap_m = 0.0;

        // Timings (ms).
        double osm_build_ms = 0.0;
        double spatial_index_ms = 0.0;
        double baseline_runtime_ms = 0.0;
        double hmm_runtime_ms = 0.0;
        double total_runtime_ms = 0.0;

        // Environment.
        std::string compiler;
        std::string build_type;
        std::string cpu;
        std::string commit;
        std::string kitti_path;
        std::string osm_path;
    };

    namespace detail {
        inline std::string jstr(const std::string &s) {
            std::string out = "\"";
            for (char c : s) {
                if (c == '"' || c == '\\') { out += '\\'; out += c; }
                else if (c == '\n') out += "\\n";
                else out += c;
            }
            out += '"';
            return out;
        }
    }

    // Serialize the report to a JSON object string.
    inline std::string to_json(const WorkflowReport &r) {
        std::ostringstream os;
        os.setf(std::ios::fixed);
        os.precision(6);
        auto kv_s = [&](const char *k, const std::string &v, bool comma = true) {
            os << "  \"" << k << "\": " << detail::jstr(v) << (comma ? ",\n" : "\n");
        };
        auto kv_n = [&](const char *k, double v, bool comma = true) {
            os << "  \"" << k << "\": " << v << (comma ? ",\n" : "\n");
        };
        auto kv_u = [&](const char *k, std::size_t v, bool comma = true) {
            os << "  \"" << k << "\": " << v << (comma ? ",\n" : "\n");
        };

        os << "{\n";
        kv_s("kitti_sequence", r.kitti_sequence);
        kv_u("gps_observations", r.gps_observations);
        kv_u("matched_observations", r.matched_observations);
        kv_n("matched_percentage", r.matched_percentage);
        kv_u("osm_node_count", r.osm_node_count);
        kv_u("osm_edge_count", r.osm_edge_count);

        kv_n("hmm_mean_snap_m", r.hmm_mean_snap_m);
        kv_n("hmm_p50_snap_m", r.hmm_p50_snap_m);
        kv_n("hmm_p95_snap_m", r.hmm_p95_snap_m);
        kv_n("hmm_max_snap_m", r.hmm_max_snap_m);
        kv_u("low_confidence_count", r.low_confidence_count);
        kv_u("disconnected_transitions", r.disconnected_transitions);

        kv_n("baseline_mean_snap_m", r.baseline_mean_snap_m);
        kv_n("baseline_p50_snap_m", r.baseline_p50_snap_m);
        kv_n("baseline_p95_snap_m", r.baseline_p95_snap_m);
        kv_n("baseline_max_snap_m", r.baseline_max_snap_m);

        kv_n("osm_build_ms", r.osm_build_ms);
        kv_n("spatial_index_ms", r.spatial_index_ms);
        kv_n("baseline_runtime_ms", r.baseline_runtime_ms);
        kv_n("hmm_runtime_ms", r.hmm_runtime_ms);
        kv_n("total_runtime_ms", r.total_runtime_ms);

        kv_s("compiler", r.compiler);
        kv_s("build_type", r.build_type);
        kv_s("cpu", r.cpu);
        kv_s("commit", r.commit);
        kv_s("kitti_path", r.kitti_path);
        kv_s("osm_path", r.osm_path, /*comma=*/false);
        os << "}\n";
        return os.str();
    }

}

#endif //MAP_MATCHING_2_WORKFLOW_PIPELINE_REPORT_HPP
