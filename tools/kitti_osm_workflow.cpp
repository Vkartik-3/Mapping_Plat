// Copyright (C) 2025 Kartik Vadhawana
//
// End-to-end real workflow:
//
//   real KITTI OXTS GPS trajectory
//     -> true WGS84 ECEF->ENU conversion (shared anchor)
//     -> real OSM road graph (same ENU anchor)
//     -> spatial index construction
//     -> nearest-edge baseline
//     -> HMM / Viterbi map matching
//     -> quality + performance JSON report
//     -> visualization CSV export (consumed by python/visualize_map.py)
//
// Paths come from CLI args or environment variables (KITTI_PATH, OSM_PATH,
// OUTPUT_PATH); no machine-specific paths are hardcoded. When KITTI_PATH or
// OSM_PATH is unavailable the workflow skips cleanly (exit 0) so CI stays green.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "geometry/coordinates/wgs84.hpp"
#include "geometry/road_graph/graph_algorithms.hpp"
#include "geometry/road_graph/map_matcher.hpp"
#include "geometry/road_graph/osm_importer.hpp"
#include "geometry/index/kdtree.hpp"
#include "types/io/lidar/kitti_lidar_reader.hpp"
#include "types/io/track/kitti_oxts_importer.hpp"
#include "workflow/pipeline_report.hpp"

#ifndef WORKFLOW_BUILD_TYPE
#define WORKFLOW_BUILD_TYPE "unknown"
#endif
#ifndef WORKFLOW_GIT_COMMIT
#define WORKFLOW_GIT_COMMIT "unknown"
#endif

namespace {
    using clock_t_ = std::chrono::steady_clock;
    double ms_since(clock_t_::time_point t0) {
        return std::chrono::duration<double, std::milli>(clock_t_::now() - t0).count();
    }

    std::string arg_or_env(int argc, char **argv, int idx, const char *env) {
        if (idx < argc && argv[idx][0] != '\0') return argv[idx];
        if (const char *e = std::getenv(env); e && *e) return e;
        return {};
    }

    std::string compiler_string() {
#if defined(__clang__)
        return std::string("Clang ") + __clang_version__;
#elif defined(__GNUC__)
        return "GCC " + std::to_string(__GNUC__) + "." + std::to_string(__GNUC_MINOR__) +
                "." + std::to_string(__GNUC_PATCHLEVEL__);
#else
        return "unknown";
#endif
    }

    std::string cpu_string() {
        std::ifstream in("/proc/cpuinfo");
        std::string line;
        while (std::getline(in, line)) {
            if (line.rfind("model name", 0) == 0) {
                const auto pos = line.find(':');
                if (pos != std::string::npos) return line.substr(pos + 2);
            }
        }
        return std::to_string(std::thread::hardware_concurrency()) + " logical CPUs";
    }

    void write_csv(const std::filesystem::path &p,
            const std::string &header, const std::vector<std::string> &rows) {
        std::ofstream out(p);
        out << header << "\n";
        for (const auto &r : rows) out << r << "\n";
    }
}

int main(int argc, char **argv) {
    namespace fs = std::filesystem;
    namespace rg = map_matching_2::geometry::road_graph;
    namespace coord = map_matching_2::geometry::coordinates;
    using map_matching_2::io::track::kitti_oxts_importer;

    const std::string kitti_path = arg_or_env(argc, argv, 1, "KITTI_PATH");
    const std::string osm_path = arg_or_env(argc, argv, 2, "OSM_PATH");
    std::string output_path = arg_or_env(argc, argv, 3, "OUTPUT_PATH");
    if (output_path.empty()) output_path = "workflow_output";

    const fs::path oxts_dir = fs::path(kitti_path) / "oxts" / "data";
    if (kitti_path.empty() || osm_path.empty() ||
            !fs::exists(oxts_dir) || !fs::exists(osm_path)) {
        std::cerr << "[SKIP] KITTI-OSM workflow: KITTI_PATH or OSM_PATH unavailable\n";
        return 0;
    }

    const auto t_total = clock_t_::now();

    // 1. Real KITTI OXTS trajectory (WGS84 ENU, anchored to first frame).
    auto seq = kitti_oxts_importer::load_sequence(oxts_dir);
    if (seq.frames.empty()) {
        std::cerr << "[SKIP] KITTI-OSM workflow: no OXTS frames parsed\n";
        return 0;
    }

    // 2. Real OSM road graph using the exact same ENU anchor.
    rg::OsmImportParams osm_params;
    osm_params.anchor_lat = seq.anchor_lat;
    osm_params.anchor_lon = seq.anchor_lon;
    osm_params.anchor_alt = seq.anchor_alt;
    rg::osm_importer importer{osm_params};

    const auto t_osm = clock_t_::now();
    rg::RoadGraph graph = importer.import_file(osm_path);
    const double osm_build_ms = ms_since(t_osm);

    // 3. Spatial index over the road-graph node geometry (ENU, z=0).
    const auto t_idx = clock_t_::now();
    std::vector<map_matching_2::io::lidar::LidarPoint> node_pts;
    node_pts.reserve(graph.nodes.size());
    for (const auto &n : graph.nodes) {
        node_pts.push_back({static_cast<float>(n.pos.e), static_cast<float>(n.pos.n), 0.0f, 0.0f});
    }
    map_matching_2::geometry::index::KDTree3D index;
    index.build(node_pts);
    const double spatial_index_ms = ms_since(t_idx);

    // 4. Nearest-edge baseline (brute force), kept separate from HMM.
    const auto t_base = clock_t_::now();
    std::vector<double> baseline_dists;
    baseline_dists.reserve(seq.frames.size());
    for (const auto &f : seq.frames) {
        double d = 0.0;
        rg::nearest_edge(graph, {f.easting, f.northing}, d);
        baseline_dists.push_back(d);
    }
    const double baseline_runtime_ms = ms_since(t_base);

    // 5. HMM / Viterbi map matching.
    const auto t_hmm = clock_t_::now();
    rg::map_matcher matcher;
    rg::MatchResult match = matcher.match(graph, seq.frames);
    const double hmm_runtime_ms = ms_since(t_hmm);

    // 6. Metrics.
    namespace wf = map_matching_2::workflow;
    const auto hmm_dists = wf::matched_snap_distances(match);
    const wf::SnapStats hmm = wf::snap_stats(hmm_dists);
    const wf::SnapStats base = wf::snap_stats(baseline_dists);
    constexpr double low_conf_threshold = 0.10;

    wf::WorkflowReport rep;
    rep.kitti_sequence = fs::path(kitti_path).filename().string();
    if (rep.kitti_sequence.empty()) rep.kitti_sequence = fs::path(kitti_path).parent_path().filename().string();
    rep.gps_observations = match.total_points;
    rep.matched_observations = match.matched_points;
    rep.matched_percentage = match.total_points == 0 ? 0.0
            : 100.0 * static_cast<double>(match.matched_points) / match.total_points;
    rep.osm_node_count = graph.node_count();
    rep.osm_edge_count = graph.edge_count();
    rep.hmm_mean_snap_m = hmm.mean;
    rep.hmm_p50_snap_m = hmm.p50;
    rep.hmm_p95_snap_m = hmm.p95;
    rep.hmm_max_snap_m = hmm.max_v;
    rep.low_confidence_count = wf::count_low_confidence(match, low_conf_threshold);
    rep.disconnected_transitions = wf::count_disconnected(graph, match);
    rep.baseline_mean_snap_m = base.mean;
    rep.baseline_p50_snap_m = base.p50;
    rep.baseline_p95_snap_m = base.p95;
    rep.baseline_max_snap_m = base.max_v;
    rep.osm_build_ms = osm_build_ms;
    rep.spatial_index_ms = spatial_index_ms;
    rep.baseline_runtime_ms = baseline_runtime_ms;
    rep.hmm_runtime_ms = hmm_runtime_ms;
    rep.compiler = compiler_string();
    rep.build_type = WORKFLOW_BUILD_TYPE;
    rep.cpu = cpu_string();
    rep.commit = WORKFLOW_GIT_COMMIT;
    rep.kitti_path = kitti_path;
    rep.osm_path = osm_path;
    rep.total_runtime_ms = ms_since(t_total);

    // 7. Write JSON report + visualization CSVs.
    fs::create_directories(output_path);
    const fs::path out_dir(output_path);
    { std::ofstream j(out_dir / "report.json"); j << wf::to_json(rep); }

    // Anchor frame for inverse ENU -> geodetic of snapped points.
    const coord::EnuReferenceFrame frame{
            coord::GeodeticCoordinate{seq.anchor_lat, seq.anchor_lon, seq.anchor_alt}};
    auto enu_to_lonlat = [&](const rg::Point2D &p) {
        const coord::GeodeticCoordinate g = frame.enuToGeodetic({p.e, p.n, 0.0});
        std::ostringstream os; os.precision(10);
        os << g.longitude_deg << "," << g.latitude_deg;
        return os.str();
    };

    std::vector<std::string> gps_rows, matched_rows, edge_rows, lowconf_rows, disc_rows;
    for (const auto &f : seq.frames) {
        std::ostringstream os; os.precision(10);
        os << f.lon << "," << f.lat;
        gps_rows.push_back(os.str());
    }
    for (const auto &e : graph.edges) {
        const auto &a = graph.nodes[e.from];
        const auto &b = graph.nodes[e.to];
        std::ostringstream os; os.precision(10);
        os << a.lon << "," << a.lat << "," << b.lon << "," << b.lat;
        edge_rows.push_back(os.str());
    }
    bool have_prev = false; std::size_t prev_edge = 0;
    for (const auto &p : match.points) {
        if (!p.matched) { have_prev = false; continue; }
        matched_rows.push_back(enu_to_lonlat(p.snapped));
        if (p.confidence < low_conf_threshold) lowconf_rows.push_back(enu_to_lonlat(p.snapped));
        if (have_prev && p.edge_index != prev_edge &&
                !wf::edges_share_node(graph, prev_edge, p.edge_index)) {
            disc_rows.push_back(enu_to_lonlat(p.snapped));
        }
        prev_edge = p.edge_index; have_prev = true;
    }
    write_csv(out_dir / "gps.csv", "lon,lat", gps_rows);
    write_csv(out_dir / "matched.csv", "lon,lat", matched_rows);
    write_csv(out_dir / "edges.csv", "lon1,lat1,lon2,lat2", edge_rows);
    write_csv(out_dir / "lowconf.csv", "lon,lat", lowconf_rows);
    write_csv(out_dir / "disconnected.csv", "lon,lat", disc_rows);

    std::cout << wf::to_json(rep);
    std::cout << "wrote report + visualization CSVs to " << out_dir.string() << "\n";
    return 0;
}
