// Copyright (C) 2025 Kartik Vadhawana
#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "geometry/road_graph/graph_algorithms.hpp"
#include "geometry/road_graph/map_matcher.hpp"
#include "geometry/road_graph/osm_importer.hpp"
#include "types/io/track/kitti_oxts_importer.hpp"
#include "util/synthetic_data.hpp"
#include "workflow/pipeline_report.hpp"

namespace fs = std::filesystem;
using namespace map_matching_2;
using namespace map_matching_2::geometry::road_graph;
using map_matching_2::io::track::kitti_oxts_importer;

// GPS and OSM must land in the exact same ENU frame when given the same anchor.
TEST(Workflow, SharedGpsOsmCoordinateFrame) {
    const double alat = 49.0, alon = 8.4, aalt = 116.0;
    const double plat = 49.0012, plon = 8.4021;

    // OSM node at (plat, plon).
    fs::path osm = fs::temp_directory_path() / "hdmap_wf_frame.osm";
    {
        std::ofstream o(osm);
        o.precision(10);
        o << "<?xml version='1.0'?>\n<osm version=\"0.6\">\n"
          << " <node id=\"1\" lat=\"" << plat << "\" lon=\"" << plon << "\"/>\n"
          << " <node id=\"2\" lat=\"49.0030\" lon=\"8.4021\"/>\n"
          << " <way id=\"10\">\n"
          << "  <nd ref=\"1\"/>\n"
          << "  <nd ref=\"2\"/>\n"
          << "  <tag k=\"highway\" v=\"residential\"/>\n"
          << " </way>\n</osm>\n";
    }
    OsmImportParams params;
    params.anchor_lat = alat; params.anchor_lon = alon; params.anchor_alt = aalt;
    RoadGraph g = osm_importer{params}.import_file(osm);
    ASSERT_GE(g.node_count(), 1u);

    double e, n, u;
    kitti_oxts_importer::to_enu(alat, alon, aalt, plat, plon, aalt, e, n, u);

    // node 1 is the first materialized node.
    EXPECT_NEAR(g.nodes[0].pos.e, e, 1e-6);
    EXPECT_NEAR(g.nodes[0].pos.n, n, 1e-6);
}

TEST(Workflow, NearestEdgeMatching) {
    auto g = util::synthetic::road_graph(5.0, 40, 40); // L-shape: east road then north
    double d = 0.0;
    // A point right on the east road (edge 0).
    std::size_t e = nearest_edge(g, {50.0, 0.0}, d);
    EXPECT_EQ(e, 0u);
    EXPECT_NEAR(d, 0.0, 1e-6);
}

TEST(Workflow, HmmOutputContinuity) {
    auto frames = util::synthetic::trajectory(40, 40, 5.0);
    auto g = util::synthetic::road_graph(5.0, 40, 40);
    map_matcher matcher;
    auto m = matcher.match(g, frames);
    EXPECT_EQ(m.matched_points, m.total_points);
    // A clean trace on connected edges must have no disconnected transitions.
    EXPECT_EQ(workflow::count_disconnected(g, m), 0u);
}

TEST(Workflow, SnapDistanceMetrics) {
    std::vector<double> d{0.0, 1.0, 2.0, 3.0, 4.0};
    auto s = workflow::snap_stats(d);
    EXPECT_EQ(s.count, 5u);
    EXPECT_DOUBLE_EQ(s.mean, 2.0);
    EXPECT_DOUBLE_EQ(s.p50, 2.0);
    EXPECT_DOUBLE_EQ(s.max_v, 4.0);
}

TEST(Workflow, PercentileCalculation) {
    std::vector<double> d{10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
    EXPECT_NEAR(workflow::percentile(d, 0.0), 10.0, 1e-9);
    EXPECT_NEAR(workflow::percentile(d, 100.0), 100.0, 1e-9);
    EXPECT_NEAR(workflow::percentile(d, 50.0), 55.0, 1e-9); // interpolated
    EXPECT_NEAR(workflow::percentile({42.0}, 95.0), 42.0, 1e-9);
    EXPECT_NEAR(workflow::percentile({}, 95.0), 0.0, 1e-9);
}

TEST(Workflow, JsonReportGeneration) {
    workflow::WorkflowReport r;
    r.kitti_sequence = "2011_09_26_drive_0001_sync";
    r.gps_observations = 108;
    r.matched_observations = 100;
    r.matched_percentage = 92.5;
    r.osm_node_count = 500;
    r.osm_edge_count = 700;
    r.hmm_p95_snap_m = 3.14;
    r.compiler = "GCC 13.3.0";
    const std::string j = workflow::to_json(r);
    EXPECT_NE(j.find("\"kitti_sequence\": \"2011_09_26_drive_0001_sync\""), std::string::npos);
    EXPECT_NE(j.find("\"gps_observations\": 108"), std::string::npos);
    EXPECT_NE(j.find("\"osm_edge_count\": 700"), std::string::npos);
    EXPECT_NE(j.find("\"hmm_p95_snap_m\": 3.14"), std::string::npos);
    EXPECT_EQ(j.front(), '{');
    EXPECT_NE(j.find('}'), std::string::npos);
}

TEST(Workflow, DisconnectedTransitionDetected) {
    // Two parallel, unconnected edges; a match that jumps between them counts.
    RoadGraph g;
    g.nodes.push_back({1, {0, 0}, 0, 0});
    g.nodes.push_back({2, {10, 0}, 0, 0});
    g.nodes.push_back({3, {0, 100}, 0, 0});   // far, separate edge
    g.nodes.push_back({4, {10, 100}, 0, 0});
    RoadEdge e0{}; e0.id = 0; e0.from = 0; e0.to = 1;
    e0.polyline = {g.nodes[0].pos, g.nodes[1].pos};
    RoadEdge e1{}; e1.id = 1; e1.from = 2; e1.to = 3;
    e1.polyline = {g.nodes[2].pos, g.nodes[3].pos};
    g.edges = {e0, e1};
    g.build_adjacency();

    MatchResult m;
    m.points.resize(2);
    m.points[0] = {true, 0, {5, 0}, 0.0, 1.0};
    m.points[1] = {true, 1, {5, 100}, 0.0, 1.0};
    EXPECT_EQ(workflow::count_disconnected(g, m), 1u);
}
