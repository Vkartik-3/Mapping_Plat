// Copyright (C) 2025 Kartik Vadhawana
#include <gtest/gtest.h>

#include "geometry/road_graph/map_matcher.hpp"
#include "geometry/road_graph/graph_algorithms.hpp"
#include "util/synthetic_data.hpp"

using namespace map_matching_2;
using namespace map_matching_2::geometry::road_graph;

TEST(MapMatching, MatchesCleanTraceToExpectedEdges) {
    auto frames = util::synthetic::trajectory(40, 40, 5.0);
    auto graph = util::synthetic::road_graph(5.0, 40, 40);
    map_matcher matcher;
    auto r = matcher.match(graph, frames);
    EXPECT_EQ(r.matched_points, r.total_points);
    EXPECT_EQ(r.unmatched_points, 0u);
    EXPECT_DOUBLE_EQ(r.matching_rate, 1.0);
    EXPECT_GT(r.mean_confidence, 0.9);
    // first frames snap to edge 0 (east road), last to edge 1 (north road)
    EXPECT_EQ(r.points.front().edge_index, 0u);
    EXPECT_EQ(r.points.back().edge_index, 1u);
}

TEST(MapMatching, ProjectionOntoSegmentClamps) {
    Point2D p{5, 5};
    Point2D a{0, 0}, b{10, 0};
    double d = 0;
    auto proj = map_matcher::project_point_to_segment(p, a, b, d);
    EXPECT_DOUBLE_EQ(proj.e, 5.0);
    EXPECT_DOUBLE_EQ(proj.n, 0.0);
    EXPECT_DOUBLE_EQ(d, 5.0);
    // beyond the segment end -> clamps to b
    double d2 = 0;
    auto proj2 = map_matcher::project_point_to_segment({20, 1}, a, b, d2);
    EXPECT_DOUBLE_EQ(proj2.e, 10.0);
}

TEST(MapMatching, EmptyGraphYieldsNoMatches) {
    auto frames = util::synthetic::trajectory(10, 0, 5.0);
    RoadGraph empty;
    map_matcher matcher;
    auto r = matcher.match(empty, frames);
    EXPECT_EQ(r.matched_points, 0u);
    EXPECT_DOUBLE_EQ(r.matching_rate, 0.0);
}

TEST(GraphAlgorithms, DijkstraShortestPathOnGrid) {
    auto g = util::synthetic::grid_graph(5, 5, 20.0);
    // node 0 = (0,0), node 24 = (4,4). Manhattan distance = 8 * 20 = 160 m.
    auto sp = dijkstra(g, 0, 24);
    ASSERT_TRUE(sp.reachable);
    EXPECT_NEAR(sp.cost, 160.0, 1e-6);
    EXPECT_EQ(sp.nodes.front(), 0u);
    EXPECT_EQ(sp.nodes.back(), 24u);
}

TEST(GraphAlgorithms, NearestEdgeQuery) {
    auto g = util::synthetic::road_graph(5.0, 40, 40);
    double d = 0;
    // point right on the east road
    std::size_t e = nearest_edge(g, {50.0, 0.0}, d);
    EXPECT_EQ(e, 0u);
    EXPECT_NEAR(d, 0.0, 1e-6);
}
