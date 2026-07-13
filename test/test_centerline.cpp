// Copyright (C) 2025 Kartik Vadhawana
#include <gtest/gtest.h>

#include "geometry/centerline/centerline_extractor.hpp"
#include "util/synthetic_data.hpp"

using namespace map_matching_2;

TEST(Centerline, ExtractsNodesAndEdgesFromLTrajectory) {
    auto frames = util::synthetic::trajectory(40, 40, 5.0);
    geometry::centerline::centerline_extractor ext;
    auto net = ext.extract(frames);
    EXPECT_GT(net.graph.node_count(), 0u);
    EXPECT_GT(net.graph.edge_count(), 0u);
    EXPECT_GT(net.graph.total_length_meters(), 100.0);
}

TEST(Centerline, DetectsIntersectionAtCorner) {
    // An L-shaped path has two distinct heading regimes meeting at the corner.
    auto frames = util::synthetic::trajectory(40, 40, 5.0);
    geometry::centerline::centerline_extractor ext;
    auto net = ext.extract(frames);
    EXPECT_GE(net.intersection_count, 1u);
}

TEST(Centerline, HeadingOfCardinalDirections) {
    using ext = geometry::centerline::centerline_extractor;
    EXPECT_NEAR(ext::heading_of(0.0, 1.0), 0.0, 1e-6);    // north
    EXPECT_NEAR(ext::heading_of(1.0, 0.0), 90.0, 1e-6);   // east
    EXPECT_NEAR(ext::heading_of(0.0, -1.0), 180.0, 1e-6); // south
    EXPECT_NEAR(ext::heading_of(-1.0, 0.0), 270.0, 1e-6); // west
}

TEST(Centerline, EmptyOrTinyInputYieldsEmptyNetwork) {
    std::vector<io::track::KittiOxtsFrame> one(1);
    geometry::centerline::centerline_extractor ext;
    auto net = ext.extract(one);
    EXPECT_EQ(net.graph.node_count(), 0u);
    EXPECT_EQ(net.graph.edge_count(), 0u);
}
